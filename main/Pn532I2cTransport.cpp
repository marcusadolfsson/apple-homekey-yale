#include "Pn532I2cTransport.hpp"

#include "esp_log.h"
#include "fmt/ranges.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <algorithm>
#include <string>

static const char *TAG = "PN532::I2C";

namespace {
constexpr uint8_t ACK_FRAME[6] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
}

Pn532I2cTransport::Pn532I2cTransport(gpio_num_t sda, gpio_num_t scl, uint32_t hz) {
  i2c_master_bus_config_t buscfg = {};
  // -1 lets the driver pick a free port, as St25r3916Reader does.
  buscfg.i2c_port = -1;
  buscfg.sda_io_num = sda;
  buscfg.scl_io_num = scl;
  buscfg.clk_source = I2C_CLK_SRC_DEFAULT;
  buscfg.glitch_ignore_cnt = 7;
  // The PN532 V3 board has 4.7k pull-ups; the internal ones are harmless
  // and cover modules that ship without.
  buscfg.flags.enable_internal_pullup = true;

  esp_err_t err = i2c_new_master_bus(&buscfg, &m_bus);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
    m_bus = nullptr;
    return;
  }

  i2c_device_config_t devcfg = {};
  devcfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  devcfg.device_address = ADDRESS;
  devcfg.scl_speed_hz = hz;
  err = i2c_master_bus_add_device(m_bus, &devcfg, &m_dev);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
    m_dev = nullptr;
  }
  m_tx.reserve(64);

  // Bring-up diagnostics: report the pins in use and what answers on the bus.
  ESP_LOGW(TAG, "PN532 I2C on SDA=%d SCL=%d @ %lu Hz", sda, scl, (unsigned long)hz);
  std::string found;
  for (uint16_t addr = 0x08; addr < 0x78; ++addr) {
    if (i2c_master_probe(m_bus, addr, 20) == ESP_OK) {
      found += fmt::format(" 0x{:02X}", addr);
    }
  }
  if (found.empty()) {
    ESP_LOGE(TAG, "I2C scan: no devices answered (check wiring, VCC, and that DIP is in I2C mode then power-cycle)");
  } else {
    ESP_LOGW(TAG, "I2C scan found:%s%s", found.c_str(),
             found.find("0x24") == std::string::npos ? " (PN532 expected at 0x24)" : "");
  }
}

Pn532I2cTransport::~Pn532I2cTransport() {
  if (m_dev) {
    i2c_master_bus_rm_device(m_dev);
    m_dev = nullptr;
  }
  if (m_bus) {
    i2c_del_master_bus(m_bus);
    m_bus = nullptr;
  }
}

void Pn532I2cTransport::swReset() {
  // A PN532 in low-power mode NACKs the first access while it wakes, so
  // retry the abort a few times before giving up.
  for (int i = 0; i < 3; ++i) {
    if (m_dev && i2c_master_transmit(m_dev, ACK_FRAME, sizeof(ACK_FRAME), XFER_TIMEOUT_MS) == ESP_OK) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  m_tx.clear();
  m_rxLen = m_rxPos = 0;
  m_expectAck = false;
}

void Pn532I2cTransport::abort() {
  if (m_dev) {
    // An ACK frame sent to the PN532 aborts the command in progress.
    (void)i2c_master_transmit(m_dev, ACK_FRAME, sizeof(ACK_FRAME), XFER_TIMEOUT_MS);
  }
  m_tx.clear();
  m_rxLen = m_rxPos = 0;
  m_expectAck = false;
}

pn532::Transaction Pn532I2cTransport::begin() {
  m_tx.clear();
  m_rxLen = m_rxPos = 0;
  m_expectAck = false;
  return pn532::Transaction(*this, m_dev != nullptr);
}

pn532::Status Pn532I2cTransport::writeChunk(pn532::span<const uint8_t> data) {
  // Buffered and sent as one I2C write in flushWrite(): the PN532 treats each
  // I2C write as a complete frame, so chunks must not go out separately.
  m_tx.insert(m_tx.end(), data.begin(), data.end());
  return pn532::Status::SUCCESS;
}

bool Pn532I2cTransport::flushWrite() {
  if (m_tx.empty()) {
    return true;
  }
  esp_err_t err = ESP_FAIL;
  for (int i = 0; i < 3; ++i) {
    err = i2c_master_transmit(m_dev, m_tx.data(), m_tx.size(), XFER_TIMEOUT_MS);
    if (err == ESP_OK) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "write failed: 0x%x (%s)", err, esp_err_to_name(err));
    m_tx.clear();
    return false;
  }
  ESP_LOGV(TAG, "write: len=%d, data=%s", m_tx.size(),
           fmt::format("{:02X}", fmt::join(m_tx, "")).c_str());
  m_tx.clear();
  m_expectAck = true;
  return true;
}

bool Pn532I2cTransport::waitReady(uint32_t timeout_ms) {
  if (!flushWrite()) {
    return false;
  }
  const TickType_t start = xTaskGetTickCount();
  const TickType_t delay = std::max<TickType_t>(1, pdMS_TO_TICKS(1));
  uint8_t status = 0;
  while (true) {
    if (i2c_master_receive(m_dev, &status, 1, XFER_TIMEOUT_MS) == ESP_OK && (status & 0x01)) {
      return true;
    }
    if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) {
      ESP_LOGV(TAG, "Timeout waiting for RDY");
      abort();
      return false;
    }
    vTaskDelay(delay);
  }
}

pn532::Status Pn532I2cTransport::prepareRead() {
  const size_t len = m_expectAck ? 1 + ACK_LEN : RX_MAX;
  m_expectAck = false;
  esp_err_t err = i2c_master_receive(m_dev, m_rx.data(), len, XFER_TIMEOUT_MS);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "read failed: 0x%x (%s)", err, esp_err_to_name(err));
    m_rxLen = m_rxPos = 0;
    return pn532::Status::TRANSPORT_ERROR;
  }
  if (!(m_rx[0] & 0x01)) {
    ESP_LOGE(TAG, "read: PN532 not ready (status 0x%02X)", m_rx[0]);
    m_rxLen = m_rxPos = 0;
    return pn532::Status::TRANSPORT_ERROR;
  }
  m_rxLen = len;
  m_rxPos = 1;  // skip the status byte
  return pn532::Status::SUCCESS;
}

pn532::Status Pn532I2cTransport::readChunk(pn532::span<uint8_t> buffer) {
  if (m_rxPos + buffer.size() > m_rxLen) {
    ESP_LOGE(TAG, "readChunk: %d bytes requested, %d buffered", buffer.size(), m_rxLen - m_rxPos);
    return pn532::Status::TRANSPORT_ERROR;
  }
  std::copy_n(m_rx.data() + m_rxPos, buffer.size(), buffer.data());
  m_rxPos += buffer.size();
  ESP_LOGV(TAG, "readChunk: len=%d, data=%s", buffer.size(),
           fmt::format("{:02X}", fmt::join(buffer, "")).c_str());
  return pn532::Status::SUCCESS;
}

void Pn532I2cTransport::endTransaction() {
  // Transaction::waitForAck() ends the transaction before waiting, which is
  // where the buffered command frame has to go out.
  (void)flushWrite();
}
