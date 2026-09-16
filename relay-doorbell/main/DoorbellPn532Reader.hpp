#pragma once

// PN532 (I2C) behind the same INfcReader interface the base uses, so the
// doorbell can carry either reader and main.cpp does not care which.
//
// This is the poll sequence that took a night to get right, kept verbatim:
// reset CIU_BitFraming, transmit the ECP frame raw, then listen. See the
// comments inline and the Express-mode section of CLAUDE.md.

#include "NfcReader.hpp"
#include "Pn532I2cTransport.hpp"
#include "pn532_cxx/pn532.hpp"

#include "esp_log.h"
#include "esp_timer.h"

#include <array>
#include <cstdint>
#include <vector>

class DoorbellPn532Reader : public INfcReader {
public:
  DoorbellPn532Reader(gpio_num_t sda, gpio_num_t scl, const std::array<uint8_t, 18> &ecpData)
      : m_sda(sda), m_scl(scl), m_ecpData(ecpData) {}
  ~DoorbellPn532Reader() override { stop(); }

  bool init() override {
    if (!m_transport) m_transport = new Pn532I2cTransport(m_sda, m_scl);
    if (!m_pn532) m_pn532 = new pn532::Frontend(*m_transport);
    if (m_pn532->begin() != pn532::Status::SUCCESS) ESP_LOGW(TAG, "PN532 begin reported an error");
    auto ver = m_pn532->GetFirmwareVersion();
    if (!ver) {
      ESP_LOGE(TAG, "no PN532 found on SDA=%d SCL=%d", m_sda, m_scl);
      return false;
    }
    m_fwMajor = uint8_t((*ver >> 24) & 0xFF);
    m_fwMinor = uint8_t((*ver >> 16) & 0xFF);
    ESP_LOGI(TAG, "PN532 firmware %u.%u", m_fwMajor, m_fwMinor);
    // Same setup the base's own PN532 driver uses.
    m_pn532->RFConfiguration(0x01, {0x03});
    m_pn532->setPassiveActivationRetries(0);
    m_pn532->RFConfiguration(0x02, {0x00, 0x0B, 0x10});
    m_pn532->RFConfiguration(0x04, {0xFF});
    m_connected = true;
    return true;
  }

  void stop() override {
    delete m_pn532; m_pn532 = nullptr;
    delete m_transport; m_transport = nullptr;
    m_connected = false;
  }

  bool isConnected() const override { return m_connected; }
  uint8_t getFwMajor() const override { return m_fwMajor; }
  uint8_t getFwMinor() const override { return m_fwMinor; }

  bool beginDiscovery() override {
    if (m_pn532) (void)m_pn532->setPassiveActivationRetries(0);
    return true;
  }
  void endDiscovery() override {}

  bool pollForTag(std::vector<uint8_t> &uid, std::array<uint8_t, 2> &atqa, uint8_t &sak,
                  uint32_t timeoutMs) override {
    if (!m_pn532) return false;
    // Reset CIU_BitFraming (0x633D) before the raw ECP transmit. Anticollision
    // sends REQA/WUPA as 7-bit short frames and can leave TxLastBits = 7, which
    // makes InCommunicateThru send the ECP frame's last byte as 7 bits: the phone
    // fails the CRC and ignores it, so Express mode never triggers while a
    // deliberate Wallet tap still works.
    (void)m_pn532->WriteRegister({0x63, 0x3d, 0x00});
    std::vector<uint8_t> res;
    // Nothing ever answers an ECP frame, so the whole timeout is dead time in
    // every cycle; 20 ms keeps the rate up (~15 Hz).
    const int64_t ecpStart = esp_timer_get_time();
    const auto ecpStatus = m_pn532->InCommunicateThru(m_ecpData, res, 20);
    const int64_t ecpDone = esp_timer_get_time();
    if (int(ecpStatus) != m_lastEcpStatus) {
      m_lastEcpStatus = int(ecpStatus);
      ESP_LOGI(TAG, "ECP transmit status now %d", m_lastEcpStatus);
    }
    // How often the phone actually sees an ECP frame is the whole game for a
    // background tap, so measure the cycle rather than assuming it.
    if (m_lastCycleUs) { m_sumCycle += ecpStart - m_lastCycleUs; m_sumEcp += ecpDone - ecpStart; ++m_cycles; }
    m_lastCycleUs = ecpStart;
    if (m_cycles == 1000) {
      ESP_LOGD(TAG, "poll cycle: %lld ms avg (ECP transmit %lld ms of it) -> ECP at %.1f Hz",
               m_sumCycle / m_cycles / 1000, m_sumEcp / m_cycles / 1000, 1000000.0 * m_cycles / double(m_sumCycle));
      m_cycles = 0; m_sumCycle = 0; m_sumEcp = 0;
    }
    const auto st = m_pn532->InListPassiveTarget(0x00, uid, atqa, sak, uint16_t(timeoutMs));
    // "No card" is the normal answer. Anything else, repeatedly, means the bus
    // or the chip is gone; report disconnected so main.cpp re-initialises
    // instead of logging an error 15 times a second forever.
    if (st == pn532::Status::SUCCESS || st == pn532::Status::TIMEOUT || st == pn532::Status::NO_TAGS_FOUND) {
      m_consecutiveFailures = 0;
    } else if (++m_consecutiveFailures >= 20) {
      ESP_LOGE(TAG, "PN532 not responding (%d consecutive bus errors)", m_consecutiveFailures);
      m_consecutiveFailures = 0;
      m_connected = false;
    }
    return st == pn532::Status::SUCCESS;
  }

  bool isTagStillPresent() override {
    if (!m_pn532) return false;
    (void)m_pn532->InRelease(1);
    std::vector<uint8_t> uid; std::array<uint8_t, 2> atqa{}; uint8_t sak = 0;
    return m_pn532->InListPassiveTarget(0x00, uid, atqa, sak, 100) == pn532::Status::SUCCESS;
  }

  void releaseTag() override {
    if (!m_pn532) return;
    (void)m_pn532->InRelease(1);
    (void)m_pn532->setPassiveActivationRetries(0);
  }

  bool exchangeApdu(const std::vector<uint8_t> &send, std::vector<uint8_t> &recv,
                    uint32_t timeoutMs) override {
    if (!m_pn532) return false;
    const bool ok = m_pn532->InDataExchange(send, recv, uint16_t(timeoutMs)) == pn532::Status::SUCCESS;
    if (ok && recv.size() >= 2) recv.erase(recv.begin(), recv.begin() + 2);  // strip PN532 status
    return ok;
  }

  bool healthCheck() override {
    return m_pn532 && m_pn532->WriteRegister({0x63, 0x3d, 0x0}) == pn532::Status::SUCCESS;
  }

  bool updateECP() override { return true; }  // held by reference

private:
  static constexpr const char *TAG = "doorbell-pn532";
  gpio_num_t m_sda, m_scl;
  const std::array<uint8_t, 18> &m_ecpData;
  Pn532I2cTransport *m_transport = nullptr;
  pn532::Frontend *m_pn532 = nullptr;
  bool m_connected = false;
  uint8_t m_fwMajor = 0, m_fwMinor = 0;
  int m_lastEcpStatus = -1;
  int64_t m_lastCycleUs = 0, m_sumCycle = 0, m_sumEcp = 0;
  int m_cycles = 0;
  int m_consecutiveFailures = 0;
};
