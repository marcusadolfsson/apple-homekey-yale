#pragma once

#include "driver/i2c_master.h"
#include "pn532_cxx/transport.hpp"
#include "soc/gpio_num.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

/**
 * @brief PN532 I2C transport built on the ESP-IDF i2c_master driver.
 *
 * pn532_hal ships an I2cTransport, but it is marked unfinished and uses the
 * legacy driver/i2c.h API, which ESP-IDF refuses to link alongside the
 * i2c_master driver that St25r3916Reader uses. This implementation uses
 * i2c_master only.
 *
 * PN532 I2C protocol (address 0x24):
 *  - A command frame is written as-is, with no data-write prefix.
 *  - Every read returns a status byte first; bit 0 set means a frame is ready.
 *  - The PN532 re-serves the pending frame from its start on each read, so
 *    a frame cannot be read in pieces. prepareRead() reads the whole frame in
 *    one transfer and readChunk() hands it out sequentially.
 */
class Pn532I2cTransport : public pn532::Transport {
public:
  Pn532I2cTransport(gpio_num_t sda, gpio_num_t scl, uint32_t hz = 400000);
  ~Pn532I2cTransport() override;

  Pn532I2cTransport(const Pn532I2cTransport &) = delete;
  Pn532I2cTransport &operator=(const Pn532I2cTransport &) = delete;

  void swReset() override;
  void abort() override;
  pn532::Transaction begin() override;

protected:
  pn532::Status writeChunk(pn532::span<const uint8_t> data) override;
  bool waitReady(uint32_t timeout_ms) override;
  pn532::Status prepareRead() override;
  pn532::Status readChunk(pn532::span<uint8_t> buffer) override;
  void endTransaction() override;

private:
  static constexpr uint8_t ADDRESS = 0x24;
  static constexpr int XFER_TIMEOUT_MS = 50;
  // Status byte + largest PN532 frame (extended frame, 264 data bytes, plus
  // preamble/length/checksum/postamble overhead), with margin.
  static constexpr size_t RX_MAX = 1 + 300;
  static constexpr size_t ACK_LEN = 6;

  bool flushWrite();

  i2c_master_bus_handle_t m_bus = nullptr;
  i2c_master_dev_handle_t m_dev = nullptr;

  std::vector<uint8_t> m_tx;
  std::array<uint8_t, RX_MAX> m_rx{};
  size_t m_rxLen = 0;
  size_t m_rxPos = 0;
  // The first read after a command frame is its 6-byte ACK; reading the
  // full RX_MAX there would cost ~7 ms per command for nothing.
  bool m_expectAck = false;
};
