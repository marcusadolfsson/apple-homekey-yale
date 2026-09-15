#pragma once

#include "NfcReader.hpp"
#include "RelayProtocol.hpp"

#include "esp_now.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <array>
#include <cstdint>
#include <vector>

/**
 * @brief INfcReader implementation that relays the card exchange to a remote
 * doorbell over ESP-NOW.
 *
 * The doorbell owns the NFC front end and nothing else: it holds no HomeKey
 * credentials and never joins WiFi. Each interface call becomes one
 * request/response over ESP-NOW, so the HomeKey transaction still runs here,
 * on mains power, while only APDUs cross the radio link.
 *
 * Pairing is automatic: the doorbell broadcasts pings while walking the
 * channels, and the first pong it receives identifies this base.
 */
class RemoteNfcReader : public INfcReader {
public:
  explicit RemoteNfcReader(const std::array<uint8_t, 18> &ecpData);
  ~RemoteNfcReader() override;

  bool init() override;
  void stop() override;
  bool isConnected() const override;

  uint8_t getFwMajor() const override { return 0; }
  uint8_t getFwMinor() const override { return 0; }

  bool beginDiscovery() override;
  bool pollForTag(std::vector<uint8_t> &uid, std::array<uint8_t, 2> &atqa, uint8_t &sak,
                  uint32_t timeoutMs) override;
  bool isTagStillPresent() override;
  void releaseTag() override;
  void endDiscovery() override;
  bool exchangeApdu(const std::vector<uint8_t> &send, std::vector<uint8_t> &recv,
                    uint32_t timeoutMs) override;
  bool healthCheck() override;
  bool updateECP() override { return true; }  // ECP travels with every poll

private:
  struct Msg {
    uint8_t mac[6];
    uint8_t data[relay::MAX_ESPNOW];
    uint16_t len;
  };
  struct Response {
    uint8_t op = 0, seq = 0, flags = 0;
    std::vector<uint8_t> payload;
  };

  static void recvTrampoline(const esp_now_recv_info_t *info, const uint8_t *data, int len);
  static void rxTaskEntry(void *arg);
  void rxTask();

  bool request(relay::Op op, const uint8_t *payload, size_t len, relay::Op expect,
               uint32_t timeoutMs, Response &out);
  void send(const uint8_t mac[6], relay::Op op, uint8_t seq, uint8_t flags, const uint8_t *payload,
            size_t len);

  static RemoteNfcReader *s_instance;

  const std::array<uint8_t, 18> &m_ecpData;
  uint8_t m_doorbell[6] = {0};
  bool m_doorbellKnown = false;
  uint8_t m_seq = 0;

  QueueHandle_t m_rx = nullptr;        // frames from the ESP-NOW callback
  QueueHandle_t m_responses = nullptr; // reassembled responses for the caller
  TaskHandle_t m_rxTask = nullptr;
  bool m_started = false;

  // Relay latency instrumentation for the test.
  uint32_t m_apduCount = 0, m_apduTotalUs = 0, m_apduMaxUs = 0;
  uint32_t m_pollsSent = 0, m_pollsAnswered = 0;
  bool m_readerReady = false;
  int64_t m_lastStatUs = 0;

  static constexpr const char *TAG = "RelayReader";
};
