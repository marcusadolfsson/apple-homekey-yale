#pragma once

#include "NfcReader.hpp"
#include "RelayProtocol.hpp"

#include "esp_now.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <array>
#include <cstdint>
#include <string>
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
  RemoteNfcReader(const std::array<uint8_t, 18> &ecpData, const std::string &pinnedMac,
                  const std::string &linkKey, bool fastPolling);
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
  bool updateECP() override {
    sendEcp();  // reader identity changed: push it to the doorbell
    return true;
  }

  /** Link status for the web UI: is a doorbell paired, how strong is the link. */
  struct LinkStatus { bool paired; int8_t rssi; bool readerReady; uint16_t batteryMv; };
  static LinkStatus linkStatus() {
    if (!s_instance) return {false, 0, false, 0};
    return {s_instance->m_doorbellKnown, s_instance->m_linkRssi, s_instance->m_readerReady,
            s_instance->m_batteryMv};
  }

private:
  struct Msg {
    uint8_t mac[6];
    uint8_t data[relay::MAX_ESPNOW];
    uint16_t len;
  };
  struct Response {
    uint8_t op = 0, seq = 0, flags = 0;
    std::vector<uint8_t> payload;
    int64_t receivedUs = 0;  // tag events: so a stale one is never acted on (last: aggregate init)
  };
  // A tag announcement older than this describes a phone that has left. The
  // doorbell repeats the announcement for ~1 s, so anything beyond that is a
  // leftover from a period the radio was busy.
  static constexpr int64_t TAG_EVENT_MAX_AGE_US = 1500000;
  int64_t m_lastBusyDropLogUs = 0;

  static void recvTrampoline(const esp_now_recv_info_t *info, const uint8_t *data, int len);
  static void rxTaskEntry(void *arg);
  void rxTask();

  bool request(relay::Op op, const uint8_t *payload, size_t len, relay::Op expect,
               uint32_t timeoutMs, Response &out);
  void send(const uint8_t mac[6], relay::Op op, uint8_t seq, uint8_t flags, const uint8_t *payload,
            size_t len);
  void sendEcp();

  static RemoteNfcReader *s_instance;

  const std::array<uint8_t, 18> &m_ecpData;
  uint8_t m_doorbell[6] = {0};
  bool m_doorbellKnown = false;
  std::array<uint8_t, 6> m_pinnedMac{};
  bool m_pinned = false;              // only talk to m_pinnedMac
  std::array<uint8_t, 16> m_pmk{}, m_lmk{};
  bool m_encrypted = false;           // link key configured
  bool macAllowed(const uint8_t mac[6]) const;
  uint8_t m_seq = 0;

  QueueHandle_t m_rx = nullptr;        // frames from the ESP-NOW callback
  QueueHandle_t m_responses = nullptr; // replies to requests we sent
  // Unsolicited tap announcements get their own queue: the polling task waits on
  // this one continuously, and would otherwise consume replies meant for
  // exchangeApdu()/healthCheck().
  QueueHandle_t m_tagEvents = nullptr;
  TaskHandle_t m_rxTask = nullptr;
  bool m_started = false;

  // Relay latency instrumentation for the test.
  uint32_t m_apduCount = 0, m_apduTotalUs = 0, m_apduMaxUs = 0;
  uint32_t m_pollsSent = 0, m_pollsAnswered = 0;
  bool m_readerReady = false;
  int8_t m_linkRssi = 0;  // of the last frame heard from the doorbell
  uint16_t m_batteryMv = 0;  // 0 = unknown (no divider fitted / USB powered)
  void noteBattery(const uint8_t *tail, size_t len);
  // These mirror NfcManager::pollingTask() exactly, because the doorbell is
  // running the same poll loop remotely. LISTEN_WINDOW_MS is how long the
  // reader holds the field open waiting for a card: an iPhone needs most of
  // that to see the ECP frame and put the Home Key up, so shortening it is what
  // kills the tap animation. POLL_DELAY_MS is the quiet gap between cycles,
  // which is the knob "fast polling" turns and the one a battery build raises.
  static constexpr uint16_t LISTEN_WINDOW_MS = 500;
  // Tighter than the base's own 100 ms: the doorbell's only job is polling,
  // and a higher ECP rate is what gives an iPhone a chance to raise the Home
  // Key during a brief tap. A battery build raises this again.
  static constexpr uint16_t POLL_DELAY_MS = 20;
  static constexpr uint16_t POLL_DELAY_FAST_MS = 5;
  const bool m_fastPolling;
  static constexpr int64_t HEARTBEAT_TIMEOUT_MS = 90000;
  int64_t m_lastHeardUs = 0;
  int64_t m_lastSilenceLogUs = 0;
  int64_t m_lastStatUs = 0;
  int64_t m_lastHelloUs = 0;

  static constexpr const char *TAG = "RelayReader";
};
