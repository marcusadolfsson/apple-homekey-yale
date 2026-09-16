#include "RemoteNfcReader.hpp"

#include "YaleBleLock.hpp"
#include "app_event_loop.hpp"
#include "app_events.hpp"
#include "eventStructs.hpp"

#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include <algorithm>
#include <cstring>
#include <string>

RemoteNfcReader *RemoteNfcReader::s_instance = nullptr;

namespace {
const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

bool addPeer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p{};
  std::memcpy(p.peer_addr, mac, 6);
  p.channel = 0;  // whatever channel the STA is on
  p.ifidx = WIFI_IF_STA;
  p.encrypt = false;
  return esp_now_add_peer(&p) == ESP_OK;
}
}  // namespace

RemoteNfcReader::RemoteNfcReader(const std::array<uint8_t, 18> &ecpData) : m_ecpData(ecpData) {
  s_instance = this;
}

RemoteNfcReader::~RemoteNfcReader() {
  stop();
  s_instance = nullptr;
}

void RemoteNfcReader::recvTrampoline(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  auto *self = s_instance;
  if (!self || !self->m_rx || len < (int)relay::HDR) return;
  Msg m{};
  if (info->rx_ctrl) self->m_linkRssi = info->rx_ctrl->rssi;
  self->m_lastHeardUs = esp_timer_get_time();
  std::memcpy(m.mac, info->src_addr, 6);
  m.len = uint16_t(std::min<size_t>(len, relay::MAX_ESPNOW));
  std::memcpy(m.data, data, m.len);
  xQueueSend(self->m_rx, &m, 0);
}

bool RemoteNfcReader::init() {
  if (m_started) return true;
  // The NFC reader is constructed before HomeSpan brings WiFi up, and
  // esp_now_init() faults if the WiFi driver has not started yet. Report
  // "not ready" so NfcManager retries; by then WiFi is running.
  wifi_mode_t mode;
  if (esp_wifi_get_mode(&mode) != ESP_OK) {
    ESP_LOGI(TAG, "waiting for WiFi to start before enabling ESP-NOW");
    return false;
  }
  if (!m_rx) m_rx = xQueueCreate(16, sizeof(Msg));
  if (!m_responses) m_responses = xQueueCreate(8, sizeof(Response *));
  if (!m_tagEvents) m_tagEvents = xQueueCreate(4, sizeof(Response *));
  esp_err_t err = esp_now_init();
  if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
    ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
    return false;
  }
  esp_now_register_recv_cb(recvTrampoline);
  addPeer(BROADCAST);
  // Leave WiFi power save at its default. Forcing WIFI_PS_NONE keeps the WiFi
  // receiver on, which shares one radio with BLE and made connecting to the lock
  // 3-4x slower (1.9 s -> 4.8-8.0 s) while saving only ~10 ms of relay latency.
  xTaskCreate(rxTaskEntry, "relay_rx", 4096, this, 6, &m_rxTask);
  m_started = true;

  uint8_t mac[6] = {0};
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  uint8_t ch = 0; wifi_second_chan_t sec{};
  esp_wifi_get_channel(&ch, &sec);
  ESP_LOGI(TAG, "relay base ready on channel %u, MAC %02X:%02X:%02X:%02X:%02X:%02X; waiting for a doorbell",
           ch, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return true;
}

void RemoteNfcReader::stop() {
  if (!m_started) return;
  esp_now_unregister_recv_cb();
  if (m_rxTask) { vTaskDelete(m_rxTask); m_rxTask = nullptr; }
  if (m_rx) { vQueueDelete(m_rx); m_rx = nullptr; }
  if (m_responses) { vQueueDelete(m_responses); m_responses = nullptr; }
  if (m_tagEvents) { vQueueDelete(m_tagEvents); m_tagEvents = nullptr; }
  m_started = false;
  m_doorbellKnown = false;
}

// The link is connectionless: "connected" means ESP-NOW is up. Whether a
// doorbell has paired yet is reported separately, since pairing is asynchronous.
bool RemoteNfcReader::isConnected() const { return m_started; }

void RemoteNfcReader::rxTaskEntry(void *arg) { static_cast<RemoteNfcReader *>(arg)->rxTask(); }

void RemoteNfcReader::rxTask() {
  // Reassembly state for the single response in flight.
  uint8_t op = 0, seq = 0, fragCnt = 0, got = 0;
  std::vector<uint8_t> buf;

  while (true) {
    Msg m{};
    if (xQueueReceive(m_rx, &m, portMAX_DELAY) != pdTRUE) continue;
    relay::Header h{};
    std::memcpy(&h, m.data, relay::HDR);
    if (h.magic0 != relay::MAGIC0 || h.magic1 != relay::MAGIC1 || h.version != relay::VERSION) continue;

    if (relay::Op(h.op) == relay::Op::Pong) {
      addPeer(m.mac);
      if (!m_doorbellKnown) {
        std::memcpy(m_doorbell, m.mac, 6);
        m_doorbellKnown = true;
        m_readerReady = (h.flags & 2) != 0;
        ESP_LOGI(TAG, "doorbell answered our hello: %02X:%02X:%02X:%02X:%02X:%02X", m_doorbell[0],
                 m_doorbell[1], m_doorbell[2], m_doorbell[3], m_doorbell[4], m_doorbell[5]);
        sendEcp();
      }
      continue;
    }
    if (relay::Op(h.op) == relay::Op::Ping) {
      // A doorbell is looking for a base: answer, and adopt the first one.
      addPeer(m.mac);
      send(m.mac, relay::Op::Pong, h.seq, 0, nullptr, 0);
      if (!m_doorbellKnown) {
        std::memcpy(m_doorbell, m.mac, 6);
        m_doorbellKnown = true;
        m_readerReady = (h.flags & 2) != 0;
        ESP_LOGI(TAG, "doorbell paired: %02X:%02X:%02X:%02X:%02X:%02X", m_doorbell[0], m_doorbell[1],
                 m_doorbell[2], m_doorbell[3], m_doorbell[4], m_doorbell[5]);
        sendEcp();
      }
      continue;
    }

    if (relay::Op(h.op) == relay::Op::HealthRsp) {
      m_readerReady = (h.flags & 2) != 0;  // heartbeat
      noteBattery(m.data + relay::HDR, m.len - relay::HDR);
      continue;
    }
    if (relay::Op(h.op) == relay::Op::ButtonPress) {
      noteBattery(m.data + relay::HDR, m.len - relay::HDR);
      ESP_LOGI(TAG, "doorbell button pressed");
      AppEventLoop::publish(HW_EVENT, HW_DOORBELL_BUTTON, nullptr, 0);
      continue;
    }
    if (relay::Op(h.op) == relay::Op::EcpReq) {
      ESP_LOGI(TAG, "doorbell asked for ECP data");
      sendEcp();
      continue;
    }

    std::vector<uint8_t> payload;
    if (h.fragCnt <= 1) {
      payload.assign(m.data + relay::HDR, m.data + m.len);
    } else {
      if (op != h.op || seq != h.seq) {
        op = h.op; seq = h.seq; fragCnt = h.fragCnt; got = 0;
        buf.assign(h.totalLen, 0);
      }
      const size_t off = size_t(h.fragIdx) * relay::MAX_PAYLOAD;
      const size_t n = m.len - relay::HDR;
      if (off + n <= buf.size()) std::memcpy(buf.data() + off, m.data + relay::HDR, n);
      if (++got < fragCnt) continue;
      payload = buf;
      op = 0;
    }

    auto *r = new Response{h.op, h.seq, h.flags, std::move(payload)};
    QueueHandle_t q = relay::Op(h.op) == relay::Op::TagEvent ? m_tagEvents : m_responses;
    if (xQueueSend(q, &r, 0) != pdTRUE) delete r;  // nobody waiting
  }
}

void RemoteNfcReader::send(const uint8_t mac[6], relay::Op op, uint8_t seq, uint8_t flags,
                           const uint8_t *payload, size_t len) {
  addPeer(mac);
  const size_t fragCnt = len == 0 ? 1 : (len + relay::MAX_PAYLOAD - 1) / relay::MAX_PAYLOAD;
  for (size_t i = 0; i < fragCnt; ++i) {
    uint8_t frame[relay::MAX_ESPNOW];
    auto *h = reinterpret_cast<relay::Header *>(frame);
    h->magic0 = relay::MAGIC0; h->magic1 = relay::MAGIC1; h->version = relay::VERSION;
    h->op = uint8_t(op); h->seq = seq; h->fragIdx = uint8_t(i); h->fragCnt = uint8_t(fragCnt);
    h->flags = flags; h->totalLen = uint16_t(len);
    const size_t off = i * relay::MAX_PAYLOAD;
    const size_t n = std::min(relay::MAX_PAYLOAD, len - off);
    if (n) std::memcpy(frame + relay::HDR, payload + off, n);
    esp_now_send(mac, frame, relay::HDR + n);
  }
}

bool RemoteNfcReader::request(relay::Op op, const uint8_t *payload, size_t len, relay::Op expect,
                              uint32_t timeoutMs, Response &out) {
  if (!m_doorbellKnown) return false;
  const uint8_t seq = ++m_seq;
  // Drop any stale response still queued from a timed-out request.
  Response *stale = nullptr;
  while (xQueueReceive(m_responses, &stale, 0) == pdTRUE) delete stale;

  send(m_doorbell, op, seq, 0, payload, len);
  const int64_t deadline = esp_timer_get_time() + int64_t(timeoutMs) * 1000;
  while (true) {
    const int64_t remainMs = (deadline - esp_timer_get_time()) / 1000;
    if (remainMs <= 0) return false;
    Response *r = nullptr;
    if (xQueueReceive(m_responses, &r, pdMS_TO_TICKS(remainMs)) != pdTRUE) return false;
    const bool match = r->op == uint8_t(expect) && r->seq == seq;
    if (match) { out = std::move(*r); delete r; return true; }
    delete r;  // late or unrelated
  }
}

bool RemoteNfcReader::beginDiscovery() {
  if (!m_doorbellKnown) ESP_LOGI(TAG, "no doorbell paired yet; waiting for its ping");
  return true;  // the doorbell may pair at any time
}
void RemoteNfcReader::endDiscovery() {}

bool RemoteNfcReader::pollForTag(std::vector<uint8_t> &uid, std::array<uint8_t, 2> &atqa,
                                 uint8_t &sak, uint32_t timeoutMs) {
  if (g_bleRadioBusy.load(std::memory_order_acquire)) {
    vTaskDelay(pdMS_TO_TICKS(100));
    return false;
  }
  if (m_doorbellKnown) {
    // Inverted polling: the doorbell watches its own reader and announces a card,
    // so the base sends nothing at all between taps.
    Response *r = nullptr;
    if (xQueueReceive(m_tagEvents, &r, pdMS_TO_TICKS(timeoutMs)) != pdTRUE) return false;
    const std::vector<uint8_t> p = std::move(r->payload);
    delete r;
    if (p.empty() || p.size() < size_t(1 + p[0] + 3)) return false;
    if (p.size() >= size_t(1 + p[0] + 3) + 2) noteBattery(p.data(), p.size());
    uid.assign(p.begin() + 1, p.begin() + 1 + p[0]);
    atqa[0] = p[1 + p[0]];
    atqa[1] = p[2 + p[0]];
    sak = p[3 + p[0]];
    m_apduCount = m_apduTotalUs = m_apduMaxUs = 0;
    return true;
  }
  if (!m_doorbellKnown) {
    // The doorbell stops pinging once paired, so after a base restart nobody is
    // looking for anybody. Advertise until a doorbell answers.
    const int64_t now = esp_timer_get_time();
    if (now - m_lastHelloUs > 3000000) {
      m_lastHelloUs = now;
      send(BROADCAST, relay::Op::Ping, ++m_seq, 0, nullptr, 0);
    }
  }
  vTaskDelay(pdMS_TO_TICKS(200));
  return false;
}

// Battery voltage rides on the tail of frames the doorbell already sends, so a
// sleepy doorbell reports every time it wakes and never transmits just for this.
void RemoteNfcReader::noteBattery(const uint8_t *tail, size_t len) {
  if (len < 2) return;
  const uint16_t mv = uint16_t(tail[len - 2]) | uint16_t(tail[len - 1]) << 8;
  if (mv == 0 || mv > 6000) return;  // 0 = no divider fitted
  const bool firstReport = m_batteryMv == 0;
  const bool moved = m_batteryMv && (mv > m_batteryMv + 50 || mv + 50 < m_batteryMv);
  m_batteryMv = mv;
  if (firstReport || moved) {
    ESP_LOGI(TAG, "doorbell battery %u mV", mv);
    // EventValueChanged carries bytes, so millivolts travel in its string field.
    EventValueChanged ev{};
    ev.name = "doorbellBatteryMv";
    ev.str = std::to_string(mv);
    std::array<uint8_t, 64> d{};
    size_t n = alpaca::serialize(ev, d);
    AppEventLoop::publish(HW_EVENT, HW_DOORBELL_BATTERY, d.data(), n);
  }
}

void RemoteNfcReader::sendEcp() {
  if (!m_doorbellKnown) return;
  uint8_t payload[20];
  std::memcpy(payload, m_ecpData.data(), 18);
  const uint16_t interval = POLL_INTERVAL_MS;
  std::memcpy(payload + 18, &interval, 2);
  send(m_doorbell, relay::Op::EcpSet, ++m_seq, 0, payload, sizeof(payload));
}

bool RemoteNfcReader::exchangeApdu(const std::vector<uint8_t> &send_, std::vector<uint8_t> &recv,
                                   uint32_t timeoutMs) {
  std::vector<uint8_t> req(4 + send_.size());
  const uint32_t t = timeoutMs;
  std::memcpy(req.data(), &t, 4);
  std::copy(send_.begin(), send_.end(), req.begin() + 4);

  const int64_t t0 = esp_timer_get_time();
  Response rsp;
  if (!request(relay::Op::ApduReq, req.data(), req.size(), relay::Op::ApduRsp, timeoutMs + 400, rsp)) {
    ESP_LOGW(TAG, "apdu relay timed out (%u bytes out)", (unsigned)send_.size());
    return false;
  }
  const uint32_t us = uint32_t(esp_timer_get_time() - t0);
  m_apduCount++;
  m_apduTotalUs += us;
  m_apduMaxUs = std::max(m_apduMaxUs, us);
  ESP_LOGI(TAG, "apdu %u -> %u bytes, relay round trip %u us", (unsigned)send_.size(),
           (unsigned)rsp.payload.size(), us);
  if (!rsp.flags) return false;
  recv = std::move(rsp.payload);
  return true;
}

bool RemoteNfcReader::isTagStillPresent() {
  if (g_bleRadioBusy.load(std::memory_order_acquire)) return false;
  Response rsp;
  if (!request(relay::Op::PresentReq, nullptr, 0, relay::Op::PresentRsp, 500, rsp)) return false;
  return rsp.flags != 0;
}

void RemoteNfcReader::releaseTag() {
  if (g_bleRadioBusy.load(std::memory_order_acquire)) return;
  Response rsp;
  (void)request(relay::Op::ReleaseReq, nullptr, 0, relay::Op::ReleaseRsp, 500, rsp);
  if (m_apduCount) {
    ESP_LOGI(TAG, "relay summary: %u APDUs, avg %u us, max %u us, total %u ms added by the link",
             m_apduCount, m_apduTotalUs / m_apduCount, m_apduMaxUs, m_apduTotalUs / 1000);
    m_apduCount = m_apduTotalUs = m_apduMaxUs = 0;
  }
}

bool RemoteNfcReader::healthCheck() {
  // With the doorbell announcing taps, the base sends nothing between them, so
  // liveness comes from the doorbell's own heartbeat rather than from a poll of
  // ours. Asking over the air would defeat the point of the inversion (and the
  // reply raced the tap-announcement wait).
  if (!m_doorbellKnown) return true;
  const int64_t silentMs = (esp_timer_get_time() - m_lastHeardUs) / 1000;
  if (silentMs > HEARTBEAT_TIMEOUT_MS) {
    ESP_LOGW(TAG, "no word from the doorbell for %lld s", silentMs / 1000);
    return false;
  }
  return true;
}
