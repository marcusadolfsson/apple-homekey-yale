// Relay doorbell: owns the PN532 and answers NFC operations for a base station
// over ESP-NOW. No HomeKey keys, no WiFi association, no Bluetooth: the base
// runs the HomeKey transaction and only the card exchange crosses the link.
//
// Test build: the radio stays awake (we are measuring latency, not power) and
// the link is unencrypted.
#include "Pn532I2cTransport.hpp"
#include "RelayProtocol.hpp"
#include "pn532_cxx/pn532.hpp"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include <array>
#include <cstring>
#include <vector>

namespace {

const char *TAG = "doorbell";
constexpr gpio_num_t PIN_SDA = GPIO_NUM_22;  // XIAO D4
constexpr gpio_num_t PIN_SCL = GPIO_NUM_23;  // XIAO D5
const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct Msg {
  uint8_t mac[6];
  uint8_t data[relay::MAX_ESPNOW];
  uint16_t len;
};

QueueHandle_t g_rx = nullptr;
uint8_t g_base[6] = {0};
bool g_baseKnown = false;
uint8_t g_channel = 0;

Pn532I2cTransport *g_transport = nullptr;
pn532::Frontend *g_pn532 = nullptr;
bool g_pn532Ready = false;
int64_t g_lastInitTryUs = 0;
uint32_t g_polls = 0;
// Inverted polling state: the doorbell drives its own reader once the base has
// given it an ECP frame, and stays quiet on the radio until a card shows up.
std::array<uint8_t, 18> g_ecp{};
bool g_haveEcp = false;
uint16_t g_pollIntervalMs = 100;
bool g_tagActive = false;       // a card is being worked on; stop polling
int64_t g_tagActiveUs = 0;
int64_t g_lastEcpReqUs = 0;
int64_t g_lastHeartbeatUs = 0;

// Reassembly for the one request in flight; the base is strictly request/response.
struct {
  uint8_t op = 0, seq = 0, fragCnt = 0, got = 0, flags = 0;
  uint16_t totalLen = 0;
  std::vector<uint8_t> buf;
} g_asm;

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < (int)relay::HDR || !g_rx) return;
  Msg m{};
  std::memcpy(m.mac, info->src_addr, 6);
  m.len = uint16_t(len);
  std::memcpy(m.data, data, len);
  xQueueSend(g_rx, &m, 0);
}

bool addPeer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p{};
  std::memcpy(p.peer_addr, mac, 6);
  p.channel = 0;  // current channel
  p.ifidx = WIFI_IF_STA;
  p.encrypt = false;
  return esp_now_add_peer(&p) == ESP_OK;
}

void send(const uint8_t mac[6], relay::Op op, uint8_t seq, uint8_t flags,
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
    esp_err_t err = esp_now_send(mac, frame, relay::HDR + n);
    if (err != ESP_OK) ESP_LOGW(TAG, "send %s frag %u failed: %s", relay::opName(op), (unsigned)i, esp_err_to_name(err));
  }
}

// ---------------------------------------------------------------- PN532

bool pn532Init() {
  if (!g_transport) g_transport = new Pn532I2cTransport(PIN_SDA, PIN_SCL);
  if (!g_pn532) g_pn532 = new pn532::Frontend(*g_transport);
  if (g_pn532->begin() != pn532::Status::SUCCESS) ESP_LOGW(TAG, "PN532 begin reported an error");
  auto ver = g_pn532->GetFirmwareVersion();
  if (!ver) {
    ESP_LOGE(TAG, "no PN532 found on SDA=%d SCL=%d", PIN_SDA, PIN_SCL);
    return false;
  }
  ESP_LOGI(TAG, "PN532 firmware %d.%d", int((*ver >> 24) & 0xFF), int((*ver >> 16) & 0xFF));
  g_pn532Ready = true;
  // Same setup the base's own PN532 driver uses.
  g_pn532->RFConfiguration(0x01, {0x03});
  g_pn532->setPassiveActivationRetries(0);
  g_pn532->RFConfiguration(0x02, {0x00, 0x0B, 0x10});
  g_pn532->RFConfiguration(0x04, {0xFF});
  return true;
}

// Poll our own reader once; announce to the base if a card is there.
void pollOnce() {
  if (!g_pn532Ready) {
    if (esp_timer_get_time() - g_lastInitTryUs > 5000000) {
      g_lastInitTryUs = esp_timer_get_time();
      pn532Init();
    }
    return;
  }
  std::vector<uint8_t> res;
  (void)g_pn532->InCommunicateThru(g_ecp, res, 50);
  std::vector<uint8_t> uid;
  std::array<uint8_t, 2> atqa{};
  uint8_t sak = 0;
  if (g_pn532->InListPassiveTarget(0x00, uid, atqa, sak, uint16_t(g_pollIntervalMs)) !=
      pn532::Status::SUCCESS) {
    return;
  }
  std::vector<uint8_t> out;
  out.push_back(uint8_t(uid.size()));
  out.insert(out.end(), uid.begin(), uid.end());
  out.push_back(atqa[0]);
  out.push_back(atqa[1]);
  out.push_back(sak);
  g_tagActive = true;  // hold the card; the base will drive APDUs now
  g_tagActiveUs = esp_timer_get_time();
  send(g_base, relay::Op::TagEvent, 0, 3, out.data(), out.size());
  ESP_LOGI(TAG, "tag detected, announced to base");
}

void handlePoll(const Msg &m, const relay::Header &h, const std::vector<uint8_t> &payload) {
  if (!g_pn532Ready) {
    // Reader missing at boot: retry occasionally so a re-seated cable recovers.
    if (esp_timer_get_time() - g_lastInitTryUs > 5000000) {
      g_lastInitTryUs = esp_timer_get_time();
      pn532Init();
    }
    send(m.mac, relay::Op::PollRsp, h.seq, 0, nullptr, 0);  // flags bit1 clear = reader not ready
    return;
  }
  if ((++g_polls % 50) == 0) ESP_LOGI(TAG, "%u polls relayed", (unsigned)g_polls);
  // payload: [18B ECP][4B timeout ms LE]
  uint32_t timeoutMs = 500;
  std::vector<uint8_t> ecp;
  if (payload.size() >= 22) {
    ecp.assign(payload.begin(), payload.begin() + 18);
    std::memcpy(&timeoutMs, payload.data() + 18, 4);
  }
  std::vector<uint8_t> res;
  if (!ecp.empty()) (void)g_pn532->InCommunicateThru(ecp, res, 50);

  std::vector<uint8_t> uid;
  std::array<uint8_t, 2> atqa{};
  uint8_t sak = 0;
  const auto st = g_pn532->InListPassiveTarget(0x00, uid, atqa, sak, uint16_t(timeoutMs));
  const bool found = st == pn532::Status::SUCCESS;

  std::vector<uint8_t> out;
  out.push_back(uint8_t(uid.size()));
  out.insert(out.end(), uid.begin(), uid.end());
  out.push_back(atqa[0]);
  out.push_back(atqa[1]);
  out.push_back(sak);
  // bit0 = tag found, bit1 = PN532 healthy
  send(m.mac, relay::Op::PollRsp, h.seq, uint8_t((found ? 1 : 0) | 2), out.data(), out.size());
  if (found) ESP_LOGI(TAG, "tag detected (uid %u bytes), relaying to base", (unsigned)uid.size());
}

void handleApdu(const Msg &m, const relay::Header &h, const std::vector<uint8_t> &payload) {
  // payload: [4B timeout ms LE][C-APDU]
  if (payload.size() < 4) return;
  uint32_t timeoutMs = 0;
  std::memcpy(&timeoutMs, payload.data(), 4);
  std::vector<uint8_t> apdu(payload.begin() + 4, payload.end());
  std::vector<uint8_t> recv;
  const int64_t t0 = esp_timer_get_time();
  const auto st = g_pn532->InDataExchange(apdu, recv, uint16_t(timeoutMs));
  const bool ok = st == pn532::Status::SUCCESS;
  if (ok && recv.size() >= 2) recv.erase(recv.begin(), recv.begin() + 2);  // strip PN532 status
  ESP_LOGD(TAG, "apdu %u -> %u bytes in %lld us", (unsigned)apdu.size(), (unsigned)recv.size(),
           esp_timer_get_time() - t0);
  send(m.mac, relay::Op::ApduRsp, h.seq, ok ? 1 : 0, recv.data(), recv.size());
}

void handle(const Msg &m, const relay::Header &h, const std::vector<uint8_t> &payload) {
  switch (relay::Op(h.op)) {
    case relay::Op::Ping:
      // A base is looking for a doorbell (it restarted, or we paired before it did).
      send(m.mac, relay::Op::Pong, h.seq, g_pn532Ready ? 2 : 0, nullptr, 0);
      if (!g_baseKnown) {
        std::memcpy(g_base, m.mac, 6);
        g_baseKnown = true;
        addPeer(g_base);
        ESP_LOGI(TAG, "base said hello: %02X:%02X:%02X:%02X:%02X:%02X", g_base[0], g_base[1],
                 g_base[2], g_base[3], g_base[4], g_base[5]);
        send(g_base, relay::Op::EcpReq, 0, 0, nullptr, 0);
      }
      break;
    case relay::Op::Pong:
      if (!g_baseKnown) {
        std::memcpy(g_base, m.mac, 6);
        g_baseKnown = true;
        addPeer(g_base);
        ESP_LOGI(TAG, "base found on channel %u: %02X:%02X:%02X:%02X:%02X:%02X", g_channel,
                 g_base[0], g_base[1], g_base[2], g_base[3], g_base[4], g_base[5]);
        send(g_base, relay::Op::EcpReq, 0, 0, nullptr, 0);
      }
      break;
    case relay::Op::PollReq: handlePoll(m, h, payload); break;  // legacy path, still supported
    case relay::Op::EcpSet:
      if (payload.size() >= 20) {
        std::memcpy(g_ecp.data(), payload.data(), 18);
        std::memcpy(&g_pollIntervalMs, payload.data() + 18, 2);
        if (g_pollIntervalMs < 20) g_pollIntervalMs = 20;
        const bool first = !g_haveEcp;
        g_haveEcp = true;
        if (first) ESP_LOGI(TAG, "got ECP data from base; polling every %u ms", g_pollIntervalMs);
      }
      break;
    case relay::Op::ApduReq: handleApdu(m, h, payload); break;
    case relay::Op::PresentReq: {
      (void)g_pn532->InRelease(1);
      std::vector<uint8_t> uid; std::array<uint8_t, 2> atqa{}; uint8_t sak = 0;
      const bool present = g_pn532->InListPassiveTarget(0x00, uid, atqa, sak, 100) == pn532::Status::SUCCESS;
      send(m.mac, relay::Op::PresentRsp, h.seq, present ? 1 : 0, nullptr, 0);
      break;
    }
    case relay::Op::ReleaseReq:
      (void)g_pn532->InRelease(1);
      (void)g_pn532->setPassiveActivationRetries(0);
      g_tagActive = false;  // resume watching for the next card
      send(m.mac, relay::Op::ReleaseRsp, h.seq, 1, nullptr, 0);
      break;
    case relay::Op::HealthReq: {
      const bool ok = g_pn532->WriteRegister({0x63, 0x3d, 0x0}) == pn532::Status::SUCCESS;
      send(m.mac, relay::Op::HealthRsp, h.seq, ok ? 1 : 0, nullptr, 0);
      break;
    }
    default: break;
  }
}

void findBase() {
  // The base sits on its access point's channel; walk the 2.4 GHz channels
  // until it answers a ping.
  uint8_t seq = 0;
  while (!g_baseKnown) {
    for (uint8_t ch = 1; ch <= 13 && !g_baseKnown; ++ch) {
      g_channel = ch;
      esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
      send(BROADCAST, relay::Op::Ping, seq++, 0, nullptr, 0);
      const int64_t until = esp_timer_get_time() + 250000;
      while (esp_timer_get_time() < until && !g_baseKnown) {
        Msg m{};
        if (xQueueReceive(g_rx, &m, pdMS_TO_TICKS(20)) == pdTRUE) {
          relay::Header h{};
          std::memcpy(&h, m.data, relay::HDR);
          if (h.magic0 == relay::MAGIC0 && h.magic1 == relay::MAGIC1) handle(m, h, {});
        }
      }
    }
    if (!g_baseKnown) ESP_LOGW(TAG, "no base station answered; retrying channel scan");
  }
}

}  // namespace

extern "C" void app_main() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  // Test build: keep the receiver on so relay latency reflects the link itself.
  // A battery build would leave power save on and accept the extra delay, or
  // sleep entirely between taps.
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(onRecv));
  addPeer(BROADCAST);

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  ESP_LOGI(TAG, "doorbell MAC %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  g_rx = xQueueCreate(16, sizeof(Msg));
  if (!pn532Init()) ESP_LOGE(TAG, "continuing without a working PN532");

  findBase();

  int64_t lastRequestUs = esp_timer_get_time();
  while (true) {
    Msg m{};
    // If the base has been silent for a while it has probably restarted or moved
    // channel: go back to searching rather than waiting forever.
    const TickType_t wait = (g_haveEcp && !g_tagActive) ? 0 : pdMS_TO_TICKS(1000);
    if (xQueueReceive(g_rx, &m, wait) != pdTRUE) {
      if (g_haveEcp && !g_tagActive) {
        pollOnce();
        // A quiet doorbell is indistinguishable from a dead one, so say hello
        // occasionally. One small frame every 30 s is cheap even on battery.
        if (g_baseKnown && esp_timer_get_time() - g_lastHeartbeatUs > 30000000) {
          g_lastHeartbeatUs = esp_timer_get_time();
          send(g_base, relay::Op::HealthRsp, 0, g_pn532Ready ? 2 : 0, nullptr, 0);
        }
        continue;
      }
      if (g_baseKnown && !g_haveEcp && esp_timer_get_time() - g_lastEcpReqUs > 2000000) {
        g_lastEcpReqUs = esp_timer_get_time();
        send(g_base, relay::Op::EcpReq, 0, 0, nullptr, 0);
      }
      // A base that stops driving APDUs after a tap should not wedge us.
      if (g_tagActive && esp_timer_get_time() - g_tagActiveUs > 5000000) {
        ESP_LOGW(TAG, "no APDUs after announcing a tag; resuming polling");
        (void)g_pn532->InRelease(1);
        g_tagActive = false;
      }
      if (g_baseKnown && esp_timer_get_time() - lastRequestUs > 30000000) {
        ESP_LOGW(TAG, "no requests for 30 s; searching for a base again");
        g_baseKnown = false;
        findBase();
        lastRequestUs = esp_timer_get_time();
      }
      continue;
    }
    lastRequestUs = esp_timer_get_time();
    relay::Header h{};
    std::memcpy(&h, m.data, relay::HDR);
    if (h.magic0 != relay::MAGIC0 || h.magic1 != relay::MAGIC1 || h.version != relay::VERSION) continue;

    std::vector<uint8_t> payload;
    if (h.fragCnt <= 1) {
      payload.assign(m.data + relay::HDR, m.data + m.len);
    } else {
      // Reassemble: the base sends one request at a time.
      if (g_asm.op != h.op || g_asm.seq != h.seq) {
        g_asm.op = h.op; g_asm.seq = h.seq; g_asm.fragCnt = h.fragCnt;
        g_asm.got = 0; g_asm.totalLen = h.totalLen;
        g_asm.buf.assign(h.totalLen, 0);
      }
      const size_t off = size_t(h.fragIdx) * relay::MAX_PAYLOAD;
      const size_t n = m.len - relay::HDR;
      if (off + n <= g_asm.buf.size()) std::memcpy(g_asm.buf.data() + off, m.data + relay::HDR, n);
      if (++g_asm.got < g_asm.fragCnt) continue;
      payload = g_asm.buf;
      g_asm.op = 0;
    }
    handle(m, h, payload);
  }
}
