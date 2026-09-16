// Relay doorbell: owns the PN532 and answers NFC operations for a base station
// over ESP-NOW. No HomeKey keys, no WiFi association, no Bluetooth: the base
// runs the HomeKey transaction and only the card exchange crosses the link.
//
// Test build: the radio stays awake (we are measuring latency, not power) and
// the link is unencrypted.
#include "DoorbellPn532Reader.hpp"
#include "NfcReader.hpp"
#include "RelayProtocol.hpp"
#include "St25r3916Reader.hpp"

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_random.h"
#include "mbedtls/sha256.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <string>
#include <vector>

namespace {

const char *TAG = "doorbell";
constexpr gpio_num_t PIN_SDA = GPIO_NUM_22;  // XIAO D4
constexpr gpio_num_t PIN_SCL = GPIO_NUM_23;  // XIAO D5
// Doorbell button: wire it between D0 and GND. GPIO0-7 are the C6's low-power
// pins, so in a battery build this same pin wakes the chip from deep sleep
// (esp_sleep_enable_ext1_wakeup) and a press costs one radio frame. D1/D2 stay
// free for the ST25R3916's interrupt line.
// D1 for the button: D0 doubles as A0, where Seeed's battery divider lands.
constexpr gpio_num_t PIN_BUTTON = GPIO_NUM_1;  // XIAO D1
// Battery sense on A0/D0 through a 1:2 divider (1M + 1M keeps the idle draw
// near 2 uA; Seeed's suggested 200k pair would waste ~10 uA, half our budget).
constexpr adc_channel_t BATTERY_CHANNEL = ADC_CHANNEL_0;  // GPIO0 on the C6
constexpr int BATTERY_DIVIDER = 2;
constexpr int64_t BUTTON_DEBOUNCE_US = 50000;
constexpr int64_t BUTTON_REPEAT_US = 1000000;  // ignore chatter/held button
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

// Which NFC front end is wired up. Compile-time default, overridable from NVS
// (namespace "relay", u8 "reader") so a board can be switched without a rebuild.
// Both sit on the same I2C pins; only the ST25R3916 has an IRQ line (D2) and the
// low-power card detection a battery build needs.
enum class ReaderKind : uint8_t { Pn532 = 0, St25r3916 = 1 };
#ifndef DOORBELL_DEFAULT_READER
#define DOORBELL_DEFAULT_READER 0
#endif
ReaderKind g_readerKind = ReaderKind(DOORBELL_DEFAULT_READER);
INfcReader *g_reader = nullptr;
bool g_readerReady = false;
int64_t g_lastInitTryUs = 0;
uint32_t g_polls = 0;
// Inverted polling state: the doorbell drives its own reader once the base has
// given it an ECP frame, and stays quiet on the radio until a card shows up.
std::array<uint8_t, 18> g_ecp{};
bool g_haveEcp = false;
// How long the reader holds the field open per cycle, and the quiet gap
// between cycles. Both are pushed by the base so the doorbell runs the same
// loop the base would have run locally. The listen window is the one that
// matters for the iPhone: it needs most of those 500 ms to see the ECP frame
// and raise the Home Key, so a short window means no animation and flaky taps.
uint16_t g_listenMs = 500;
uint16_t g_pollDelayMs = 100;
bool g_tagActive = false;       // a card is being worked on; stop polling
int64_t g_tagActiveUs = 0;
// The tag announcement is the one frame a tap cannot afford to lose: the base
// drives everything else, so a dropped announcement means the card sits there
// doing nothing until the 5 s timeout. Keep it and repeat it until the base
// answers with an APDU.
std::vector<uint8_t> g_tagPayload;
int64_t g_tagAnnouncedUs = 0;
int g_tagAnnounceTries = 0;
constexpr int64_t TAG_ANNOUNCE_RETRY_US = 250000;
constexpr int TAG_ANNOUNCE_MAX_TRIES = 4;
int64_t g_lastEcpReqUs = 0;
int64_t g_lastHeartbeatUs = 0;
int g_buttonLast = 1;  // pulled up: 1 = released
int64_t g_buttonChangedUs = 0;
int64_t g_lastPressUs = 0;
// Link security. The key is generated here on first boot, stored in NVS and
// printed once over USB: it never crosses the air. Paste it into the base's web
// page. The base MAC is pinned after the first pairing, also in NVS.
std::array<uint8_t, 16> g_pmk{}, g_lmk{};
bool g_encrypted = false;
bool g_basePinned = false;

void linkSecurityInit() {
  nvs_handle_t h;
  std::array<uint8_t, 16> key{};
  bool haveKey = false;
  if (nvs_open("relay", NVS_READWRITE, &h) == ESP_OK) {
    size_t len = key.size();
    haveKey = nvs_get_blob(h, "key", key.data(), &len) == ESP_OK && len == key.size();
    if (!haveKey) {
      esp_fill_random(key.data(), key.size());
      if (nvs_set_blob(h, "key", key.data(), key.size()) == ESP_OK) nvs_commit(h);
      haveKey = true;
    }
    size_t maclen = 6;
    if (nvs_get_blob(h, "base", g_base, &maclen) == ESP_OK && maclen == 6) {
      // Pinned means "only ever talk to this MAC" -- it does NOT mean we know
      // where it is. The base sits on its AP's channel, which we only learn by
      // scanning, and esp_wifi_start() leaves us on channel 1. Leaving
      // g_baseKnown false here keeps the channel scan running; the pin just
      // makes us ignore every other base that answers.
      g_basePinned = true;
      nvs_get_u8(h, "chan", &g_channel);  // last known channel: tried first
    }
    nvs_close(h);
  }
  if (!haveKey) {
    ESP_LOGE(TAG, "could not store a link key; the link stays unencrypted");
    return;
  }
  uint8_t digest[32];
  mbedtls_sha256(key.data(), key.size(), digest, 0);
  std::copy_n(digest, 16, g_pmk.begin());
  std::copy_n(digest + 16, 16, g_lmk.begin());
  g_encrypted = esp_now_set_pmk(g_pmk.data()) == ESP_OK;

  std::string hex;
  for (uint8_t b : key) { char t[3]; snprintf(t, sizeof(t), "%02x", b); hex += t; }
  ESP_LOGW(TAG, "relay link key: %s", hex.c_str());
  ESP_LOGW(TAG, "paste that into the base's web page (Hardware -> Link key), once.");
  if (g_basePinned) {
    ESP_LOGI(TAG, "base pinned: %02X:%02X:%02X:%02X:%02X:%02X", g_base[0], g_base[1], g_base[2],
             g_base[3], g_base[4], g_base[5]);
  }
}

void pinBase(const uint8_t mac[6]) {
  nvs_handle_t h;
  if (nvs_open("relay", NVS_READWRITE, &h) != ESP_OK) return;
  // Remember the channel we found it on even when the MAC is already pinned:
  // it turns the next boot's channel sweep into a single ping.
  bool dirty = false;
  uint8_t stored = 0;
  if (nvs_get_u8(h, "chan", &stored) != ESP_OK || stored != g_channel) {
    dirty = nvs_set_u8(h, "chan", g_channel) == ESP_OK;
  }
  if (!g_basePinned && nvs_set_blob(h, "base", mac, 6) == ESP_OK) {
    g_basePinned = true;
    dirty = true;
  }
  if (dirty) nvs_commit(h);
  nvs_close(h);
}
adc_oneshot_unit_handle_t g_adc = nullptr;
adc_cali_handle_t g_adcCali = nullptr;

// Battery voltage in millivolts, or 0 when no divider is fitted (USB builds).
uint16_t readBatteryMv() {
  if (!g_adc) return 0;
  int sum = 0, n = 0;
  for (int i = 0; i < 8; ++i) {
    int raw = 0;
    if (adc_oneshot_read(g_adc, BATTERY_CHANNEL, &raw) != ESP_OK) continue;
    sum += raw;
    ++n;
  }
  if (!n) return 0;
  int mv = 0;
  if (g_adcCali) {
    if (adc_cali_raw_to_voltage(g_adcCali, sum / n, &mv) != ESP_OK) return 0;
  } else {
    mv = (sum / n) * 3300 / 4095;  // uncalibrated fallback
  }
  const int battery = mv * BATTERY_DIVIDER;
  // Below ~1 V there is no divider connected, just a floating pin.
  return battery < 1000 ? 0 : uint16_t(battery);
}

void batteryInit() {
  adc_oneshot_unit_init_cfg_t unit{};
  unit.unit_id = ADC_UNIT_1;
  if (adc_oneshot_new_unit(&unit, &g_adc) != ESP_OK) { g_adc = nullptr; return; }
  adc_oneshot_chan_cfg_t chan{};
  chan.bitwidth = ADC_BITWIDTH_DEFAULT;
  chan.atten = ADC_ATTEN_DB_12;  // full 0-3.3 V span
  adc_oneshot_config_channel(g_adc, BATTERY_CHANNEL, &chan);
  adc_cali_curve_fitting_config_t cali{};
  cali.unit_id = ADC_UNIT_1;
  cali.chan = BATTERY_CHANNEL;
  cali.atten = ADC_ATTEN_DB_12;
  cali.bitwidth = ADC_BITWIDTH_DEFAULT;
  if (adc_cali_create_scheme_curve_fitting(&cali, &g_adcCali) != ESP_OK) g_adcCali = nullptr;
}

// Two bytes of battery voltage ride along on frames we were sending anyway.
void appendBattery(std::vector<uint8_t> &out) {
  const uint16_t mv = readBatteryMv();
  out.push_back(uint8_t(mv & 0xFF));
  out.push_back(uint8_t(mv >> 8));
}

// Reassembly for the one request in flight; the base is strictly request/response.
struct {
  uint8_t op = 0, seq = 0, fragCnt = 0, got = 0, flags = 0;
  uint16_t totalLen = 0;
  std::vector<uint8_t> buf;
} g_asm;

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < (int)relay::HDR || !g_rx) return;
  if (g_basePinned && std::memcmp(info->src_addr, g_base, 6) != 0) return;
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
  const bool broadcast = std::all_of(mac, mac + 6, [](uint8_t b) { return b == 0xFF; });
  p.encrypt = g_encrypted && !broadcast;  // broadcasts must stay in the clear
  if (p.encrypt) std::memcpy(p.lmk, g_lmk.data(), g_lmk.size());
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

bool readerInit() {
  if (!g_reader) {
    if (g_readerKind == ReaderKind::St25r3916) {
      // Same 4-pin array the base uses: [0] = SDA, [1] = SCL.
      g_reader = new St25r3916Reader({uint8_t(PIN_SDA), uint8_t(PIN_SCL), 255, 255}, g_ecp);
      ESP_LOGI(TAG, "reader: ST25R3916 on SDA=%d SCL=%d", PIN_SDA, PIN_SCL);
    } else {
      g_reader = new DoorbellPn532Reader(PIN_SDA, PIN_SCL, g_ecp);
      ESP_LOGI(TAG, "reader: PN532 on SDA=%d SCL=%d", PIN_SDA, PIN_SCL);
    }
  }
  g_readerReady = g_reader->init() && g_reader->beginDiscovery();
  return g_readerReady;
}

// Poll our own reader once; announce to the base if a card is there.
void pollOnce() {
  if (!g_readerReady) {
    if (esp_timer_get_time() - g_lastInitTryUs > 5000000) {
      g_lastInitTryUs = esp_timer_get_time();
      readerInit();
    }
    return;
  }
  std::vector<uint8_t> res;
  // The ECP frame is what makes an iPhone raise the Home Key on its own. It is
  // fire-and-forget (nothing answers it), but a transport-level failure here is
  // invisible except as "taps only work with the key open in Wallet", so report
  // a status change rather than discarding it.
  std::vector<uint8_t> uid;
  std::array<uint8_t, 2> atqa{};
  uint8_t sak = 0;
  if (!g_reader->pollForTag(uid, atqa, sak, g_listenMs)) return;
  std::vector<uint8_t> out;
  out.push_back(uint8_t(uid.size()));
  out.insert(out.end(), uid.begin(), uid.end());
  out.push_back(atqa[0]);
  out.push_back(atqa[1]);
  out.push_back(sak);
  appendBattery(out);
  g_tagActive = true;  // hold the card; the base will drive APDUs now
  g_tagActiveUs = esp_timer_get_time();
  g_tagPayload = out;
  g_tagAnnouncedUs = g_tagActiveUs;
  g_tagAnnounceTries = 1;
  send(g_base, relay::Op::TagEvent, 0, 3, out.data(), out.size());
  ESP_LOGI(TAG, "tag detected, announced to base");
}

// Debounced press detection. Polled here because this test build stays awake;
// a battery build would let the pin wake the chip instead.
void checkButton() {
  const int level = gpio_get_level(PIN_BUTTON);
  const int64_t now = esp_timer_get_time();
  if (level != g_buttonLast) {
    g_buttonLast = level;
    g_buttonChangedUs = now;
    return;
  }
  if (level != 0 || now - g_buttonChangedUs < BUTTON_DEBOUNCE_US) return;
  if (now - g_lastPressUs < BUTTON_REPEAT_US) return;
  g_lastPressUs = now;
  if (!g_baseKnown) {
    ESP_LOGW(TAG, "button pressed but no base is paired");
    return;
  }
  ESP_LOGI(TAG, "button pressed; telling the base");
  std::vector<uint8_t> payload;
  appendBattery(payload);
  send(g_base, relay::Op::ButtonPress, 0, 0, payload.data(), payload.size());
}

void handlePoll(const Msg &m, const relay::Header &h, const std::vector<uint8_t> &payload) {
  if (!g_readerReady) {
    // Reader missing at boot: retry occasionally so a re-seated cable recovers.
    if (esp_timer_get_time() - g_lastInitTryUs > 5000000) {
      g_lastInitTryUs = esp_timer_get_time();
      readerInit();
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
  // Legacy base-driven path: the ECP frame arrives per request; the reader
  // transmits whatever g_ecp holds, so drop it in there first.
  if (!ecp.empty()) std::copy(ecp.begin(), ecp.end(), g_ecp.begin());

  std::vector<uint8_t> uid;
  std::array<uint8_t, 2> atqa{};
  uint8_t sak = 0;
  const bool found = g_reader->pollForTag(uid, atqa, sak, timeoutMs);

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
  const bool ok = g_reader && g_reader->exchangeApdu(apdu, recv, timeoutMs);
  ESP_LOGD(TAG, "apdu %u -> %u bytes in %lld us", (unsigned)apdu.size(), (unsigned)recv.size(),
           esp_timer_get_time() - t0);
  send(m.mac, relay::Op::ApduRsp, h.seq, ok ? 1 : 0, recv.data(), recv.size());
}

void handle(const Msg &m, const relay::Header &h, const std::vector<uint8_t> &payload) {
  switch (relay::Op(h.op)) {
    case relay::Op::Ping:
      // A base is looking for a doorbell (it restarted, or we paired before it did).
      send(m.mac, relay::Op::Pong, h.seq, g_readerReady ? 2 : 0, nullptr, 0);
      if (!g_baseKnown) {
        std::memcpy(g_base, m.mac, 6);
        g_baseKnown = true;
        addPeer(g_base);
        pinBase(g_base);
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
        pinBase(g_base);
        ESP_LOGI(TAG, "base found on channel %u: %02X:%02X:%02X:%02X:%02X:%02X", g_channel,
                 g_base[0], g_base[1], g_base[2], g_base[3], g_base[4], g_base[5]);
        send(g_base, relay::Op::EcpReq, 0, 0, nullptr, 0);
      }
      break;
    case relay::Op::PollReq: handlePoll(m, h, payload); break;  // legacy path, still supported
    case relay::Op::EcpSet:
      if (payload.size() >= 20) {
        std::memcpy(g_ecp.data(), payload.data(), 18);
        std::memcpy(&g_listenMs, payload.data() + 18, 2);
        if (g_listenMs < 100) g_listenMs = 100;
        if (payload.size() >= 22) std::memcpy(&g_pollDelayMs, payload.data() + 20, 2);
        const bool first = !g_haveEcp;
        g_haveEcp = true;
        if (first) {
          char hex[3 * 18 + 1];
          for (int i = 0; i < 18; ++i) snprintf(hex + 3 * i, 4, "%02X ", g_ecp[i]);
          ESP_LOGI(TAG, "got ECP data from base; listening %u ms every %u ms: %s", g_listenMs,
                   g_pollDelayMs, hex);
        }
      }
      break;
    case relay::Op::ApduReq: handleApdu(m, h, payload); break;
    case relay::Op::PresentReq: {
      const bool present = g_reader && g_reader->isTagStillPresent();
      send(m.mac, relay::Op::PresentRsp, h.seq, present ? 1 : 0, nullptr, 0);
      break;
    }
    case relay::Op::ReleaseReq:
      if (g_reader) g_reader->releaseTag();
      g_tagActive = false;  // resume watching for the next card
      send(m.mac, relay::Op::ReleaseRsp, h.seq, 1, nullptr, 0);
      break;
    case relay::Op::HealthReq: {
      const bool ok = g_reader && g_reader->healthCheck();
      send(m.mac, relay::Op::HealthRsp, h.seq, ok ? 1 : 0, nullptr, 0);
      break;
    }
    default: break;
  }
}

void findBase() {
  // The base sits on its access point's channel; walk the 2.4 GHz channels
  // until it answers a ping. The channel we last found it on is tried first,
  // so the usual case costs one ping rather than a sweep.
  uint8_t seq = 0;
  const uint8_t first = (g_channel >= 1 && g_channel <= 13) ? g_channel : 1;
  std::vector<uint8_t> order{first};
  for (uint8_t ch = 1; ch <= 13; ++ch)
    if (ch != first) order.push_back(ch);
  while (!g_baseKnown) {
    for (uint8_t ch : order) {
      if (g_baseKnown) break;
      g_channel = ch;
      esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
      // Carry the reader state in the ping too: the base adopts us from this
      // frame, and otherwise it would show "reader not ready" until the first
      // heartbeat 30 s later.
      send(BROADCAST, relay::Op::Ping, seq++, g_readerReady ? 2 : 0, nullptr, 0);
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

  gpio_config_t btn{};
  btn.pin_bit_mask = 1ULL << PIN_BUTTON;
  btn.mode = GPIO_MODE_INPUT;
  btn.pull_up_en = GPIO_PULLUP_ENABLE;  // button shorts to GND when pressed
  btn.pull_down_en = GPIO_PULLDOWN_DISABLE;
  btn.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&btn);
  ESP_LOGI(TAG, "doorbell button on GPIO%d (wire it to GND)", PIN_BUTTON);
  batteryInit();
  const uint16_t mv = readBatteryMv();
  if (mv) ESP_LOGI(TAG, "battery %u mV", mv);
  else ESP_LOGI(TAG, "no battery divider on A0; reporting battery as unknown");

  linkSecurityInit();
  // The base answers our broadcast ping with an encrypted unicast Pong, and
  // ESP-NOW only decrypts frames from a peer we already hold with the matching
  // key. Adding the pinned base up front is what makes that Pong readable --
  // without it we ping forever and never hear the answer.
  if (g_basePinned) addPeer(g_base);
  g_rx = xQueueCreate(16, sizeof(Msg));
  {
    // NVS override of the compiled default, so a re-wired board needs no rebuild.
    nvs_handle_t h;
    if (nvs_open("relay", NVS_READONLY, &h) == ESP_OK) {
      uint8_t kind = 0;
      if (nvs_get_u8(h, "reader", &kind) == ESP_OK && kind <= 1) g_readerKind = ReaderKind(kind);
      nvs_close(h);
    }
  }
  if (!readerInit()) ESP_LOGE(TAG, "continuing without a working NFC reader");

  findBase();

  int64_t lastRequestUs = esp_timer_get_time();
  while (true) {
    Msg m{};
    // If the base has been silent for a while it has probably restarted or moved
    // channel: go back to searching rather than waiting forever.
    // Between poll cycles we sit on the queue for the configured gap, which
    // both paces the reader like the base's own loop and keeps us responsive to
    // anything the base sends.
    const TickType_t wait =
        (g_haveEcp && !g_tagActive) ? pdMS_TO_TICKS(g_pollDelayMs) : pdMS_TO_TICKS(1000);
    if (xQueueReceive(g_rx, &m, wait) != pdTRUE) {
      checkButton();
      // A quiet doorbell is indistinguishable from a dead one, so say hello
      // occasionally. One small frame every 30 s is cheap even on battery.
      // This has to run even while we are still waiting for ECP data, or a
      // doorbell stuck in that state never reports its reader or its battery.
      if (g_baseKnown && !g_tagActive &&
          esp_timer_get_time() - g_lastHeartbeatUs > 30000000) {
        g_lastHeartbeatUs = esp_timer_get_time();
        std::vector<uint8_t> hb;
        appendBattery(hb);
        send(g_base, relay::Op::HealthRsp, 0, g_readerReady ? 2 : 0, hb.data(), hb.size());
      }
      // Silence means the base restarted or its AP moved channel. This has to
      // run before the poll-and-continue below, or a polling doorbell never
      // notices and stays parked on a dead channel until it is power-cycled.
      // Heartbeats are not replies, so with nothing else on the wire this
      // fires ~30 s after the base goes quiet.
      if (g_baseKnown && esp_timer_get_time() - lastRequestUs > 30000000) {
        ESP_LOGW(TAG, "no word from the base for 30 s; searching for it again");
        g_baseKnown = false;
        findBase();
        lastRequestUs = esp_timer_get_time();
      }
      if (g_haveEcp && !g_tagActive) {
        pollOnce();
        continue;
      }
      if (g_baseKnown && !g_haveEcp && esp_timer_get_time() - g_lastEcpReqUs > 2000000) {
        g_lastEcpReqUs = esp_timer_get_time();
        send(g_base, relay::Op::EcpReq, 0, 0, nullptr, 0);
      }
      // Repeat the announcement until the base starts driving APDUs. At -67 dBm
      // a single frame is lost often enough to cost a tap outright.
      if (g_tagActive && g_tagAnnounceTries > 0 &&
          g_tagAnnounceTries < TAG_ANNOUNCE_MAX_TRIES &&
          esp_timer_get_time() - g_tagAnnouncedUs > TAG_ANNOUNCE_RETRY_US) {
        g_tagAnnouncedUs = esp_timer_get_time();
        ++g_tagAnnounceTries;
        ESP_LOGW(TAG, "no APDU yet; re-announcing the tag (try %d)", g_tagAnnounceTries);
        send(g_base, relay::Op::TagEvent, 0, 3, g_tagPayload.data(), g_tagPayload.size());
      }
      // A base that stops driving APDUs after a tap should not wedge us.
      // Fallback only: the base releases us explicitly. Auth completes in
      // ~0.3 s, so 2 s bounds the blind window if that release ever goes
      // missing, without cutting a slow transaction short.
      if (g_tagActive && esp_timer_get_time() - g_tagActiveUs > 2000000) {
        ESP_LOGW(TAG, "no release from the base 2 s after the tag; resuming polling");
        if (g_reader) g_reader->releaseTag();
        g_tagActive = false;
      }
      continue;
    }
    lastRequestUs = esp_timer_get_time();
    g_tagAnnounceTries = 0;  // the base is talking to us; stop repeating
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
