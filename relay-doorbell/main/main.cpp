// Relay doorbell: owns the NFC reader and answers NFC operations for a base
// station over ESP-NOW. No HomeKey keys, no WiFi association, no Bluetooth: the
// base runs the HomeKey transaction and only the card exchange crosses the link,
// encrypted and pinned by MAC.
//
// With the ST25R3916 the doorbell deep-sleeps between events (DOORBELL_SLEEP):
// the reader sits in its own low-power wake-up mode and raises IRQ (D1) when a
// phone arrives; the button (D2) and an hourly heartbeat timer are the other
// wake sources. State that has to survive the sleep lives in RTC memory.
#include "DoorbellPn532Reader.hpp"
#include "NfcReader.hpp"
#include "RelayProtocol.hpp"
#include "St25r3916Reader.hpp"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/rtc_io.h"
#include "driver/usb_serial_jtag.h"
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
#include "esp_sleep.h"
#include "mbedtls/sha256.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <string>
#include <vector>

#include <sys/time.h>
#include <unistd.h>

namespace {

const char *TAG = "doorbell";
// Production doorbell wiring; any C6 pin can carry I2C. (The first bench
// doorbell, MAC ..8C:DC, has SDA on D4 and SCL on D5.)
constexpr gpio_num_t PIN_SDA = GPIO_NUM_23;  // XIAO D5
constexpr gpio_num_t PIN_SCL = GPIO_NUM_22;  // XIAO D4
// Doorbell button: wire it between D2 and GND. GPIO0-7 are the C6's low-power
// pins, so in a battery build this same pin wakes the chip from deep sleep
// and a press costs one radio frame. (Not D0: that doubles as A0, where the
// battery divider lands.)
constexpr gpio_num_t PIN_BUTTON = GPIO_NUM_2;  // XIAO D2
// ST25R3916 IRQ, push-pull from the chip. Also a low-power pin, so the same
// wire is what wakes the C6 from deep sleep when a phone arrives.
constexpr gpio_num_t PIN_NFC_IRQ = GPIO_NUM_1;  // XIAO D1 (the first bench doorbell, MAC ..8C:DC, had it on D2)
// Battery sense on A0/D0 through a 1:2 divider (1M + 1M keeps the idle draw
// near 2 uA; Seeed's suggested 200k pair would waste ~10 uA, half our budget).
constexpr adc_channel_t BATTERY_CHANNEL = ADC_CHANNEL_0;  // GPIO0 on the C6
constexpr int BATTERY_DIVIDER = 2;
constexpr int64_t BUTTON_DEBOUNCE_US = 50000;
constexpr int64_t HEARTBEAT_US = 30000000;  // while awake (on a computer's USB)
constexpr int64_t BASE_SILENCE_US = HEARTBEAT_US * 3 / 2;

// Deep sleep between events (stage 2 of the battery work). DOORBELL_SLEEP turns
// it on; it only ever happens with the ST25R3916, whose wake-up mode is what
// wakes us. While a computer is on the USB port the doorbell stays awake so it
// can be flashed and logged -- a sleeping C6 drops off USB. DOORBELL_SLEEP_ON_USB
// overrides that for bench tests, and also shortens the heartbeat so the board
// reappears on USB every couple of minutes.
#ifndef DOORBELL_SLEEP
#define DOORBELL_SLEEP 1
#endif
#ifndef DOORBELL_SLEEP_ON_USB
#define DOORBELL_SLEEP_ON_USB 0
#endif
constexpr int64_t HEARTBEAT_SLEEP_US = DOORBELL_SLEEP_ON_USB ? 120000000LL : 3600000000LL;
constexpr int64_t COLD_BOOT_AWAKE_US = 20000000;  // time to flash or attach after a reset
// After a wake, stay up at least this long. Bench tests need ~3 s for macOS to
// re-enumerate the USB console, or nothing the wake did gets logged.
constexpr int64_t WAKE_MIN_AWAKE_US = DOORBELL_SLEEP_ON_USB ? 3000000 : 0;
constexpr int64_t HEARTBEAT_ACK_WAIT_US = 300000;  // the base acks in a few ms
constexpr int HEARTBEAT_TRIES = 3;  // single frames get lost (base WiFi power save, AP hops)
// After a failed search a sleeping doorbell tries again this soon, not an hour later.
constexpr int64_t HEARTBEAT_RETRY_US = DOORBELL_SLEEP_ON_USB ? 30000000LL : 300000000LL;
constexpr int64_t TX_DRAIN_US = 50000;  // let the last frame leave before sleeping
constexpr int64_t LPCD_REARM_SLEEP_US = 3600000000LL;  // refresh the reference hourly
// No base found (it is down, or out of range): search for a few seconds, then
// sleep and try again later, backing off. Searching costs ~80 mA; doing it
// without a limit would flatten the battery in a day.
constexpr int BASE_SEARCH_ROUNDS = 2;  // ~7 s: 13 channels x 250 ms, twice
// Someone is at the door (a tap or the button woke us): search longer, since
// the base may still be starting up. ~0.7 mAh per such wake.
constexpr int64_t BASE_SEARCH_ATTENDED_US = 30000000;
// Taps also wake a doorbell without a base (see sleepWithoutBase), so the
// timer only matters when nobody is at the door; 15 min caps how long the
// base's dashboard shows it missing after an outage.
constexpr int64_t BASE_SEARCH_BACKOFF_S[] = {60, 120, 300, 600, 900};
constexpr int64_t BASE_SEARCH_BACKOFF_TEST_S = 30;  // sleep-on-USB bench build
constexpr int64_t BUTTON_REPEAT_US = 1000000;  // ignore chatter/held button
const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct Msg {
  uint8_t mac[6];
  uint8_t data[relay::MAX_ESPNOW];
  uint16_t len;
};

// Survives deep sleep (not a power cut). Plain bytes only: RTC_DATA_ATTR data is
// not re-initialised on a wake, and a constructor would undo exactly that.
struct RtcState {
  uint32_t magic;
  uint8_t base[6];
  uint8_t channel;
  uint8_t ecp[18];
  uint16_t listenMs, pollDelayMs;
  uint8_t dres;
  uint8_t readerInWakeUp;
  St25r3916Reader::WakeUpPersist wu;
  int64_t lastHeartbeatUs;  // rtcNowUs() of the last heartbeat
  int64_t lastArmUs;        // rtcNowUs() of the last wake-up reference
  uint32_t wakes, nfcWakes, buttonWakes, timerWakes, taps;
  uint8_t baseLost;     // asleep because no base answered: search again on waking
  uint8_t searchFails;  // consecutive failed searches, for the back-off
  uint8_t dresSwept;    // `dres` holds this power-up's sweep result
};
RTC_DATA_ATTR RtcState g_rtc;
constexpr uint32_t RTC_MAGIC = 0xD00B5703;

// Keeps counting through deep sleep, unlike esp_timer_get_time().
int64_t rtcNowUs() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return int64_t(tv.tv_sec) * 1000000 + tv.tv_usec;
}

bool g_fromSleep = false;     // this boot is a wake with valid RTC state
bool g_wokeByNfc = false, g_wokeByButton = false, g_wokeByTimer = false;
int64_t g_stayAwakeUntilUs = 0;
int64_t g_lastTxUs = 0;
bool g_hbAcked = true;        // the last heartbeat has been answered
int64_t g_hbAckDeadlineUs = 0;
int g_hbTries = 0;
int64_t g_tagAnnouncedAtUs = 0;  // esp_timer at this wake's first announcement
int64_t g_readyToSleepAtUs = 0;  // first moment nothing but the minimum awake time kept us up

QueueHandle_t g_rx = nullptr;
uint8_t g_base[6] = {0};
bool g_baseKnown = false;
uint8_t g_channel = 0;

// Which NFC front end is wired up. Compile-time default, overridable from NVS
// (namespace "relay", u8 "reader") so a board can be switched without a rebuild.
// Both sit on the same I2C pins; only the ST25R3916 has an IRQ line (D1) and the
// low-power card detection a battery build needs.
enum class ReaderKind : uint8_t { Pn532 = 0, St25r3916 = 1 };
#ifndef DOORBELL_DEFAULT_READER
#define DOORBELL_DEFAULT_READER 1
#endif
ReaderKind g_readerKind = ReaderKind(DOORBELL_DEFAULT_READER);
INfcReader *g_reader = nullptr;
bool g_readerReady = false;
int64_t g_lastInitTryUs = 0;
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
// One physical tap = one announcement. After a transaction the phone is usually
// still lying on the reader, and announcing it again sends the base through a
// second full transaction and a second unlock request while the first unlock is
// still connecting - with the fast ST25R3916 that came ~300 ms later, and it
// (together with a since-removed pre-emption on the base) left a tap with no
// unlock at all. After each release, wait for the field to be empty for two
// polls in a row before announcing anything again.
bool g_awaitFieldClear = false;
int g_fieldClearPolls = 0;

// Low-power card detection (LPCD). Instead of polling ~15 times a second with
// the field on, the ST25R3916 rests in its wake-up mode, sampling the antenna
// every LPCD_PERIOD_MS, and raises IRQ (D1) when a phone loads it. Only then
// does the doorbell poll, for LPCD_ACTIVE_US, before re-arming once the field
// is clear. Stage 1 of the battery work: the C6 itself still stays awake.
#ifndef DOORBELL_LPCD
#define DOORBELL_LPCD 1
#endif
constexpr uint16_t LPCD_PERIOD_MS = 100;
// In A/D steps. Wake-up-mode readings on the Click sit within ~2 steps of
// each other at rest, so 2 (RFAL's default) fired on noise; 3 is the floor.
constexpr uint8_t LPCD_AMPLITUDE_DELTA = 3;
// Phase is off: on the Click it reads 0 in every mode, so it can never move.
// RFAL's default wake-up configuration is amplitude-only too.
constexpr uint8_t LPCD_PHASE_DELTA = 0;
constexpr int64_t LPCD_ACTIVE_US = 1500000;    // poll this long after a wake-up
constexpr int64_t LPCD_RECAL_US = 60000000;    // re-take the reference when idle
constexpr int64_t LPCD_PEEK_US = 2000000;      // backup register check, see lpcdGate()
St25r3916Reader *g_st = nullptr;  // set when the ST25R3916 is the reader
int64_t g_lpcdArmedUs = 0;
int64_t g_lpcdActiveUntilUs = 0;
int64_t g_lpcdLastPeekUs = 0;
int64_t g_lpcdWakeUs = 0;
bool g_lpcdWakePending = false;  // woke; waiting to see whether a card shows up
bool g_lpcdCardSeen = false;     // this wake found a card: no false-alarm window to wait out
int64_t g_fieldClearSinceUs = 0; // when g_awaitFieldClear was set
// A phone left lying on the reader would hold the doorbell awake polling until
// it is lifted. After this long, re-arm with it there: the reference then
// includes the phone, and lifting it off is the next wake-up.
constexpr int64_t FIELD_CLEAR_GIVE_UP_US = 10000000;
int g_lpcdWakes = 0, g_lpcdWakesWithCard = 0, g_lpcdFalseWakes = 0;
int64_t g_lpcdStatsUs = 0;
int64_t g_lpcdLastWarnUs = 0;
// TX driver resistance used in wake-up mode, picked by a sweep on first arm:
// the strongest drive whose at-rest reading stays clear of the A/D ceiling.
uint8_t g_lpcdDres = 0;
bool g_lpcdSwept = false;  // the sweep has run (this boot, or before the sleep)
// At most 160. With a ferrite sheet behind the antenna the production Click
// reads 187 at full drive, under the old 200 limit, and full drive false-woke
// every 1-2 s: each re-arm right after a burst of polling took a reference
// that the reading then crept 3 steps above. The bench Click ran at ~140
// (d_res 3) with no false wake-ups in 12 minutes.
constexpr uint8_t LPCD_TARGET_MAX = 160;
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
  bool generated = false;
  if (nvs_open("relay", NVS_READWRITE, &h) == ESP_OK) {
    size_t len = key.size();
    haveKey = nvs_get_blob(h, "key", key.data(), &len) == ESP_OK && len == key.size();
    if (!haveKey) {
      generated = true;
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

  // The key is a pairing secret: show it when it is created, and afterwards
  // only on request (doorbell button held while powering on), not every boot.
  if (generated || (!g_fromSleep && gpio_get_level(PIN_BUTTON) == 0)) {
    std::string hex;
    for (uint8_t b : key) { char t[3]; snprintf(t, sizeof(t), "%02x", b); hex += t; }
    ESP_LOGW(TAG, "relay link key: %s", hex.c_str());
    ESP_LOGW(TAG, "paste that into the base's web page (Hardware -> Link key), once.");
  } else {
    ESP_LOGI(TAG, "relay link key in NVS (hold the doorbell button while powering on to print it)");
  }
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
// The tap announcement uses the last reading rather than sampling the ADC
// (8 samples + calibration) between seeing the card and announcing it: the
// voltage cannot change meaningfully between heartbeats, and that frame is
// the one a tap cannot afford to delay.
uint16_t g_batteryMv = 0;
void appendBattery(std::vector<uint8_t> &out, bool fresh) {
  if (fresh) g_batteryMv = readBatteryMv();
  out.push_back(uint8_t(g_batteryMv & 0xFF));
  out.push_back(uint8_t(g_batteryMv >> 8));
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
  g_lastTxUs = esp_timer_get_time();
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

// Who answers on the reader's I2C pins? The ST25R3916 driver only reports
// "no response at 0x50"; on a freshly modified board (COMM SEL jumpers moved,
// headers replaced by wires) the useful fact is what, if anything, is there.
// Runs once at boot on its own short-lived bus, before the driver claims it.
void i2cBusReport(uint8_t expect) {
  i2c_master_bus_config_t cfg{};
  cfg.i2c_port = -1;
  cfg.sda_io_num = PIN_SDA;
  cfg.scl_io_num = PIN_SCL;
  cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  cfg.glitch_ignore_cnt = 7;
  cfg.flags.enable_internal_pullup = true;
  i2c_master_bus_handle_t bus = nullptr;
  if (i2c_new_master_bus(&cfg, &bus) != ESP_OK) return;
  bool present = false;
  for (int i = 0; i < 3 && !present; ++i) {  // first transaction may wake the chip
    present = i2c_master_probe(bus, expect, 20) == ESP_OK;
    if (!present) vTaskDelay(pdMS_TO_TICKS(10));
  }
  if (present) {
    ESP_LOGI(TAG, "I2C: reader answers at 0x%02X", expect);
  } else {
    std::string found;
    for (uint16_t a = 0x08; a < 0x78; ++a) {
      if (i2c_master_probe(bus, a, 20) == ESP_OK) {
        char t[8];
        snprintf(t, sizeof(t), " 0x%02X", a);
        found += t;
      }
    }
    char want[8];
    snprintf(want, sizeof(want), " 0x%02X", expect);
    if (found.find(want) != std::string::npos) {
      ESP_LOGI(TAG, "I2C: reader answers at 0x%02X (slow to wake)", expect);
    } else {
      ESP_LOGE(TAG, "I2C: nothing at 0x%02X; the bus answered at:%s", expect,
               found.empty() ? " (nothing - check wiring, power and the COMM SEL jumpers)" : found.c_str());
    }
  }
  i2c_del_master_bus(bus);
}

void configureIrqPin() {
  gpio_config_t irq{};
  irq.pin_bit_mask = 1ULL << PIN_NFC_IRQ;
  irq.mode = GPIO_MODE_INPUT;
  irq.pull_down_en = GPIO_PULLDOWN_ENABLE;  // a broken wire reads "nothing here"
  irq.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&irq);
}

// After a deep-sleep wake: re-attach to a reader the doorbell left in wake-up
// mode, without the reset readerInit() does (that would wipe the wake-up
// configuration and the event that woke us).
bool readerResume() {
  g_st = new St25r3916Reader({uint8_t(PIN_SDA), uint8_t(PIN_SCL), 255, 255}, g_ecp);
  g_reader = g_st;
  configureIrqPin();
  if (!g_st->resumeInWakeUpMode(g_rtc.wu)) {
    ESP_LOGW(TAG, "could not resume the reader; resetting it");
    delete g_st;
    g_st = nullptr;
    g_reader = nullptr;
    return false;
  }
  g_readerReady = true;
  const int64_t now = esp_timer_get_time();
  g_lpcdArmedUs = now;
  g_lpcdLastPeekUs = now - LPCD_PEEK_US;  // read the wake-up register right away
  return true;
}

bool readerInit() {
  if (!g_reader) {
    if (g_readerKind == ReaderKind::St25r3916) {
      i2cBusReport(0x50);
      // Same 4-pin array the base uses: [0] = SDA, [1] = SCL.
      g_st = new St25r3916Reader({uint8_t(PIN_SDA), uint8_t(PIN_SCL), 255, 255}, g_ecp);
      g_reader = g_st;
      ESP_LOGI(TAG, "reader: ST25R3916 on SDA=%d SCL=%d, IRQ on GPIO%d", PIN_SDA, PIN_SCL,
               PIN_NFC_IRQ);
      configureIrqPin();
    } else {
      g_reader = new DoorbellPn532Reader(PIN_SDA, PIN_SCL, g_ecp);
      ESP_LOGI(TAG, "reader: PN532 on SDA=%d SCL=%d", PIN_SDA, PIN_SCL);
    }
  }
  g_readerReady = g_reader->init() && g_reader->beginDiscovery();
  if (g_readerReady && g_st && DOORBELL_LPCD) {
    // How noisy is the antenna at rest? The wake-up deltas only work if they
    // sit above this, so measure it once and say so.
    int aMin = 255, aMax = 0, pMin = 255, pMax = 0, n = 0;
    for (int i = 0; i < 16; ++i) {
      St25r3916Reader::AntennaReading r;
      if (!g_st->measureAntenna(r)) continue;
      aMin = std::min<int>(aMin, r.amplitude); aMax = std::max<int>(aMax, r.amplitude);
      pMin = std::min<int>(pMin, r.phase);     pMax = std::max<int>(pMax, r.phase);
      ++n;
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (n) {
      ESP_LOGI(TAG, "antenna at rest, measure command (%d samples): amplitude %d-%d, phase %d-%d",
               n, aMin, aMax, pMin, pMax);
    } else {
      ESP_LOGW(TAG, "antenna measurement failed; low-power detection may not work");
    }
  }
  return g_readerReady;
}

void lpcdArm() {
  // First arm: sweep the driver resistance. On the NFC 4 Click the amplitude
  // reading sits at the 255 ceiling at full drive, where a phone cannot pull it
  // down far enough to register (two minutes of taps produced no wake-up).
  // Each step arms insensitive (delta 15), so its reference is the mode's own
  // reading at that drive.
  if (!g_lpcdSwept) {
    g_lpcdSwept = true;
    std::string line;
    int chosen = -1;
    for (uint8_t d = 0; d <= 15; ++d) {
      if (!g_st->startWakeUpMode(LPCD_PERIOD_MS, 15, 0, d)) break;
      const uint8_t a = g_st->wakeUpReference().amplitude;
      g_st->stopWakeUpMode();
      char t[12];
      snprintf(t, sizeof(t), " %u:%u", d, a);
      line += t;
      if (chosen < 0 && a <= LPCD_TARGET_MAX) chosen = d;
    }
    ESP_LOGI(TAG, "wake-up drive sweep (d_res:amplitude):%s", line.c_str());
    if (chosen < 0) {
      g_lpcdDres = 15;
      ESP_LOGW(TAG, "no drive setting brought the reading to %u or below; using the weakest",
               LPCD_TARGET_MAX);
    } else {
      g_lpcdDres = uint8_t(chosen);
      ESP_LOGI(TAG, "wake-up drive: d_res %u", g_lpcdDres);
    }
  }
  const int64_t now = esp_timer_get_time();
  if (!g_st->startWakeUpMode(LPCD_PERIOD_MS, LPCD_AMPLITUDE_DELTA, LPCD_PHASE_DELTA, g_lpcdDres)) {
    ESP_LOGE(TAG, "could not enter wake-up mode; polling instead, retry in 5 s");
    g_lpcdActiveUntilUs = now + 5000000;
    return;
  }
  g_lpcdArmedUs = now;
  g_lpcdLastPeekUs = now;
  g_lpcdActiveUntilUs = 0;
  g_lpcdCardSeen = false;
  g_rtc.lastArmUs = rtcNowUs();
  const auto &ref = g_st->wakeUpReference();
  static bool firstArm = true;
  if (firstArm) {
    firstArm = false;
    ESP_LOGI(TAG, "wake-up armed: reference amplitude %u (measure command said %u, spread %u), "
                  "IRQ line %d", ref.amplitude, g_st->wakeUpDirectAmplitude(), g_st->wakeUpSpread(),
             gpio_get_level(PIN_NFC_IRQ));
  } else {
    ESP_LOGD(TAG, "wake-up armed: reference amplitude %u, spread %u", ref.amplitude,
             g_st->wakeUpSpread());
  }
}

// Returns true while the reader is resting in wake-up mode and the caller must
// not poll; false when it is time to poll (just woke, or still in the active
// window after a wake-up).
bool lpcdGate() {
  if (!DOORBELL_LPCD || !g_st) return false;
  const int64_t now = esp_timer_get_time();

  if (now - g_lpcdStatsUs > 60000000) {
    if (g_lpcdWakes)
      ESP_LOGI(TAG, "low-power detection, last minute: %d wake-up(s), %d with a card, %d false",
               g_lpcdWakes, g_lpcdWakesWithCard, g_lpcdFalseWakes);
    g_lpcdWakes = g_lpcdWakesWithCard = g_lpcdFalseWakes = 0;
    g_lpcdStatsUs = now;
  }

  if (!g_st->inWakeUpMode()) {
    // Polling. Go back to sleep once the window has passed and no card is
    // (still) in the field. If this wake already found its card, the window -
    // there only to tell a false alarm from a slow phone - is cut short.
    if (g_awaitFieldClear && now - g_fieldClearSinceUs > FIELD_CLEAR_GIVE_UP_US) {
      ESP_LOGI(TAG, "card still on the reader after %lld s; re-arming with it there",
               FIELD_CLEAR_GIVE_UP_US / 1000000);
      g_awaitFieldClear = false;
    }
    if ((now < g_lpcdActiveUntilUs && !g_lpcdCardSeen) || g_awaitFieldClear || g_tagActive)
      return false;
    if (g_lpcdWakePending) {
      ++g_lpcdFalseWakes;
      g_lpcdWakePending = false;
      ESP_LOGI(TAG, "wake-up was a false alarm: no card within %lld ms", LPCD_ACTIVE_US / 1000);
    }
    lpcdArm();
    return g_st->inWakeUpMode();
  }

  // Resting. The IRQ line is the signal; reading the wake-up register over I2C
  // every LPCD_PEEK_US is a backup that turns a broken or swapped IRQ wire into
  // a warning instead of silently missed taps.
  const bool irqLine = gpio_get_level(PIN_NFC_IRQ) != 0;
  if (!irqLine && now - g_lpcdLastPeekUs < LPCD_PEEK_US) {
    if (now - g_lpcdArmedUs > LPCD_RECAL_US) {  // drift: re-take the reference
      g_st->stopWakeUpMode();
      lpcdArm();
    }
    return true;
  }
  g_lpcdLastPeekUs = now;
  St25r3916Reader::AntennaReading seen;
  const uint8_t ev = g_st->takeWakeUpEvents(&seen);
  if (!ev) {
    if (irqLine && now - g_lpcdLastWarnUs > 10000000) {
      g_lpcdLastWarnUs = now;
      ESP_LOGW(TAG, "IRQ line high but no wake-up event pending");
    }
    return true;
  }
  if (!irqLine && now - g_lpcdLastWarnUs > 10000000) {
    g_lpcdLastWarnUs = now;
    ESP_LOGW(TAG, "wake-up found by register check, but the IRQ line on D1 stayed low - check that wire");
  }
  g_st->stopWakeUpMode();
  ++g_lpcdWakes;
  g_lpcdWakePending = true;
  g_lpcdWakeUs = now;
  g_lpcdActiveUntilUs = now + LPCD_ACTIVE_US;
  const auto &ref = g_st->wakeUpReference();
  ESP_LOGI(TAG, "wake-up (%s%s) after %lld ms: amplitude %u (ref %u), phase %u (ref %u)",
           (ev & St25r3916Reader::WAKE_AMPLITUDE) ? "amplitude" : "",
           (ev & St25r3916Reader::WAKE_PHASE) ? ((ev & St25r3916Reader::WAKE_AMPLITUDE) ? "+phase" : "phase") : "",
           (now - g_lpcdArmedUs) / 1000, seen.amplitude, ref.amplitude, seen.phase, ref.phase);
  return false;
}

bool sleepAllowedNow() {
  if (!DOORBELL_SLEEP || !g_st) return false;
  return DOORBELL_SLEEP_ON_USB || !usb_serial_jtag_is_connected();
}

// Wake on the reader's IRQ (high, when `nfc`) and the button (low). The pulls
// are set on the low-power pads; the IDF holds them through the sleep.
void armWakePins(bool nfc) {
  rtc_gpio_init(PIN_BUTTON);
  rtc_gpio_set_direction(PIN_BUTTON, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_dis(PIN_BUTTON);
  rtc_gpio_pullup_en(PIN_BUTTON);
  rtc_gpio_init(PIN_NFC_IRQ);
  rtc_gpio_set_direction(PIN_NFC_IRQ, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_dis(PIN_NFC_IRQ);
  rtc_gpio_pulldown_en(PIN_NFC_IRQ);
  if (nfc) esp_sleep_enable_ext1_wakeup_io(1ULL << PIN_NFC_IRQ, ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_sleep_enable_ext1_wakeup_io(1ULL << PIN_BUTTON, ESP_EXT1_WAKEUP_ANY_LOW);
}

void flushConsoleBeforeSleep() {
  // Let the console drain: with 20 ms this last line never reached the
  // computer, and it is the one carrying the wake-up timing.
  fflush(stdout);
  fsync(fileno(stdout));
  vTaskDelay(pdMS_TO_TICKS(200));
}

// No base answered. Sleep until the back-off timer, the button, or a tap: the
// reader keeps watching for phones, so someone at the door makes the doorbell
// search again at once (last known channel first, usually one ping), and if
// the base is back, the phone still on the reader is served on the same wake.
// Without a working reader it is powered down and only timer and button wake.
[[noreturn]] void sleepWithoutBase() {
  g_rtc.magic = RTC_MAGIC;
  g_rtc.baseLost = 1;
  g_rtc.readerInWakeUp = 0;
  if (g_rtc.searchFails < 255) ++g_rtc.searchFails;
  bool nfcWake = false;
  if (DOORBELL_LPCD && g_st && g_readerReady) {
    if (g_st->inWakeUpMode()) g_st->takeWakeUpEvents();  // drop the tap that woke us
    if (!g_st->inWakeUpMode() || gpio_get_level(PIN_NFC_IRQ) != 0) {
      if (g_st->inWakeUpMode()) g_st->stopWakeUpMode();
      lpcdArm();  // sweeps the drive first if this power-up has not yet
    }
    nfcWake = g_st->inWakeUpMode() && gpio_get_level(PIN_NFC_IRQ) == 0;
  }
  if (nfcWake) {
    g_rtc.readerInWakeUp = 1;
    g_rtc.wu = g_st->wakeUpPersist();
    g_rtc.dres = g_lpcdDres;
    g_rtc.dresSwept = 1;
  } else if (g_st) {
    g_st->powerDown();
  }
  armWakePins(nfcWake);
  constexpr int steps = sizeof(BASE_SEARCH_BACKOFF_S) / sizeof(BASE_SEARCH_BACKOFF_S[0]);
  const int64_t waitS = DOORBELL_SLEEP_ON_USB ? BASE_SEARCH_BACKOFF_TEST_S
                                              : BASE_SEARCH_BACKOFF_S[std::min<int>(g_rtc.searchFails, steps) - 1];
  esp_sleep_enable_timer_wakeup(uint64_t(waitS) * 1000000);
  ESP_LOGW(TAG, "no base station found (%u searches in a row); sleeping, next search in %lld s "
                "or on the button%s", g_rtc.searchFails, waitS, nfcWake ? " or a tap" : "");
  flushConsoleBeforeSleep();
  esp_deep_sleep_start();
}

[[noreturn]] void enterDeepSleep() {
  g_rtc.magic = RTC_MAGIC;
  std::memcpy(g_rtc.base, g_base, 6);
  g_rtc.channel = g_channel;
  std::memcpy(g_rtc.ecp, g_ecp.data(), g_ecp.size());
  g_rtc.listenMs = g_listenMs;
  g_rtc.pollDelayMs = g_pollDelayMs;
  g_rtc.dres = g_lpcdDres;
  g_rtc.dresSwept = 1;
  g_rtc.readerInWakeUp = 1;
  g_rtc.wu = g_st->wakeUpPersist();

  g_rtc.baseLost = 0;
  g_rtc.searchFails = 0;
  armWakePins(true);
  int64_t untilHeartbeat = g_rtc.lastHeartbeatUs + HEARTBEAT_SLEEP_US - rtcNowUs();
  if (untilHeartbeat < 1000000) untilHeartbeat = 1000000;
  esp_sleep_enable_timer_wakeup(uint64_t(untilHeartbeat));

  const char *why = g_wokeByNfc ? "a tap" : g_wokeByButton ? "the button" : g_wokeByTimer ? "the heartbeat timer"
                                                                                       : "power-up";
  char tap[64] = "";
  if (g_tagAnnouncedAtUs)
    snprintf(tap, sizeof(tap), ", card announced %lld ms after boot", g_tagAnnouncedAtUs / 1000);
  ESP_LOGI(TAG, "sleeping: woken by %s, done after %lld ms (awake %lld ms)%s; next heartbeat in %lld s. "
                "Since power-up: %lu wake-ups (%lu tap, %lu button, %lu timer), %lu cards",
           why, g_readyToSleepAtUs / 1000, esp_timer_get_time() / 1000, tap, untilHeartbeat / 1000000,
           (unsigned long)g_rtc.wakes,
           (unsigned long)g_rtc.nfcWakes, (unsigned long)g_rtc.buttonWakes,
           (unsigned long)g_rtc.timerWakes, (unsigned long)g_rtc.taps);
  flushConsoleBeforeSleep();
  esp_deep_sleep_start();
}

// Sleep if there is nothing left to do. Called from the idle loop.
void maybeDeepSleep() {
  if (!sleepAllowedNow()) return;
  const int64_t now = esp_timer_get_time();
  if (!g_baseKnown || !g_haveEcp || !g_st->inWakeUpMode()) return;
  if (g_tagActive || g_awaitFieldClear || g_lpcdWakePending || now < g_lpcdActiveUntilUs) return;
  if (gpio_get_level(PIN_BUTTON) == 0 || gpio_get_level(PIN_NFC_IRQ) != 0) return;
  if (!g_hbAcked && now < g_hbAckDeadlineUs) return;
  if (now - g_lastTxUs < TX_DRAIN_US) return;
  if (rtcNowUs() - g_rtc.lastArmUs > LPCD_REARM_SLEEP_US) {
    // Hours asleep: refresh the wake-up reference before the next stretch.
    g_st->stopWakeUpMode();
    lpcdArm();
    return;  // sleep on a later pass, once it is armed again
  }
  // The minimum awake time comes last, so the log can say when the doorbell
  // was actually done (bench builds stay up 3 s for the USB console).
  if (!g_readyToSleepAtUs) g_readyToSleepAtUs = now;
  if (now < g_stayAwakeUntilUs) return;
  enterDeepSleep();
}

// Poll our own reader once; announce to the base if a card is there.
void pollOnce() {
  if (g_readerReady && !g_reader->isConnected()) {
    // The reader driver gave up on the bus (see DoorbellPn532Reader): tear the
    // transport down so the re-init below starts from a fresh bus.
    ESP_LOGW(TAG, "NFC reader stopped answering; will re-initialise");
    g_reader->stop();
    g_readerReady = false;
  }
  if (!g_readerReady) {
    if (esp_timer_get_time() - g_lastInitTryUs > 5000000) {
      g_lastInitTryUs = esp_timer_get_time();
      readerInit();
    }
    return;
  }
  if (lpcdGate()) return;  // resting in low-power detection
  std::vector<uint8_t> uid;
  std::array<uint8_t, 2> atqa{};
  uint8_t sak = 0;
  // The base's 500 ms listen window is a PN532 figure, where it is only a
  // ceiling. The ST25R3916 driver waits it out when the phone is not answering
  // yet, which made a wake-up take ~585 ms to find the card. A short window
  // repeats ECP + WUPA every few tens of ms instead, and the phone answers as
  // soon as it is ready.
  constexpr uint32_t ST25R3916_LISTEN_MS = 25;
  const uint32_t listenMs = g_st ? std::min<uint32_t>(g_listenMs, ST25R3916_LISTEN_MS) : g_listenMs;
  const bool found = g_reader->pollForTag(uid, atqa, sak, listenMs);
  if (g_awaitFieldClear) {
    if (found) {
      g_fieldClearPolls = 0;
      g_reader->releaseTag();  // same phone, still here: not a new tap
    } else if (++g_fieldClearPolls >= 2) {
      g_awaitFieldClear = false;  // the field is clear; the next card is a new tap
    }
    return;
  }
  if (!found) return;
  std::vector<uint8_t> out;
  out.push_back(uint8_t(uid.size()));
  out.insert(out.end(), uid.begin(), uid.end());
  out.push_back(atqa[0]);
  out.push_back(atqa[1]);
  out.push_back(sak);
  appendBattery(out, false);
  g_tagActive = true;  // hold the card; the base will drive APDUs now
  g_tagActiveUs = esp_timer_get_time();
  g_tagPayload = out;
  g_tagAnnouncedUs = g_tagActiveUs;
  g_tagAnnounceTries = 1;
  send(g_base, relay::Op::TagEvent, 0, 3, out.data(), out.size());
  ++g_rtc.taps;
  g_lpcdCardSeen = true;
  if (!g_tagAnnouncedAtUs) g_tagAnnouncedAtUs = g_tagActiveUs;
  if (g_lpcdWakePending) {
    g_lpcdWakePending = false;
    ++g_lpcdWakesWithCard;
    ESP_LOGI(TAG, "tag detected %lld ms after wake-up, announced to base",
             (g_tagActiveUs - g_lpcdWakeUs) / 1000);
  } else {
    ESP_LOGI(TAG, "tag detected, announced to base");
  }
}

void sendButtonPress() {
  if (!g_baseKnown) {
    ESP_LOGW(TAG, "button pressed but no base is paired");
    return;
  }
  ESP_LOGI(TAG, "button pressed; telling the base");
  std::vector<uint8_t> payload;
  appendBattery(payload, false);
  send(g_base, relay::Op::ButtonPress, 0, 0, payload.data(), payload.size());
}

// Debounced press detection while awake. When asleep, the same pin wakes the
// chip, and a button wake sends the press straight away (see app_main).
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
  sendButtonPress();
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
      g_awaitFieldClear = true;  // ...but only once this phone has left
      g_fieldClearSinceUs = esp_timer_get_time();
      g_fieldClearPolls = 0;
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

// rounds == 0: keep sweeping until a base answers. A sleeping doorbell passes a
// limit, so an unreachable base costs a few seconds rather than the battery.
void findBase(int rounds = 0) {
  // The base sits on its access point's channel; walk the 2.4 GHz channels
  // until it answers a ping. The channel we last found it on is tried first,
  // so the usual case costs one ping rather than a sweep.
  uint8_t seq = 0;
  const uint8_t first = (g_channel >= 1 && g_channel <= 13) ? g_channel : 1;
  std::vector<uint8_t> order{first};
  for (uint8_t ch = 1; ch <= 13; ++ch)
    if (ch != first) order.push_back(ch);
  for (int round = 0; !g_baseKnown && (rounds == 0 || round < rounds); ++round) {
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

// The XIAO ESP32C6 routes its antenna through an RF switch: GPIO3 low enables
// the switch, GPIO14 low selects the built-in ceramic antenna (high: the U.FL
// connector). Seeed's Arduino board package sets both; plain ESP-IDF leaves them
// floating. That happened to work until deep sleep, after which the doorbell's
// frames reached the base at -94 dBm instead of -51..-67, too weak to hear the
// base's replies at all. Neither pin is broken out, so driving them is safe.
void selectBuiltInAntenna() {
  constexpr gpio_num_t RF_SWITCH_EN = GPIO_NUM_3;   // an LP pin: undo any sleep hold first
  constexpr gpio_num_t RF_ANTENNA_SEL = GPIO_NUM_14;
  rtc_gpio_hold_dis(RF_SWITCH_EN);
  rtc_gpio_deinit(RF_SWITCH_EN);
  gpio_config_t rf{};
  rf.pin_bit_mask = (1ULL << RF_SWITCH_EN) | (1ULL << RF_ANTENNA_SEL);
  rf.mode = GPIO_MODE_OUTPUT;
  gpio_config(&rf);
  gpio_set_level(RF_SWITCH_EN, 0);
  gpio_set_level(RF_ANTENNA_SEL, 0);
}

extern "C" void app_main() {
  selectBuiltInAntenna();  // before the radio comes up
  const esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  const uint64_t ext1 = cause == ESP_SLEEP_WAKEUP_EXT1 ? esp_sleep_get_ext1_wakeup_status() : 0;
  g_fromSleep = (cause == ESP_SLEEP_WAKEUP_EXT1 || cause == ESP_SLEEP_WAKEUP_TIMER) &&
                g_rtc.magic == RTC_MAGIC;
  // The wake pins come back routed to the low-power domain and held; give them
  // back to the GPIO matrix before reading them.
  for (gpio_num_t pin : {PIN_BUTTON, PIN_NFC_IRQ}) {
    rtc_gpio_hold_dis(pin);
    rtc_gpio_deinit(pin);
  }
  if (g_fromSleep) {
    ++g_rtc.wakes;
    g_wokeByNfc = ext1 & (1ULL << PIN_NFC_IRQ);
    g_wokeByButton = ext1 & (1ULL << PIN_BUTTON);
    g_wokeByTimer = cause == ESP_SLEEP_WAKEUP_TIMER;
    if (g_wokeByNfc) ++g_rtc.nfcWakes;
    if (g_wokeByButton) ++g_rtc.buttonWakes;
    if (g_wokeByTimer) ++g_rtc.timerWakes;
    g_stayAwakeUntilUs = WAKE_MIN_AWAKE_US;
  } else {
    std::memset(&g_rtc, 0, sizeof(g_rtc));
    g_stayAwakeUntilUs = COLD_BOOT_AWAKE_US;
  }

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
  const uint16_t mv = g_batteryMv = readBatteryMv();
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
  // A wake after a failed search starts over like a cold boot (minus the
  // 20 s stay-awake window); any other wake resumes the link.
  const bool resumeLink = g_fromSleep && !g_rtc.baseLost;
  if (resumeLink) {
    // Restore what the cold boot learnt: no channel sweep, no ECP request and
    // no drive sweep on a wake.
    std::memcpy(g_base, g_rtc.base, 6);
    g_channel = g_rtc.channel;
    esp_wifi_set_channel(g_channel, WIFI_SECOND_CHAN_NONE);
    addPeer(g_base);
    g_baseKnown = true;
    std::memcpy(g_ecp.data(), g_rtc.ecp, g_ecp.size());
    g_listenMs = g_rtc.listenMs;
    g_pollDelayMs = g_rtc.pollDelayMs;
    g_haveEcp = true;
  }
  if (g_fromSleep && g_rtc.dresSwept) {
    g_lpcdDres = g_rtc.dres;
    g_lpcdSwept = true;
  }
  // Also after a base-less sleep: a tap woke us, and resuming (rather than
  // resetting) the reader keeps that wake-up event, so the phone is polled as
  // soon as the base answers and sends the ECP frame.
  const bool resumed = g_fromSleep && g_rtc.readerInWakeUp &&
                       g_readerKind == ReaderKind::St25r3916 && readerResume();
  if (!resumed && !readerInit()) ESP_LOGE(TAG, "continuing without a working NFC reader");
  if (!resumeLink) {
    if (!sleepAllowedNow()) {
      findBase();  // on a computer: keep looking, as before
    } else {
      // Bounded, but never shorter than the cold boot's stay-awake window,
      // which is there so the board can be flashed.
      findBase(BASE_SEARCH_ROUNDS);
      int64_t searchUntil = g_stayAwakeUntilUs;
      if (g_fromSleep && (g_wokeByNfc || g_wokeByButton))
        searchUntil = std::max<int64_t>(searchUntil, BASE_SEARCH_ATTENDED_US);
      while (!g_baseKnown && esp_timer_get_time() < searchUntil) findBase(1);
      if (!g_baseKnown) sleepWithoutBase();
    }
    if (g_rtc.baseLost) ESP_LOGI(TAG, "base found after %u failed searches", g_rtc.searchFails);
    g_rtc.lastHeartbeatUs = 0;  // heartbeat (with the battery) right away
    g_rtc.baseLost = 0;
    g_rtc.searchFails = 0;
  }
  if (g_wokeByButton) {
    // Send it now: a short press may already be over, and checkButton() would
    // never see it.
    g_lastPressUs = esp_timer_get_time();
    g_buttonLast = gpio_get_level(PIN_BUTTON);
    sendButtonPress();
  }

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
      const int64_t heartbeatEvery = sleepAllowedNow() ? HEARTBEAT_SLEEP_US : HEARTBEAT_US;
      if (g_baseKnown && !g_tagActive && rtcNowUs() - g_rtc.lastHeartbeatUs > heartbeatEvery) {
        g_rtc.lastHeartbeatUs = rtcNowUs();
        std::vector<uint8_t> hb;
        appendBattery(hb, true);
        send(g_base, relay::Op::HealthRsp, 0, g_readerReady ? 2 : 0, hb.data(), hb.size());
        g_hbAcked = false;
        g_hbTries = 1;
        g_hbAckDeadlineUs = esp_timer_get_time() + HEARTBEAT_ACK_WAIT_US;
        // Pick up a new ECP frame if HomeKit was re-provisioned while we slept:
        // the base cannot push one to a sleeping doorbell.
        if (sleepAllowedNow()) send(g_base, relay::Op::EcpReq, 0, 0, nullptr, 0);
      }
      if (!g_hbAcked && esp_timer_get_time() > g_hbAckDeadlineUs && g_hbTries < HEARTBEAT_TRIES) {
        // Resend before concluding anything: a single frame is easily lost.
        ++g_hbTries;
        std::vector<uint8_t> hb;
        appendBattery(hb, false);
        send(g_base, relay::Op::HealthRsp, 0, g_readerReady ? 2 : 0, hb.data(), hb.size());
        g_hbAckDeadlineUs = esp_timer_get_time() + HEARTBEAT_ACK_WAIT_US;
      } else if (!g_hbAcked && esp_timer_get_time() > g_hbAckDeadlineUs) {
        // No ack after several tries: the base restarted or its AP moved
        // channel. Search, but a sleeping doorbell only sweeps twice: if the
        // base is still unreachable it keeps the pinned base and its last
        // channel, sleeps, and tries again in HEARTBEAT_RETRY_US. (Giving up by
        // forgetting the base left it awake for good: "base unknown" blocks
        // sleep, and nothing searched again.)
        g_hbAcked = true;
        ESP_LOGW(TAG, "heartbeat not acknowledged after %d tries; searching for the base", g_hbTries);
        const uint8_t lastChannel = g_channel;
        g_baseKnown = false;
        findBase(sleepAllowedNow() ? 2 : 0);
        if (g_baseKnown) {
          // Found again, usually on a new channel after the base restarted. The
          // heartbeat never arrived, and the Ping that found the base carries no
          // battery, so the base would show "not fitted" for another hour: send
          // the heartbeat again now (and stay up for its ack).
          std::vector<uint8_t> hb;
          appendBattery(hb, false);
          send(g_base, relay::Op::HealthRsp, 0, g_readerReady ? 2 : 0, hb.data(), hb.size());
          g_hbAcked = false;
          g_hbTries = 1;
          g_hbAckDeadlineUs = esp_timer_get_time() + HEARTBEAT_ACK_WAIT_US;
        } else if (g_basePinned) {
          g_baseKnown = true;
          g_channel = lastChannel;
          esp_wifi_set_channel(g_channel, WIFI_SECOND_CHAN_NONE);
          g_rtc.lastHeartbeatUs = rtcNowUs() - heartbeatEvery + HEARTBEAT_RETRY_US;
          ESP_LOGW(TAG, "base unreachable; keeping it on channel %u and retrying in %lld s",
                   g_channel, HEARTBEAT_RETRY_US / 1000000);
        }
        lastRequestUs = esp_timer_get_time();
      }
      // Silence means the base restarted or its AP moved channel. This has to
      // run before the poll-and-continue below, or a polling doorbell never
      // notices and stays parked on a dead channel until it is power-cycled.
      // The base acknowledges each heartbeat, so silence is measured against
      // 1.5 heartbeat intervals: with the two equal, this check ran in the same
      // iteration that had just sent a heartbeat, before its ack could arrive,
      // and re-scanned every 30 s for nothing.
      if (g_baseKnown && esp_timer_get_time() - lastRequestUs > BASE_SILENCE_US) {
        ESP_LOGW(TAG, "no word from the base for %lld s; searching for it again",
                 BASE_SILENCE_US / 1000000);
        g_baseKnown = false;
        findBase();
        lastRequestUs = esp_timer_get_time();
      }
      if (g_haveEcp && !g_tagActive) {
        pollOnce();
        maybeDeepSleep();
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
        g_awaitFieldClear = true;
      g_fieldClearSinceUs = esp_timer_get_time();
        g_fieldClearPolls = 0;
      }
      continue;
    }
    lastRequestUs = esp_timer_get_time();
    g_hbAcked = true;        // anything from the base answers the heartbeat
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
      payload = std::move(g_asm.buf);  // re-assigned on the next first fragment
      g_asm.op = 0;
    }
    handle(m, h, payload);
  }
}
