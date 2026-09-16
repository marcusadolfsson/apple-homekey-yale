#include "YaleBleLock.hpp"

#include "LockManager.hpp"
#include "app_events.hpp"
#include "config.hpp"
#include "eventStructs.hpp"

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <vector>

YaleBleLock *YaleBleLock::s_instance = nullptr;
std::atomic<bool> g_bleRadioBusy{false};
std::atomic<bool> g_bleLinkAttempt{false};

namespace {

// 0000fe24-0000-1000-8000-00805f9b34fb
const ble_uuid16_t SVC_UUID = BLE_UUID16_INIT(0xFE24);
// NimBLE stores 128-bit UUIDs little-endian: the string read backwards.
// bd4ac611-0b45-11e3-8ffd-0800200c9a66 (write)
const ble_uuid128_t UUID_WRITE = BLE_UUID128_INIT(0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0xfd, 0x8f,
                                                  0xe3, 0x11, 0x45, 0x0b, 0x11, 0xc6, 0x4a, 0xbd);
// bd4ac612-... (read/notify)
const ble_uuid128_t UUID_READ = BLE_UUID128_INIT(0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0xfd, 0x8f,
                                                 0xe3, 0x11, 0x45, 0x0b, 0x12, 0xc6, 0x4a, 0xbd);
// bd4ac613-... (secure write)
const ble_uuid128_t UUID_SEC_WRITE = BLE_UUID128_INIT(0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0xfd, 0x8f,
                                                      0xe3, 0x11, 0x45, 0x0b, 0x13, 0xc6, 0x4a, 0xbd);
// bd4ac614-... (secure read/notify)
const ble_uuid128_t UUID_SEC_READ = BLE_UUID128_INIT(0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0xfd, 0x8f,
                                                     0xe3, 0x11, 0x45, 0x0b, 0x14, 0xc6, 0x4a, 0xbd);
const ble_uuid16_t UUID_CCCD = BLE_UUID16_INIT(0x2902);

constexpr uint8_t OP_GETSTATUS = 0x02;
constexpr uint8_t OP_UNLOCK = 0x0A;
constexpr uint8_t OP_LOCK = 0x0B;
constexpr uint8_t STATUS_LOCK_ONLY = 0x02;
constexpr uint8_t STATUS_DOOR_AND_LOCK = 0x2F;

// A scan holds the shared radio, and the relay reader drops tag announcements
// while it runs: with the lock out of range a 30 s scan blinded the doorbell
// for half a minute after every tap. In range the lock is found in ~2.6 s.
constexpr uint32_t SCAN_MS = 8000;
constexpr uint32_t CONNECT_MS = 10000;
constexpr uint32_t DIRECT_CONNECT_MS = 4000;  // cached-address attempt
constexpr uint32_t GATT_MS = 5000;
constexpr uint32_t OP_RESULT_MS = 15000;
// Match yalexs-ble's default (DISCONNECT_DELAY 5.1 s): the lock shares one radio
// with the Yale app, Apple Home and HA, so let go promptly after each command.
constexpr uint32_t LINGER_MS = 5000;
// Faster connection interval than NimBLE's default so the handshake's GATT round
// trips finish sooner; the link only lives a few seconds.
constexpr uint16_t CONN_ITVL_MIN = 12;     // 15 ms
constexpr uint16_t CONN_ITVL_MAX = 24;     // 30 ms
constexpr uint16_t SUPERVISION_TMO = 400;  // 4 s

constexpr const char *NVS_NS = "yaleble";
constexpr const char *NVS_KEY = "gatt";
struct __attribute__((packed)) GattCache {
  uint8_t version;
  uint8_t mac[6];
  uint8_t addrType;
  uint16_t hWrite, hRead, hReadCccd, hSecWrite, hSecRead, hSecReadCccd;
  uint8_t readProps, secReadProps;
};
constexpr uint8_t CACHE_VERSION = 1;

ble_gap_conn_params connParams() {
  ble_gap_conn_params cp{};
  cp.scan_itvl = 0x0010;
  cp.scan_window = 0x0010;
  cp.itvl_min = CONN_ITVL_MIN;
  cp.itvl_max = CONN_ITVL_MAX;
  cp.latency = 0;
  cp.supervision_timeout = SUPERVISION_TMO;
  return cp;
}
constexpr int64_t COOLDOWN_US = 250 * 1000;

// Short gaps between writes are what the library's cooldown exists for: sending
// too fast can crash the lock's radio until its batteries are pulled.
constexpr uint32_t DEDUPE_MS = 2000;

std::vector<std::pair<uint16_t, uint16_t>> s_chrs;  // (def_handle, val_handle)
std::vector<uint64_t> s_seenYale;  // scan diagnostics
uint32_t s_advReports = 0;

uint32_t u32le(const uint8_t *p) {
  return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}

uint32_t securityChecksum(const uint8_t *f) {
  // yalexs-ble sums buffer[8:18] as one int; only its low 32 bits survive the mask.
  return uint32_t(0) - (u32le(f) + u32le(f + 4) + u32le(f + 8));
}

uint8_t simpleChecksum(const uint8_t *f) {
  uint32_t sum = 0;
  for (size_t i = 0; i < 18; ++i) sum += f[i];
  return uint8_t((0u - sum) & 0xFF);
}

bool parseHex(const std::string &s, uint8_t *out, size_t n) {
  if (s.size() != n * 2) return false;
  for (size_t i = 0; i < n; ++i) {
    auto nib = [](char c) -> int {
      if (c >= '0' && c <= '9') return c - '0';
      c = char(std::tolower(static_cast<unsigned char>(c)));
      if (c >= 'a' && c <= 'f') return c - 'a' + 10;
      return -1;
    };
    int hi = nib(s[2 * i]), lo = nib(s[2 * i + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i] = uint8_t(hi << 4 | lo);
  }
  return true;
}

bool parseMac(const std::string &s, std::array<uint8_t, 6> &out) {
  std::string hex;
  for (char c : s) {
    if (c != ':' && c != '-') hex.push_back(c);
  }
  return parseHex(hex, out.data(), 6);
}

const char *cmdName(YaleBleLock::Cmd c) {
  switch (c) {
    case YaleBleLock::Cmd::Status: return "status";
    case YaleBleLock::Cmd::Unlock: return "unlock";
    case YaleBleLock::Cmd::Lock: return "lock";
    case YaleBleLock::Cmd::Prepare: return "prepare";
  }
  return "?";
}

}  // namespace

YaleBleLock::YaleBleLock(const espConfig::misc_config_t &misc) {
  s_instance = this;
  m_enabled = misc.yaleBleEnabled;
  m_slot = misc.yaleBleSlot;
  m_alwaysUnlock = misc.lockAlwaysUnlock;
  if (!m_enabled) return;
  if (!parseMac(misc.yaleBleMac, m_mac)) {
    ESP_LOGE(TAG, "Yale BLE enabled but MAC '%s' is invalid (expected AA:BB:CC:DD:EE:FF)", misc.yaleBleMac.c_str());
    m_enabled = false;
  }
  if (!parseHex(misc.yaleBleOfflineKey, m_offlineKey.data(), 16)) {
    // Never echo the key itself.
    ESP_LOGE(TAG, "Yale BLE enabled but offline key is not 32 hex characters (got %u chars)",
             (unsigned)misc.yaleBleOfflineKey.size());
    m_enabled = false;
  }
  mbedtls_aes_init(&m_ecbEnc);
  mbedtls_aes_init(&m_ecbDec);
  mbedtls_aes_init(&m_cbcEnc);
  mbedtls_aes_init(&m_cbcDec);
}

YaleBleLock::~YaleBleLock() {
  mbedtls_aes_free(&m_ecbEnc);
  mbedtls_aes_free(&m_ecbDec);
  mbedtls_aes_free(&m_cbcEnc);
  mbedtls_aes_free(&m_cbcDec);
  m_offlineKey.fill(0);
  s_instance = nullptr;
}

void YaleBleLock::begin() {
  if (!m_enabled) {
    ESP_LOGI(TAG, "Yale BLE lock disabled");
    return;
  }
  m_events = xQueueCreate(24, sizeof(Ev));
  m_commands = xQueueCreate(4, sizeof(Cmd));

  m_nfcSub = AppEventLoop::subscribe(NFC_EVENT, NFC_TAP_EVENT, [this](const uint8_t *data, size_t size) {
    if (size == 0 || data == nullptr) return;
    std::error_code ec;
    std::span<const uint8_t> payload(data, size);
    NfcEvent ev = alpaca::deserialize<NfcEvent>(payload, ec);
    if (ec || ev.type != HOMEKEY_TAP) return;
    EventHKTap tap = alpaca::deserialize<EventHKTap>(ev.data, ec);
    if (ec || !tap.status) return;
    if (m_alwaysUnlock) {
      request(Cmd::Unlock);
    } else {
      ESP_LOGW(TAG, "HomeKey tap ignored: enable 'Always Unlock' so a tap opens the Yale");
    }
  });
  // NOTE: do not start the BLE session when a card is merely detected. Tried and
  // reverted: one 2.4 GHz radio means a connect attempt running alongside the card
  // exchange corrupted APDUs ("Auth0 response invalid") and pushed connect from
  // 1.9 s to 15.8 s. The unlock request arrives ~300 ms later anyway.
  m_targetSub = AppEventLoop::subscribe(LOCK_EVENT, LOCK_TARGET_STATE_CHANGED, [this](const uint8_t *data, size_t size) {
    if (size == 0 || data == nullptr) return;
    std::error_code ec;
    std::span<const uint8_t> payload(data, size);
    EventLockState s = alpaca::deserialize<EventLockState>(payload, ec);
    if (ec) return;
    // Unlock only: the mortise relocks itself.
    if (s.targetState == LockManager::UNLOCKED) request(Cmd::Unlock);
  });

  loadCache();
  xTaskCreate(taskEntry, "yale_ble", 6144, this, 5, &m_task);
}

void YaleBleLock::abortLinkAttempt() {
  if (!g_bleLinkAttempt.load(std::memory_order_acquire)) return;
  if (s_instance) s_instance->m_attemptAborted = true;
  ble_gap_conn_cancel();  // BLE_HS_EALREADY when nothing is connecting: harmless
  ble_gap_disc_cancel();
}

void YaleBleLock::request(Cmd cmd) {
  if (!m_events) return;
  Ev ev{};
  ev.type = EvType::Command;
  ev.cmd = cmd;
  post(ev);
}

void YaleBleLock::post(const Ev &ev) {
  // Called from the NimBLE host task: must not block or log.
  if (m_events) (void)xQueueSend(m_events, &ev, 0);
}

// ------------------------------------------------------------------ NimBLE

void YaleBleLock::onSync() {
  if (!s_instance) return;
  Ev ev{};
  ev.type = EvType::Synced;
  s_instance->post(ev);
}

void YaleBleLock::onReset(int reason) {
  ESP_LOGW(TAG, "NimBLE host reset, reason=%d", reason);
}

void YaleBleLock::hostTask(void *) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}

int YaleBleLock::gapEvent(ble_gap_event *event, void *arg) {
  auto *self = static_cast<YaleBleLock *>(arg);
  Ev ev{};
  switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
      const auto &a = event->disc.addr;
      s_advReports++;
      {
        ble_hs_adv_fields fields{};
        if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) == 0) {
          bool yaleish = false;
          char name[32] = {0};
          if (fields.name && fields.name_len > 0) {
            std::memcpy(name, fields.name, std::min<size_t>(fields.name_len, sizeof(name) - 1));
            if (fields.name_len == 7 && name[0] == 'M') yaleish = true;  // e.g. M50067E
          }
          if (fields.mfg_data && fields.mfg_data_len >= 2) {
            uint16_t company = fields.mfg_data[0] | fields.mfg_data[1] << 8;
            if (company == 465) yaleish = true;  // Yale/August
            // HomeKit-over-BLE accessory advert (Apple company id, type 0x06); the
            // Yale Access module advertises this too.
            if (company == 76 && fields.mfg_data_len >= 3 && fields.mfg_data[2] == 0x06) yaleish = true;
          }
          uint64_t key = 0;
          for (int i = 0; i < 6; ++i) key |= uint64_t(a.val[i]) << (8 * i);
          if (yaleish && std::find(s_seenYale.begin(), s_seenYale.end(), key) == s_seenYale.end()) {
            // Logging here runs on the NimBLE host task's small stack and can stall
            // it; hand the report to the worker instead.
            s_seenYale.push_back(key);
            Ev seen{};
            seen.type = EvType::DiscSeen;
            seen.status = event->disc.rssi;
            seen.addrType = a.type;
            for (int i = 0; i < 6; ++i) seen.addr[i] = a.val[5 - i];
            std::memcpy(seen.name, name, sizeof(seen.name) - 1);
            self->post(seen);
          }
        }
      }
      bool match = true;
      for (int i = 0; i < 6; ++i) {
        // ble_addr_t.val is least-significant byte first.
        if (a.val[i] != self->m_mac[5 - i]) { match = false; break; }
      }
      if (match && !self->m_scanMatched) {
        self->m_scanMatched = true;
        ev.type = EvType::DiscFound;
        ev.addrType = a.type;
        ev.status = event->disc.rssi;  // signal strength dominates connect time
        self->post(ev);
      }
      return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
      ev.type = EvType::DiscDone;
      self->post(ev);
      return 0;
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0) {
        self->m_conn = event->connect.conn_handle;
        ev.type = EvType::Connected;
      } else {
        ev.type = EvType::ConnectFailed;
        ev.status = event->connect.status;
      }
      self->post(ev);
      return 0;
    case BLE_GAP_EVENT_CONN_UPDATE:
    case BLE_GAP_EVENT_CONN_UPDATE_REQ: {
      ble_gap_conn_desc desc{};
      if (ble_gap_conn_find(self->m_conn, &desc) == 0) {
        self->m_connItvl = desc.conn_itvl;  // units of 1.25 ms
      }
      return 0;
    }
    case BLE_GAP_EVENT_DISCONNECT:
      self->m_conn = 0xFFFF;
      self->m_lastDisconnectUs = esp_timer_get_time();
      ev.type = EvType::Disconnected;
      ev.status = event->disconnect.reason;
      self->post(ev);
      return 0;
    case BLE_GAP_EVENT_NOTIFY_RX: {
      uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
      if (len != FRAME_LEN) {
        // A partial block would desynchronise the CBC chain; drop it (logged by the worker).
        ev.type = EvType::NotifyDropped;
        ev.status = len;
        self->post(ev);
        return 0;
      }
      os_mbuf_copydata(event->notify_rx.om, 0, FRAME_LEN, ev.frame.data());
      if (event->notify_rx.attr_handle == self->m_hSecRead) ev.type = EvType::NotifySecure;
      else if (event->notify_rx.attr_handle == self->m_hRead) ev.type = EvType::NotifyCmd;
      else return 0;
      self->post(ev);
      return 0;
    }
    default:
      return 0;
  }
}

int YaleBleLock::onSvc(uint16_t, const ble_gatt_error *error, const ble_gatt_svc *svc, void *arg) {
  auto *self = static_cast<YaleBleLock *>(arg);
  if (error->status == 0 && svc) {
    self->m_svcStart = svc->start_handle;
    self->m_svcEnd = svc->end_handle;
    return 0;
  }
  Ev ev{};
  ev.type = EvType::GattDone;
  ev.status = error->status == BLE_HS_EDONE ? 0 : error->status;
  self->post(ev);
  return 0;
}

int YaleBleLock::onChr(uint16_t, const ble_gatt_error *error, const ble_gatt_chr *chr, void *arg) {
  auto *self = static_cast<YaleBleLock *>(arg);
  if (error->status == 0 && chr) {
    s_chrs.emplace_back(chr->def_handle, chr->val_handle);
    if (ble_uuid_cmp(&chr->uuid.u, &UUID_WRITE.u) == 0) self->m_hWrite = chr->val_handle;
    else if (ble_uuid_cmp(&chr->uuid.u, &UUID_READ.u) == 0) { self->m_hRead = chr->val_handle; self->m_readProps = chr->properties; }
    else if (ble_uuid_cmp(&chr->uuid.u, &UUID_SEC_WRITE.u) == 0) self->m_hSecWrite = chr->val_handle;
    else if (ble_uuid_cmp(&chr->uuid.u, &UUID_SEC_READ.u) == 0) { self->m_hSecRead = chr->val_handle; self->m_secReadProps = chr->properties; }
    return 0;
  }
  Ev ev{};
  ev.type = EvType::GattDone;
  ev.status = error->status == BLE_HS_EDONE ? 0 : error->status;
  self->post(ev);
  return 0;
}

int YaleBleLock::onDsc(uint16_t, const ble_gatt_error *error, uint16_t, const ble_gatt_dsc *dsc, void *arg) {
  auto *self = static_cast<YaleBleLock *>(arg);
  if (error->status == 0 && dsc) {
    if (ble_uuid_cmp(&dsc->uuid.u, &UUID_CCCD.u) == 0) {
      if (self->m_dscTarget == self->m_hRead) self->m_hReadCccd = dsc->handle;
      else if (self->m_dscTarget == self->m_hSecRead) self->m_hSecReadCccd = dsc->handle;
    }
    return 0;
  }
  Ev ev{};
  ev.type = EvType::GattDone;
  ev.status = error->status == BLE_HS_EDONE ? 0 : error->status;
  self->post(ev);
  return 0;
}

int YaleBleLock::onWrite(uint16_t, const ble_gatt_error *error, ble_gatt_attr *, void *arg) {
  auto *self = static_cast<YaleBleLock *>(arg);
  Ev ev{};
  ev.type = EvType::GattDone;
  ev.status = error->status;
  self->post(ev);
  return 0;
}

// ------------------------------------------------------------------ worker

void YaleBleLock::taskEntry(void *arg) {
  static_cast<YaleBleLock *>(arg)->run();
}

bool YaleBleLock::waitFor(EvType type, uint32_t timeoutMs, Ev &out) {
  const int64_t deadline = esp_timer_get_time() + int64_t(timeoutMs) * 1000;
  while (true) {
    int64_t remainMs = (deadline - esp_timer_get_time()) / 1000;
    if (remainMs <= 0) return false;
    Ev ev{};
    if (xQueueReceive(m_events, &ev, pdMS_TO_TICKS(remainMs)) != pdTRUE) return false;
    if (ev.type == type) {
      out = ev;
      return true;
    }
    // A cancelled connect surfaces as ConnectFailed, a cancelled scan as
    // DiscDone. Waiting on for the timeout after either would defeat the abort.
    if ((type == EvType::Connected && ev.type == EvType::ConnectFailed) ||
        (type == EvType::DiscFound && ev.type == EvType::DiscDone)) {
      return false;
    }
    switch (ev.type) {
      case EvType::Command:
        // Defer; handled once the current operation finishes.
        xQueueSend(m_commands, &ev.cmd, 0);
        break;
      case EvType::NotifyCmd: {
        // Every command-channel frame must pass through the CBC chain in order,
        // including unsolicited pushes that arrive while waiting for something else.
        Frame f = ev.frame;
        cbcDecrypt(f.data());
        if (simpleChecksum(f.data()) == 0 && (f[0] == 0xAA || f[0] == 0xBB)) {
          m_lastRxUs = esp_timer_get_time();
          handleCommandFrame(f);
        }
        break;
      }
      case EvType::DiscSeen:
        ESP_LOGD(TAG, "scan: Yale-like device %02X:%02X:%02X:%02X:%02X:%02X type=%u rssi=%d name='%s'",
                 ev.addr[0], ev.addr[1], ev.addr[2], ev.addr[3], ev.addr[4], ev.addr[5], ev.addrType, ev.status, ev.name);
        break;
      case EvType::NotifyDropped:
        ESP_LOGW(TAG, "dropped %d-byte notification", ev.status);
        break;
      case EvType::Disconnected:
        m_sessionReady = false;
        if (type != EvType::Disconnected) {
          ESP_LOGW(TAG, "lock disconnected (reason 0x%x)", ev.status);
          return false;
        }
        break;
      default:
        break;
    }
  }
}

void YaleBleLock::run() {
  esp_err_t err = nimble_port_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_init failed: %d", err);
    vTaskDelete(nullptr);
    return;
  }
  ble_hs_cfg.sync_cb = onSync;
  ble_hs_cfg.reset_cb = onReset;
  nimble_port_freertos_init(hostTask);

  Ev ev{};
  if (!waitFor(EvType::Synced, 10000, ev)) {
    ESP_LOGE(TAG, "NimBLE host did not sync");
    vTaskDelete(nullptr);
    return;
  }
  ble_hs_util_ensure_addr(0);
  ble_hs_id_infer_auto(0, &m_ownAddrType);
  ESP_LOGI(TAG, "BLE ready; lock %02X:%02X:%02X:%02X:%02X:%02X slot %u",
           m_mac[0], m_mac[1], m_mac[2], m_mac[3], m_mac[4], m_mac[5], m_slot);

  Cmd lastCmd = Cmd::Status;
  int64_t lastCmdUs = 0;
  int64_t lingerUntilUs = 0;

  while (true) {
    Cmd cmd;
    if (xQueueReceive(m_commands, &cmd, 0) == pdTRUE) {
      int64_t now = esp_timer_get_time();
      if (cmd != Cmd::Status && cmd != Cmd::Prepare && cmd == lastCmd &&
          now - lastCmdUs < int64_t(DEDUPE_MS) * 1000) {
        ESP_LOGD(TAG, "coalescing duplicate %s request", cmdName(cmd));
        continue;
      }
      lastCmd = cmd;
      lastCmdUs = now;
      performCommand(cmd);
      lingerUntilUs = esp_timer_get_time() + int64_t(LINGER_MS) * 1000;
      continue;
    }

    TickType_t wait = portMAX_DELAY;
    if (m_conn != 0xFFFF) {
      int64_t remainMs = (lingerUntilUs - esp_timer_get_time()) / 1000;
      if (remainMs <= 0) {
        disconnect();
        continue;
      }
      wait = pdMS_TO_TICKS(remainMs);
    }
    if (xQueueReceive(m_events, &ev, wait) != pdTRUE) {
      continue;  // linger expired; loop disconnects
    }
    switch (ev.type) {
      case EvType::Command:
        xQueueSend(m_commands, &ev.cmd, 0);
        break;
      case EvType::NotifyCmd: {
        Frame f = ev.frame;
        cbcDecrypt(f.data());
        if (simpleChecksum(f.data()) == 0 && (f[0] == 0xAA || f[0] == 0xBB)) {
          m_lastRxUs = esp_timer_get_time();
          handleCommandFrame(f);
        } else {
          ESP_LOGW(TAG, "invalid pushed frame dropped");
        }
        break;
      }
      case EvType::Disconnected:
        m_sessionReady = false;
        g_bleRadioBusy.store(false, std::memory_order_release);
        ESP_LOGI(TAG, "lock disconnected (reason 0x%x)", ev.status);
        break;
      default:
        break;
    }
  }
}

void YaleBleLock::performCommand(Cmd cmd) {
  ESP_LOGI(TAG, "%s requested", cmdName(cmd));
  const int64_t t0 = esp_timer_get_time();
  g_bleRadioBusy.store(true, std::memory_order_release);
  struct BusyGuard {
    ~BusyGuard() { g_bleRadioBusy.store(false, std::memory_order_release); }
  } busyGuard;
  if (!ensureConnected()) {
    ESP_LOGE(TAG, "%s failed: could not establish a session with the lock", cmdName(cmd));
    return;
  }
  Frame resp{};
  switch (cmd) {
    case Cmd::Prepare:
      // ensureConnected() above did the work; hold the session for the linger
      // window so the unlock that usually follows needs no connect at all.
      ESP_LOGI(TAG, "session prepared in %lld ms", (esp_timer_get_time() - t0) / 1000);
      break;
    case Cmd::Status:
      if (sendCommand(OP_GETSTATUS, STATUS_LOCK_ONLY, 0xBB, OP_GETSTATUS, STATUS_LOCK_ONLY, GATT_MS, resp)) {
        ESP_LOGI(TAG, "status read in %lld ms", (esp_timer_get_time() - t0) / 1000);
      }
      break;
    case Cmd::Unlock:
    case Cmd::Lock: {
      const bool unlock = cmd == Cmd::Unlock;
      const uint8_t op = unlock ? OP_UNLOCK : OP_LOCK;
      // Return on the lock's ack (0xAA, tens of ms), not its result (0xBB,
      // ~1.8 s later when the motor stops). Nothing here depends on the
      // result - state is not tracked - but waiting for it held the shared
      // radio, and with it every tap, for the whole mechanical cycle. The
      // result still arrives during the linger and handleCommandFrame() logs
      // it. expectFlag 0: take the result if it somehow comes first.
      if (sendCommand(op, 0x00, 0, op, -1, GATT_MS, resp)) {
        if (resp[0] == 0xBB) {
          const uint8_t result = resp[0x0F];
          if (result == 0x00) ESP_LOGI(TAG, "%s succeeded in %lld ms", cmdName(cmd), (esp_timer_get_time() - t0) / 1000);
          else ESP_LOGE(TAG, "%s failed: lock reported error 0x%02X", cmdName(cmd), result);
        } else {
          ESP_LOGI(TAG, "%s accepted by the lock in %lld ms", cmdName(cmd), (esp_timer_get_time() - t0) / 1000);
        }
      } else {
        ESP_LOGE(TAG, "%s: lock did not acknowledge the command", cmdName(cmd));
      }
      break;
    }
  }
}

bool YaleBleLock::ensureConnected() {
  if (m_conn != 0xFFFF && m_sessionReady) return true;
  if (m_conn != 0xFFFF) disconnect();
  const int64_t t0 = esp_timer_get_time();
  const bool justDisconnected =
      m_lastDisconnectUs != 0 && esp_timer_get_time() - m_lastDisconnectUs < 10000000;
  // Right after we drop a session the lock takes a moment to advertise again.
  // A direct connect waits for that advertisement exactly as a scan would, so
  // there is no reason to scan here: just allow it more time.
  const bool tryDirect = m_addrKnown;
  m_attemptAborted = false;
  bool connected = tryDirect && connectDirect(justDisconnected ? CONNECT_MS : DIRECT_CONNECT_MS);
  if (!connected && m_attemptAborted) {
    ESP_LOGI(TAG, "link attempt aborted for a tap; its unlock will start a new one");
    return false;
  }
  if (!connected) {
    if (tryDirect) ++m_directFailures;
    const bool scanWorthIt = !m_addrKnown ||
                             m_directFailures >= DIRECT_FAILURES_BEFORE_SCAN;
    if (!scanWorthIt) {
      ESP_LOGW(TAG, "direct connect failed (%d in a row); not scanning - lock is most likely out of range",
               m_directFailures);
      return false;
    }
    if (!scanAndConnect()) {
      if (m_attemptAborted) ESP_LOGI(TAG, "scan aborted for a tap; its unlock will start a new one");
      return false;
    }
  }
  m_directFailures = 0;
  {
    // The lock may negotiate a slow interval, which stretches every handshake
    // round trip; ask for a fast one and report what we ended up with.
    ble_gap_upd_params up{};
    up.itvl_min = CONN_ITVL_MIN;
    up.itvl_max = CONN_ITVL_MAX;
    up.latency = 0;
    up.supervision_timeout = SUPERVISION_TMO;
    up.min_ce_len = 0;
    up.max_ce_len = 0;
    int urc = ble_gap_update_params(m_conn, &up);
    ble_gap_conn_desc desc{};
    if (ble_gap_conn_find(m_conn, &desc) == 0) m_connItvl = desc.conn_itvl;
    ESP_LOGI(TAG, "connection interval %u ms (update rc=%d)", unsigned(m_connItvl * 5 / 4), urc);
  }
  if (!m_gattCached) {
    if (!discover()) { disconnect(); return false; }
    m_gattCached = true;
    saveCache();
  }
  // Order matters: secure notifications first, then the handshake, then the
  // command channel (as yalexs-ble does).
  if (!writeCccd(m_hSecReadCccd, m_secReadProps, "secure read")) { clearCache(); disconnect(); return false; }
  if (!handshake()) { disconnect(); return false; }
  if (!writeCccd(m_hReadCccd, m_readProps, "command read")) { disconnect(); return false; }
  m_sessionReady = true;
  ESP_LOGI(TAG, "session ready in %lld ms", (esp_timer_get_time() - t0) / 1000);
  return true;
}

bool YaleBleLock::connectDirect(uint32_t timeoutMs) {
  const int64_t t0 = esp_timer_get_time();
  struct AttemptGuard { AttemptGuard() { g_bleLinkAttempt.store(true, std::memory_order_release); }
                        ~AttemptGuard() { g_bleLinkAttempt.store(false, std::memory_order_release); } } attempt;
  ble_addr_t peer{};
  peer.type = m_peerAddrType;
  for (int i = 0; i < 6; ++i) peer.val[i] = m_mac[5 - i];
  const ble_gap_conn_params cp = connParams();
  int rc = ble_gap_connect(m_ownAddrType, &peer, int32_t(timeoutMs), &cp, gapEvent, this);
  Ev ev{};
  if (rc != 0 || !waitFor(EvType::Connected, timeoutMs + 500, ev)) {
    if (rc == 0) ble_gap_conn_cancel();
    if (!m_attemptAborted) ESP_LOGW(TAG, "direct connect failed (rc=%d) after %u ms", rc, unsigned(timeoutMs));
    return false;
  }
  ESP_LOGI(TAG, "connected directly in %lld ms", (esp_timer_get_time() - t0) / 1000);
  return true;
}

bool YaleBleLock::scanAndConnect() {
  const int64_t t0 = esp_timer_get_time();
  struct AttemptGuard { AttemptGuard() { g_bleLinkAttempt.store(true, std::memory_order_release); }
                        ~AttemptGuard() { g_bleLinkAttempt.store(false, std::memory_order_release); } } attempt;
  ble_gap_disc_params dp{};
  dp.filter_duplicates = 0;  // keep reporting while the diagnostics run
  dp.passive = 0;            // active: the local name usually arrives in the scan response
  dp.itvl = 0x0060;          // 60 ms
  dp.window = 0x0056;        // ~54 ms
  s_seenYale.clear();
  s_advReports = 0;
  m_scanMatched = false;
  int rc = ble_gap_disc(m_ownAddrType, SCAN_MS, &dp, gapEvent, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
    return false;
  }
  Ev ev{};
  if (!waitFor(EvType::DiscFound, SCAN_MS + 1000, ev)) {
    ble_gap_disc_cancel();
    ESP_LOGE(TAG, "lock not found while scanning for %u s: heard %u advertisements, %u Yale/HomeKit-BLE devices (listed above)",
             (unsigned)(SCAN_MS / 1000), (unsigned)s_advReports, (unsigned)s_seenYale.size());
    return false;
  }
  ble_gap_disc_cancel();
  m_peerAddrType = ev.addrType;
  m_addrKnown = true;
  ESP_LOGI(TAG, "lock found after %lld ms (rssi %d dBm, addr type %u)",
           (esp_timer_get_time() - t0) / 1000, ev.status, m_peerAddrType);

  ble_addr_t peer{};
  peer.type = m_peerAddrType;
  for (int i = 0; i < 6; ++i) peer.val[i] = m_mac[5 - i];
  const ble_gap_conn_params cp = connParams();
  rc = ble_gap_connect(m_ownAddrType, &peer, CONNECT_MS, &cp, gapEvent, this);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_connect failed: %d", rc);
    return false;
  }
  while (true) {
    if (!waitFor(EvType::Connected, CONNECT_MS + 1000, ev)) {
      ESP_LOGE(TAG, "connect timed out");
      ble_gap_conn_cancel();
      return false;
    }
    break;
  }
  ESP_LOGI(TAG, "connected in %lld ms", (esp_timer_get_time() - t0) / 1000);
  return true;
}

bool YaleBleLock::discover() {
  m_svcStart = m_svcEnd = 0;
  m_hWrite = m_hRead = m_hSecWrite = m_hSecRead = 0;
  m_hReadCccd = m_hSecReadCccd = 0;
  m_readProps = m_secReadProps = 0;
  s_chrs.clear();
  Ev ev{};

  int rc = ble_gattc_disc_svc_by_uuid(m_conn, &SVC_UUID.u, onSvc, this);
  if (rc != 0 || !waitFor(EvType::GattDone, GATT_MS, ev) || ev.status != 0 || m_svcStart == 0) {
    ESP_LOGE(TAG, "service 0xFE24 not found (rc=%d status=%d)", rc, ev.status);
    return false;
  }
  rc = ble_gattc_disc_all_chrs(m_conn, m_svcStart, m_svcEnd, onChr, this);
  if (rc != 0 || !waitFor(EvType::GattDone, GATT_MS, ev) || ev.status != 0) {
    ESP_LOGE(TAG, "characteristic discovery failed (rc=%d status=%d)", rc, ev.status);
    return false;
  }
  if (!m_hWrite || !m_hRead || !m_hSecWrite || !m_hSecRead) {
    ESP_LOGE(TAG, "lock is missing expected characteristics");
    return false;
  }
  std::sort(s_chrs.begin(), s_chrs.end());
  for (uint16_t target : {m_hSecRead, m_hRead}) {
    uint16_t end = m_svcEnd;
    for (auto &c : s_chrs) {
      if (c.first > target) { end = c.first - 1; break; }
    }
    m_dscTarget = target;
    rc = ble_gattc_disc_all_dscs(m_conn, target, end, onDsc, this);
    if (rc != 0 || !waitFor(EvType::GattDone, GATT_MS, ev)) {
      ESP_LOGW(TAG, "descriptor discovery failed for handle %u; assuming CCCD at value+1", target);
    }
  }
  const bool secAssumed = !m_hSecReadCccd, readAssumed = !m_hReadCccd;
  if (secAssumed) m_hSecReadCccd = m_hSecRead + 1;
  if (readAssumed) m_hReadCccd = m_hRead + 1;
  ESP_LOGI(TAG, "gatt: svc %u-%u write=%u read=%u(props 0x%02X cccd %u%s) secWrite=%u secRead=%u(props 0x%02X cccd %u%s)",
           m_svcStart, m_svcEnd, m_hWrite, m_hRead, m_readProps, m_hReadCccd, readAssumed ? " assumed" : "",
           m_hSecWrite, m_hSecRead, m_secReadProps, m_hSecReadCccd, secAssumed ? " assumed" : "");
  return true;
}

bool YaleBleLock::writeCccd(uint16_t cccdHandle, uint8_t props, const char *what) {
  // Notify if the characteristic supports it, otherwise indicate (as BlueZ/bleak choose).
  const bool notify = props & BLE_GATT_CHR_PROP_NOTIFY;
  const bool indicate = props & BLE_GATT_CHR_PROP_INDICATE;
  const uint8_t enable[2] = {uint8_t(notify ? 0x01 : (indicate ? 0x02 : 0x01)), 0x00};
  int rc = ble_gattc_write_flat(m_conn, cccdHandle, enable, sizeof(enable), onWrite, this);
  Ev ev{};
  if (rc != 0 || !waitFor(EvType::GattDone, GATT_MS, ev) || ev.status != 0) {
    ESP_LOGE(TAG, "enabling %s on %s (cccd %u, props 0x%02X) failed (rc=%d status=0x%x)",
             enable[0] == 0x02 ? "indications" : "notifications", what, cccdHandle, props, rc, ev.status);
    return false;
  }
  return true;
}

bool YaleBleLock::writeFrame(uint16_t handle, const Frame &frame) {
  int rc = ble_gattc_write_flat(m_conn, handle, frame.data(), FRAME_LEN, onWrite, this);
  Ev ev{};
  if (rc != 0 || !waitFor(EvType::GattDone, GATT_MS, ev) || ev.status != 0) {
    ESP_LOGE(TAG, "write failed (rc=%d status=%d)", rc, ev.status);
    return false;
  }
  return true;
}

void YaleBleLock::cooldown() {
  int64_t since = esp_timer_get_time() - m_lastRxUs;
  if (since < COOLDOWN_US) vTaskDelay(pdMS_TO_TICKS((COOLDOWN_US - since) / 1000 + 1));
}

bool YaleBleLock::secureExchange(uint8_t opcode, const uint8_t *payload8, Frame &response) {
  Frame cmd{};
  cmd[0x00] = opcode;
  std::memcpy(&cmd[0x04], payload8, 8);
  cmd[0x10] = 0x0F;
  cmd[0x11] = m_slot;
  uint32_t cs = securityChecksum(cmd.data());
  cmd[0x0C] = cs & 0xFF; cmd[0x0D] = cs >> 8 & 0xFF; cmd[0x0E] = cs >> 16 & 0xFF; cmd[0x0F] = cs >> 24 & 0xFF;
  ecb(true, cmd.data(), cmd.data());

  // No cooldown inside the handshake: yalexs-ble enables it only after setup.
  if (!writeFrame(m_hSecWrite, cmd)) return false;
  Ev ev{};
  if (!waitFor(EvType::NotifySecure, GATT_MS, ev)) {
    ESP_LOGE(TAG, "no response on secure channel (opcode 0x%02X): wrong key or slot?", opcode);
    return false;
  }
  m_lastRxUs = esp_timer_get_time();
  response = ev.frame;
  ecb(false, response.data(), response.data());
  if (u32le(&response[0x0C]) != securityChecksum(response.data())) {
    ESP_LOGE(TAG, "secure response checksum mismatch (opcode 0x%02X): wrong key?", opcode);
    return false;
  }
  return true;
}

bool YaleBleLock::handshake() {
  uint8_t rnd[16];
  esp_fill_random(rnd, sizeof(rnd));
  ecbSetKey(m_offlineKey.data());

  Frame resp{};
  if (!secureExchange(0x01, rnd, resp) || resp[0] != 0x02) {
    ESP_LOGE(TAG, "key exchange rejected: offline key or slot is incorrect");
    std::memset(rnd, 0, sizeof(rnd));
    return false;
  }
  uint8_t session[16];
  std::memcpy(session, rnd, 8);
  std::memcpy(session + 8, &resp[0x04], 8);
  ecbSetKey(session);

  if (!secureExchange(0x03, rnd + 8, resp) || resp[0] != 0x04) {
    ESP_LOGE(TAG, "session initialisation rejected");
    std::memset(rnd, 0, sizeof(rnd));
    std::memset(session, 0, sizeof(session));
    return false;
  }
  cbcSetKey(session);
  std::memset(rnd, 0, sizeof(rnd));
  std::memset(session, 0, sizeof(session));
  ESP_LOGI(TAG, "secure session established");
  return true;
}

bool YaleBleLock::sendCommand(uint8_t opcode, uint8_t subtype, uint8_t expectFlag, uint8_t expectOpcode,
                              int expectSubtype, uint32_t timeoutMs, Frame &response) {
  Frame cmd{};
  cmd[0x00] = 0xEE;
  cmd[0x01] = opcode;
  cmd[0x04] = subtype;
  cmd[0x10] = 0x02;
  cmd[0x03] = simpleChecksum(cmd.data());
  cbcEncrypt(cmd.data());

  cooldown();
  if (!writeFrame(m_hWrite, cmd)) {
    // The encrypt chain has advanced past a frame the lock never saw.
    disconnect();
    return false;
  }
  const int64_t deadline = esp_timer_get_time() + int64_t(timeoutMs) * 1000;
  while (true) {
    int64_t remainMs = (deadline - esp_timer_get_time()) / 1000;
    Ev ev{};
    if (remainMs <= 0 || !waitFor(EvType::NotifyCmd, uint32_t(remainMs), ev)) {
      ESP_LOGE(TAG, "timed out waiting for response to opcode 0x%02X", opcode);
      return false;
    }
    Frame f = ev.frame;
    cbcDecrypt(f.data());
    m_lastRxUs = esp_timer_get_time();
    if (simpleChecksum(f.data()) != 0 || (f[0] != 0xAA && f[0] != 0xBB)) {
      ESP_LOGW(TAG, "invalid frame in response to opcode 0x%02X", opcode);
      continue;
    }
    handleCommandFrame(f);
    const bool flagOk = expectFlag == 0 ? (f[0] == 0xAA || f[0] == 0xBB) : f[0] == expectFlag;
    if (flagOk && f[1] == expectOpcode && (expectSubtype < 0 || f[4] == expectSubtype)) {
      response = f;
      return true;
    }
  }
}

void YaleBleLock::handleCommandFrame(const Frame &f) {
  // State is not tracked (see class comment); frames are only logged.
  if (f[0] == 0xBB && f[1] == OP_GETSTATUS && (f[4] == STATUS_LOCK_ONLY || f[4] == STATUS_DOOR_AND_LOCK)) {
    ESP_LOGI(TAG, "lock status 0x%02X", f[8]);
  } else if (f[0] == 0xBB && (f[1] == OP_UNLOCK || f[1] == OP_LOCK)) {
    if (f[0x0F] == 0x00) ESP_LOGI(TAG, "lock reports %s done", f[1] == OP_UNLOCK ? "unlock" : "lock");
    else ESP_LOGE(TAG, "lock reports %s failed: error 0x%02X", f[1] == OP_UNLOCK ? "unlock" : "lock", f[0x0F]);
  } else {
    ESP_LOGD(TAG, "frame %02X %02X sub %02X result %02X", f[0], f[1], f[4], f[0x0F]);
  }
}

void YaleBleLock::disconnect() {
  m_sessionReady = false;
  if (m_conn == 0xFFFF) return;
  ble_gap_terminate(m_conn, BLE_ERR_REM_USER_CONN_TERM);
  Ev ev{};
  if (!waitFor(EvType::Disconnected, 3000, ev)) {
    ESP_LOGW(TAG, "no disconnect confirmation");
    m_conn = 0xFFFF;
  }
  ESP_LOGI(TAG, "session closed");
}

// ------------------------------------------------------------------ cache

void YaleBleLock::loadCache() {
  nvs_handle_t h;
  if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
  GattCache c{};
  size_t len = sizeof(c);
  const bool ok = nvs_get_blob(h, NVS_KEY, &c, &len) == ESP_OK && len == sizeof(c) &&
                  c.version == CACHE_VERSION && std::memcmp(c.mac, m_mac.data(), 6) == 0;
  nvs_close(h);
  if (!ok) return;
  m_peerAddrType = c.addrType;
  m_hWrite = c.hWrite; m_hRead = c.hRead; m_hReadCccd = c.hReadCccd;
  m_hSecWrite = c.hSecWrite; m_hSecRead = c.hSecRead; m_hSecReadCccd = c.hSecReadCccd;
  m_readProps = c.readProps; m_secReadProps = c.secReadProps;
  m_addrKnown = m_gattCached = true;
  ESP_LOGI(TAG, "using cached lock address type and GATT handles");
}

void YaleBleLock::saveCache() {
  GattCache c{};
  c.version = CACHE_VERSION;
  std::memcpy(c.mac, m_mac.data(), 6);
  c.addrType = m_peerAddrType;
  c.hWrite = m_hWrite; c.hRead = m_hRead; c.hReadCccd = m_hReadCccd;
  c.hSecWrite = m_hSecWrite; c.hSecRead = m_hSecRead; c.hSecReadCccd = m_hSecReadCccd;
  c.readProps = m_readProps; c.secReadProps = m_secReadProps;
  nvs_handle_t h;
  if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
  if (nvs_set_blob(h, NVS_KEY, &c, sizeof(c)) == ESP_OK) nvs_commit(h);
  nvs_close(h);
}

void YaleBleLock::clearCache() {
  m_addrKnown = m_gattCached = false;
  nvs_handle_t h;
  if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
  nvs_erase_key(h, NVS_KEY);
  nvs_commit(h);
  nvs_close(h);
}

// ------------------------------------------------------------------ crypto

void YaleBleLock::ecbSetKey(const uint8_t *key16) {
  mbedtls_aes_setkey_enc(&m_ecbEnc, key16, 128);
  mbedtls_aes_setkey_dec(&m_ecbDec, key16, 128);
}

void YaleBleLock::ecb(bool encrypt, const uint8_t *in16, uint8_t *out16) {
  uint8_t tmp[16];
  mbedtls_aes_crypt_ecb(encrypt ? &m_ecbEnc : &m_ecbDec, encrypt ? MBEDTLS_AES_ENCRYPT : MBEDTLS_AES_DECRYPT,
                        in16, tmp);
  std::memcpy(out16, tmp, 16);
}

void YaleBleLock::cbcSetKey(const uint8_t *key16) {
  mbedtls_aes_setkey_enc(&m_cbcEnc, key16, 128);
  mbedtls_aes_setkey_dec(&m_cbcDec, key16, 128);
  m_ivEnc.fill(0);
  m_ivDec.fill(0);
}

void YaleBleLock::cbcEncrypt(uint8_t *block16) {
  uint8_t x[16];
  for (int i = 0; i < 16; ++i) x[i] = block16[i] ^ m_ivEnc[i];
  mbedtls_aes_crypt_ecb(&m_cbcEnc, MBEDTLS_AES_ENCRYPT, x, block16);
  std::memcpy(m_ivEnc.data(), block16, 16);
}

void YaleBleLock::cbcDecrypt(uint8_t *block16) {
  uint8_t c[16], p[16];
  std::memcpy(c, block16, 16);
  mbedtls_aes_crypt_ecb(&m_cbcDec, MBEDTLS_AES_DECRYPT, c, p);
  for (int i = 0; i < 16; ++i) block16[i] = p[i] ^ m_ivDec[i];
  std::memcpy(m_ivDec.data(), c, 16);
}
