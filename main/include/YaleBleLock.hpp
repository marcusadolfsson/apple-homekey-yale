#pragma once

#include "app_event_loop.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mbedtls/aes.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <string>

// Set while the BLE link to the lock is being used. The C6 has a single 2.4 GHz
// radio, so relay polling over ESP-NOW and a BLE connection attempt starve each
// other: connecting took 9.5 s alongside 10 polls/s, against 1.9 s on a quiet
// radio. RemoteNfcReader pauses polling while this is set.
extern std::atomic<bool> g_bleRadioBusy;

namespace espConfig { struct misc_config_t; }
struct ble_gap_event;
struct ble_gatt_error;
struct ble_gatt_svc;
struct ble_gatt_chr;
struct ble_gatt_dsc;
struct ble_gatt_attr;

/**
 * @brief Drives a Yale/August lock (e.g. Yale nexTouch with a Yale Access BLE
 * module) directly over BLE using its offline key, mirroring the protocol in
 * bdraco/yalexs-ble.
 *
 * Protocol summary:
 *  - Service 0xFE24. Frames are always 18 bytes: a 16-byte AES block plus two
 *    plain trailing bytes.
 *  - Secure channel (chars ...613 write / ...614 notify), AES-128-ECB. The
 *    offline key protects a key exchange (opcode 0x01) that yields an 8+8 byte
 *    session key, confirmed by opcode 0x03. Frames carry the key slot at 0x11
 *    and a 32-bit checksum at 0x0C.
 *  - Command channel (chars ...611 write / ...612 notify), AES-128-CBC with a
 *    zero IV and the session key. The CBC chain runs continuously across every
 *    frame of the connection in each direction. Frames start 0xEE (command) or
 *    0xAA/0xBB (ack/result) with an 8-bit checksum at 0x03.
 *
 * Behaviour: unlock only, fire and forget. A successful HomeKey tap (with
 * lockAlwaysUnlock) or a Home app "unlock" on the reader sends UNLOCK; "lock" is
 * ignored because the mortise relocks itself. The reader does not track the lock's
 * state (Home Assistant and the lock's own HomeKit module do), which also suits a
 * battery reader that should talk to the lock as little as possible.
 *
 * The connection is opened on demand, held about 5 s after the command (as
 * yalexs-ble does), then closed so the Yale app, Apple Home and Home Assistant can
 * reach the lock. The lock's address type and GATT handles are cached in NVS so a
 * cold start (or a wake from deep sleep) connects directly without scan or discovery.
 *
 * The offline key and session key are never logged.
 */
class YaleBleLock {
public:
  explicit YaleBleLock(const espConfig::misc_config_t &misc);
  ~YaleBleLock();
  YaleBleLock(const YaleBleLock &) = delete;
  YaleBleLock &operator=(const YaleBleLock &) = delete;

  /** Parse config and start the worker. No-op (logged) if disabled or invalid. */
  void begin();

  enum class Cmd : uint8_t { Status, Unlock, Lock, Prepare };
  void request(Cmd cmd);

private:
  static constexpr const char *TAG = "YaleBle";
  static constexpr size_t FRAME_LEN = 18;
  using Frame = std::array<uint8_t, FRAME_LEN>;

  // Messages from the NimBLE host task to the worker.
  enum class EvType : uint8_t { Synced, DiscFound, DiscSeen, DiscDone, Connected, ConnectFailed, Disconnected,
                                GattDone, NotifySecure, NotifyCmd, NotifyDropped, Command };
  struct Ev {
    EvType type;
    int status = 0;  // also: rssi for DiscSeen, length for NotifyDropped
    uint8_t addrType = 0;
    Cmd cmd = Cmd::Status;
    Frame frame{};
    std::array<uint8_t, 6> addr{};  // DiscSeen, MSB first
    char name[16] = {0};            // DiscSeen
  };

  // Worker
  static void taskEntry(void *arg);
  void run();
  bool ensureConnected();
  bool scanAndConnect();
  bool connectDirect(uint32_t timeoutMs);
  bool discover();
  bool writeCccd(uint16_t cccdHandle, uint8_t props, const char *what);
  bool handshake();
  bool secureExchange(uint8_t opcode, const uint8_t *payload8, Frame &response);
  bool sendCommand(uint8_t opcode, uint8_t subtype, uint8_t expectFlag, uint8_t expectOpcode,
                   int expectSubtype, uint32_t timeoutMs, Frame &response);
  bool writeFrame(uint16_t handle, const Frame &frame);
  void handleCommandFrame(const Frame &plain);
  void performCommand(Cmd cmd);
  void disconnect();
  void loadCache();
  // Consecutive failed direct (cached-address) connects. With the address known,
  // a failed direct connect almost always means "out of range", and an 8 s scan
  // then just holds the shared radio (dropping taps) to learn the same thing.
  // Scan only when the address is unknown or after this many direct failures,
  // in case the lock's address really did change.
  int m_directFailures = 0;
  static constexpr int DIRECT_FAILURES_BEFORE_SCAN = 3;
  void saveCache();
  void clearCache();
  bool waitFor(EvType type, uint32_t timeoutMs, Ev &out);
  void cooldown();

  // Crypto helpers
  void ecbSetKey(const uint8_t *key16);
  void ecb(bool encrypt, const uint8_t *in16, uint8_t *out16);
  void cbcSetKey(const uint8_t *key16);
  void cbcEncrypt(uint8_t *block16);
  void cbcDecrypt(uint8_t *block16);

  // NimBLE callbacks (host task)
  static int gapEvent(ble_gap_event *event, void *arg);
  static int onSvc(uint16_t conn, const ble_gatt_error *error, const ble_gatt_svc *svc, void *arg);
  static int onChr(uint16_t conn, const ble_gatt_error *error, const ble_gatt_chr *chr, void *arg);
  static int onDsc(uint16_t conn, const ble_gatt_error *error, uint16_t chr_val_handle,
                   const ble_gatt_dsc *dsc, void *arg);
  static int onWrite(uint16_t conn, const ble_gatt_error *error, ble_gatt_attr *attr, void *arg);
  static void onSync();
  static void onReset(int reason);
  static void hostTask(void *param);
  void post(const Ev &ev);

  static YaleBleLock *s_instance;

  bool m_enabled = false;
  std::array<uint8_t, 6> m_mac{};  // canonical order, as printed (MSB first)
  std::array<uint8_t, 16> m_offlineKey{};
  uint8_t m_slot = 0;
  bool m_alwaysUnlock = false;

  QueueHandle_t m_events = nullptr;
  QueueHandle_t m_commands = nullptr;
  TaskHandle_t m_task = nullptr;

  // Connection state (worker-owned unless noted)
  uint8_t m_ownAddrType = 0;
  uint8_t m_peerAddrType = 0;
  uint16_t m_conn = 0xFFFF;  // written by host task on connect/disconnect
  uint16_t m_connItvl = 0;   // negotiated interval, units of 1.25 ms
  int64_t m_lastDisconnectUs = 0;
  uint16_t m_svcStart = 0, m_svcEnd = 0;
  uint16_t m_hWrite = 0, m_hRead = 0, m_hSecWrite = 0, m_hSecRead = 0;
  uint16_t m_hReadCccd = 0, m_hSecReadCccd = 0;
  uint8_t m_readProps = 0, m_secReadProps = 0;  // GATT characteristic properties
  uint16_t m_dscTarget = 0;  // characteristic whose CCCD is being searched
  bool m_sessionReady = false;
  bool m_scanMatched = false;  // set by host task once the lock is heard during a scan
  bool m_addrKnown = false;    // lock address type learned from a scan; lets us connect directly
  bool m_gattCached = false;   // handles from a previous discovery; the lock's GATT table is static
  int64_t m_lastRxUs = 0;

  mbedtls_aes_context m_ecbEnc{}, m_ecbDec{};
  mbedtls_aes_context m_cbcEnc{}, m_cbcDec{};
  std::array<uint8_t, 16> m_ivEnc{}, m_ivDec{};

  AppEventLoop::SubscriptionHandle m_nfcSub;
  AppEventLoop::SubscriptionHandle m_tagSub;
  AppEventLoop::SubscriptionHandle m_targetSub;
};
