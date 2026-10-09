# Apple HomeKey → Yale nexTouch, on ESP32-C6

Tap an iPhone (Apple Home Key, Express Mode) on a reader; the reader authenticates
the key **offline** and unlocks a **Yale nexTouch** mortise lock over **Bluetooth**,
using the lock's `yalexs-ble` offline key. No cloud, no Home Assistant and no Yale
app in the unlock path.

This is a fork of [rednblkx/HomeKey-ESP32](https://github.com/rednblkx/HomeKey-ESP32)
(HomeKit/HomeKey side) plus work added here:

| Added | Where |
|---|---|
| PN532 over **I2C** (upstream is SPI-only) | `main/Pn532I2cTransport.*`, reader type 3 |
| **Yale/August BLE lock driver** (a C++ port of `yalexs-ble`) | `main/YaleBleLock.*` |
| **Relay reader**: NFC lives on a second board, over ESP-NOW | `main/RemoteNfcReader.*`, reader type 4 |
| **Relay doorbell** firmware for that second board | `relay-doorbell/` |

## The two architectures

**A. Single device.** One ESP32-C6 runs HomeKit, the NFC reader and the Bluetooth
link to the lock. Simple; the reader holds the HomeKey credentials and the lock's
offline key. Validated end to end.

**B. Split (base + doorbell)** — the one to build for battery use:

```
   corridor side                          inside the unit
┌────────────────────────┐            ┌──────────────────────────┐
│ doorbell               │  ESP-NOW   │ base (mains)             │
│  PN532 / ST25R3916     │◄──────────►│  HomeKit (WiFi)          │
│  no keys, no WiFi      │  APDUs     │  HomeKey keys + crypto   │
│  no Bluetooth          │            │  BLE ──► Yale nexTouch   │
└────────────────────────┘            └──────────────────────────┘
```

Why: key issuance and revocation happen over HomeKit, so the device holding the
keys must always be reachable. Putting it on mains removes the battery/reachability
tradeoff, keeps credentials off the corridor-side device, and makes revocation
immediate. Measured ~0.37 Ah/yr for the doorbell versus ~1.5 Ah/yr for a sleeping
single device with hourly check-ins.

## Measurements (ESP32-C6, 2026-09-15)

| | Direct reader | Relayed over ESP-NOW |
|---|---|---|
| HomeKey transaction | **256 ms** | **338 ms** |
| Endpoint authentication | 174 ms | 187 ms |
| Per-APDU round trip | ~10–20 ms | **42–158 ms (avg 77)** |

**Apple's HomeKey tolerates the relay** — the iPhone completed transactions with
80–160 ms per exchange. That was the risk that could have sunk architecture B.

**Tap to door open: ~3.2 s** next to the lock, broken down:

| Stage | Time |
|---|---|
| HomeKey exchange (relayed) | 307 ms |
| BLE connect (cached address, no scan or discovery) | 722 ms |
| Secure handshake (30 ms connection interval) | 367 ms |
| **The lock's own unlock and reply** | **1.77 s** |

Over half of what remains is the lock itself. Idle current target for a battery
doorbell is ~20–30 µA (ST25R3916 wake-up mode ~3 µA + C6 deep sleep 7 µA + regulator).

**Two Express taps 5 s apart (2026-09-16, at the door):** first tap authenticated
in 185 ms, `connected directly in 342 ms`, **unlock succeeded in 2798 ms**; second
tap authenticated in 189 ms and rode the lingering session — **unlock succeeded in
437 ms**, no connect at all. Cold ≈ 3 s, warm ≈ 0.6 s, no drops.

**Range dominates everything.** The same firmware, with Home Assistant out of the
picture entirely: next to the lock, connect 722 ms and unlock 2.9 s; well away from
it, the cached-address connect fails outright, a scan takes 2.6 s and the unlock
takes 9.9 s. Weak signal, not contention, explained our worst measurements — keep
the base within good range of the lock and re-measure before blaming anything else.

### Tried and reverted

- **Disabling WiFi power save** on the base to cut relay latency: saved ~10 ms on the
  link but made BLE connects 3–4× slower (1.9 s → 4.8–8.0 s). One radio, shared.
- **Connecting to the lock speculatively when a card is detected**, to overlap the
  ~2 s connect with authentication: corrupted the card exchange itself
  ("Auth0 response invalid", a 112-byte APDU answered with 0 bytes) and pushed the
  connect to 15.8 s. **Do not run BLE and the NFC relay at the same time.**
  `g_bleRadioBusy` now pauses all relay traffic while the lock link is up.
- A failed connect used to clear the GATT cache, making every retry pay for
  rediscovery. A connection failure says nothing about the GATT layout.

### Efficiency audit (2026-09-16)

Done in one pass after the relay stabilised; the reasoning, so it is not undone:

- **Cooldown timed from command-channel traffic only** (`m_lastCmdRxUs`). The
  250 ms write spacing protects the lock's radio between *command* writes;
  timing it from the last handshake frame made the first unlock of every
  session wait ~150–220 ms for nothing.
- **Stale announcements drained in one `pollForTag()` call**, and no between-poll
  delay for the relay reader (the queue wait already paces it): each stale
  entry used to cost a full 100 ms poll cycle. Tag-removal presence checks run
  every 250 ms on the relay (each is a radio round trip), 60 ms locally.
- **Hot-path logs to debug**: per-APDU round trip, relay summary, ECP hex on
  push, "doorbell asked for ECP", the doorbell's poll-cycle stats. Keep them at
  debug — they were the evidence for every fix in this file, but 8–12 INFO lines
  per tap ran during the timing-critical transaction and once starved the WebSocket.
- **BLE scan**: `filter_duplicates = 1`; the per-device Yale report is only
  parsed and posted at debug level (it ran on the NimBLE host task for every
  advertisement during every scan).
- **PN532 I2C transport**: probe 0x24 (three tries — a cold PN532 does not ACK
  its very first transaction) and sweep the bus only if that fails; the
  unconditional 112-probe sweep cost up to 2.2 s per reader init. Bus-error
  logs throttled to one per 5 s per site; after 20 consecutive bus errors the
  reader reports disconnected so the 5 s re-init actually runs (the ready flag
  never cleared before, and a wedged bus logged an error 15×/s forever).
- **Doorbell**: legacy base-driven `PollReq` path removed (nothing sends it);
  link key printed only when generated or with the button held at power-on;
  battery ADC burst taken on the heartbeat and cached for tap/button frames
  (it sat between seeing the card and announcing it); reassembly buffer moved
  rather than copied; no 260-byte zero-init per frame in the receive callback.
- Housekeeping: `Response` objects drained before their queues are deleted;
  dead counters and the unsubscribed `NFC_TAG_DETECTED` publish removed; relay
  metrics only emitted for reader type 4; HA discovery JSON unformatted.

**Rejected on purpose:** relaxing the connection interval during the 5 s linger
(would cut radio contention but slow the measured 437 ms warm tap); the PN532
transport's fixed 301-byte reads (~7 ms per response — real, but that reader is
being replaced and the fix depends on PN532 re-serve semantics to verify on
hardware); interrupt-driven button and RTC-retained link state (deep-sleep work).

## Yale BLE protocol (as implemented in `YaleBleLock`)

Service `0xFE24`; every frame is 18 bytes (a 16-byte AES block plus two plain bytes).

- **Secure channel** (`…613` write / `…614` notify), **AES-128-ECB** with the offline
  key: opcode `0x01` exchanges 8 random bytes each way; the session key is
  `ours[0:8] || theirs[0:8]`; opcode `0x03` confirms. Frames carry the key slot at
  `0x11` and a 32-bit checksum at `0x0C`.
- **Command channel** (`…611` write / `…612` notify), **AES-128-CBC**, zero IV, and
  **the chain continues across every frame of the connection** in each direction.
  Frames start `0xEE`; replies `0xAA` (ack) or `0xBB` (result), 8-bit checksum at `0x03`.
  Opcodes: GETSTATUS `0x02`, UNLOCK `0x0A`, LOCK `0x0B`; result byte at `0x0F`.

Hard-won details:

- **Both notify characteristics are INDICATE-only** (properties `0x20`). Writing
  `0x0001` to the CCCD is rejected with ATT error `0x80`; write `0x0002`.
- **Never log from a NimBLE host callback** — it stalled the host task mid-scan.
  Post to a worker task instead.
- Lock status: `0x03` unlocked, `0x05` locked, `0x07` jammed, `0x0C` secure mode.
- The lock allows very few concurrent BLE connections and drops idle ones after
  ~30 s. We disconnect ~5 s after a command, matching `yalexs-ble`'s default, so
  Home Assistant and the Yale app can still reach it. Verified: zero HA connection
  failures while both were in use.
- Address type and GATT handles are cached in NVS (namespace `yaleble`), so a cold
  start connects directly with no scan or discovery.

**Behaviour (2026-10-08): the base tracks the lock's real state** — before this
it was unlock-only, fire and forget. Commands still return on the lock's **ack
(`0xAA`, tens of ms)**, not its **result (`0xBB`, ~1.8 s later when the motor
stops)**, so a tap never holds the radio for the mechanical cycle; the result
arrives during the 5 s linger. After our own disconnect the lock is slow to
advertise again; that is handled with a longer direct-connect timeout
(`CONNECT_MS`), never a scan.

- **State sources:** the lock's own answers to our commands (`0xAA` = moving,
  `0xBB` = done; a non-zero result = jammed, as yalexs-ble), and **status reads**
  (GETSTATUS lock `0x02`, door `0x2E`, battery `0x0F` every 6 h; byte 8, battery
  bytes 8–9 in mV) at boot +5 s, every **5 min** (`POLL_US`), and 10 s after a
  cloud report that changed the state (`CONFIRM_DELAY_US`). A read takes ~1.8–3.4 s.
- **Cloud state:** Home Assistant forwards the Yale cloud's state (the cloud sees
  keypad/thumb-turn/door changes within 1–2 s) to `<clientId>/yale/cloud`:
  `locked` / `unlocked` / `jammed` / `door_open` / `door_closed`. Applied at once,
  then confirmed by a read; a report repeating what we know (our own unlock echoed
  back) costs no read. The base's read is the authority.
- **Reads never cost a tap:** a read waits while card traffic is under way
  (`RELAY_QUIET_US`) and, if a tap arrives during one, `RemoteNfcReader` sets
  `g_bleAbort` and drops that announcement; the read stops within ~50 ms
  (`waitFor` slices) and keeps the session, and the doorbell's next announcement
  (250 ms later) goes through and unlocks on the open session. `request(Unlock|Lock)`
  pre-empts a read the same way. An aborted connect does not count as an
  out-of-range failure.
- **Reported to:** LockManager via `LOCK_OVERRIDE_STATE` (the HomeKit tile and the
  MQTT lock entity show the real state; moving states set only the target, so the
  tile says "Unlocking…"); the momentary snap-back to "locked" is off while Yale
  BLE is enabled. `YALE_STATUS` → MQTT `<clientId>/yale/status` (JSON lock, door,
  battery_mv, source; retained) and `<clientId>/yale/read` (each read, not retained;
  HA sensor with `force_update` + `expire_after` 900 s, so it goes unavailable if
  the base stops reading). Discovery adds a door `binary_sensor`, a lock battery
  voltage sensor and that read sensor.
- **Commands:** HomeKit / MQTT "lock" now locks (`0x0B`); a failed command puts
  the tile back to the last known state and reads again 15 s later.

**Why polling and not push (measured 2026-10-08):** this lock's advertisement is
`020106 030224FE 04FFD10101` + a scan response with the 18-byte Yale ID and name
`M50067E`. The one-byte Yale flag stayed `01` through manual lock/unlock, and
there is no HAP advertisement (Apple `0x004C` type `06`) at all while the lock's
HomeKit side is down — Apple Home "No Response" from ~14:00 that day, "Accessory
not found" on re-add, even after a battery pull. yalexs-ble's push depends
entirely on the HAP GSN (or a Yale flag change), so HA's `yalexs_ble` entities
froze too. `CONFIG_YALE_BLE_LISTEN` (off) keeps a 5 %-duty listener for the GSN
should HomeKit come back; with it on, 7 taps showed no Auth0 errors but the
sample was too small to judge lock-connect latency (median 1.71 s vs 1.33 s
baseline, both with HA also connecting to the lock).

## Relay protocol (`main/RelayProtocol.hpp`)

ESP-NOW, 10-byte header, fragmented above 240 bytes, one request in flight.
**Unicast is encrypted and both ends are pinned by MAC**; pairing broadcasts are
in the clear because ESP-NOW cannot encrypt broadcast.

- **Key sharing.** The doorbell generates a random 16-byte key on first boot,
  keeps it in NVS (`relay/key`) and prints it **once over USB**. The user pastes it
  into the base's web page (Hardware → Link key; masked on read-back, never logged).
  Both sides derive PMK = SHA-256(key)[0:16], LMK = [16:32]. The key never crosses
  the air. The doorbell pins the base's MAC on first contact (`relay/base`); the
  base pins the doorbell from its config (`relayDoorbellMac`).
- **Add the pinned peer before first contact — on both sides.** ESP-NOW only
  decrypts a unicast from a peer already held with the matching LMK, and the first
  frame from the other box is exactly what would tell you its MAC. Learning the peer
  from that frame is a deadlock (it happened twice, once per direction). A link key
  without a pinned MAC therefore cannot pair; the base warns about it.
- `Ping`/`Pong` — pairing, from **either** side: the doorbell walks the WiFi
  channels until a base answers, a base with no doorbell advertises every 3 s.
  **Pinning must not skip channel discovery**: the base sits on its AP's channel,
  which moved four times in one evening (13 → 6 → 7 → 1). The doorbell caches the
  last good channel (`relay/chan`) and tries it first, and re-scans after 30 s of
  silence.
- `EcpReq`/`EcpSet` — the base hands the doorbell the 18-byte ECP frame plus the
  **listen window (500 ms) and the gap between cycles (20 ms; 5 with "fast
  polling")**, mirroring `NfcManager::pollingTask()`. Measured ECP rate 15.2 Hz.
- **`TagEvent`** — the doorbell announces a card. **The base sends nothing between
  taps**: polling was inverted so a battery doorbell can sleep and so the radio is
  free for BLE. The announcement is **repeated up to 4× at 250 ms** until an APDU
  arrives — it is the one frame a tap cannot afford to lose, and at −67 dBm a
  single frame went missing often enough to cost whole taps. The doorbell holds the
  card until `ReleaseReq` (or a 2 s fallback). **One physical tap = one
  announcement:** after each release the doorbell waits until the field has been
  empty for two polls in a row before it will announce again, so a phone left
  resting on the reader is one tap however long it stays (a re-detected phone is
  just released). Verified with the ST25R3916: three taps, three announcements,
  three unlock attempts. **The base must send that release
  even while BLE holds the radio**: a successful tap sets `g_bleRadioBusy` the
  instant the unlock is requested, so a "skip when busy" guard in `releaseTag()`
  meant no release after any *successful* tap and a 5 s blind window after every
  good one (a tap 2 s after another was invisible). One frame is nothing next to
  a BLE connect; only the wait for the reply is skipped while busy. Every doorbell
  frame carries the reader-ready bit in its flags; the base tracks it continuously.
- `ApduReq/Rsp`, `PresentReq/Rsp`, `ReleaseReq/Rsp` — the transaction itself.
- `HealthRsp` — unsolicited heartbeat every 30 s; liveness is inferred from traffic
  received rather than polled for (`HEARTBEAT_TIMEOUT_MS`). **The base answers it
  with a `Pong`.** Between taps that ack is the only frame the base ever sends, and
  the doorbell re-scans for the base after `BASE_SILENCE_US` (45 s, 1.5 heartbeat
  intervals) without hearing from it — without the ack an idle doorbell "lost" the
  base every 30 s and swept the channels for nothing. **Keep the silence timeout
  longer than the heartbeat interval:** with both at 30 s, the silence check ran
  in the same loop iteration that had just sent a heartbeat, before its ack could
  arrive, and re-scanned every 30 s anyway (18 times in 13 minutes, found
  2026-10-06).
- `ButtonPress` — the doorbell button.

**Tap announcements have their own queue** on the base. Sharing one queue with
request/response replies made the polling wait swallow them.

### Doorbell button and battery

Wiring drawing: `docs/wiring.svg` (generated by `docs/wiring.py`; keep it in
step with the `PIN_*` constants in `relay-doorbell/main/main.cpp`).

**Pins swapped 2026-10-08** to match the production doorbell (XIAO MAC
..AE:A0:98): reader IRQ on **D1**, button on **D2**, and the Click's **SCL on D4, SDA on D5**.
The first bench doorbell (..8C:DC) is wired IRQ→D2, SDA→D4, SCL→D5 and needs
those `PIN_*` constants swapped back.


- **Button on D2** (any momentary switch to GND; internal pull-up, debounced 50 ms).
  D2 is a low-power pin, so in a battery build the same wire wakes the chip from
  deep sleep and a press costs one wake plus one frame.
- **Battery on A0/D0** through a 1:2 divider. Use 1 MΩ + 1 MΩ (~2 µA), not Seeed's
  suggested 200 kΩ pair (~10 µA — half the idle budget). Reported as 0 when absent.
- Voltage is **appended to frames the doorbell already sends** (tap announcements,
  heartbeats, button presses), so nothing transmits solely to report it.
- The base shows both on the dashboard (pairing state, link RSSI, battery) and
  publishes the button to `<id>/doorbell` and the battery to
  `<id>/doorbell/battery`, with Home Assistant discovery for each.

## Hardware

- **ESP32-C6** (XIAO ESP32C6 used here). Deep sleep 7 µA; BLE RX 71 mA,
  802.15.4 RX 74 mA, WiFi RX 78 mA — receive costs are near-identical, so link
  choice matters far less than time spent awake.
- **PN532 V3** over I2C: VCC→**3V3** (its pull-ups follow VCC and the C6 is not 5 V
  tolerant), SDA→D4 (GPIO22), SCL→D5 (GPIO23), DIP switches to I2C. 5 V on VCC was
  tried for a stronger field: no change to Express mode, and the rewiring caused
  reboots. A hung PN532 holds SDA low (`clear bus failed`, `I2C scan: no devices`)
  and **esptool's reset does not power-cycle it** — unplug the board's USB.

### Low-power card detection (ST25R3916 wake-up mode) — stage 1, 2026-10-06

The doorbell no longer polls ~15 times a second with the field on. Its reader
rests in the ST25R3916's wake-up mode (oscillator off, the chip sampling the
antenna every `LPCD_PERIOD_MS` = 100 ms); a phone moves the reading, the chip
raises IRQ on **D1 (GPIO1)**, and only then does the doorbell poll, for
`LPCD_ACTIVE_US` (1.5 s), re-arming once the field is clear. The reference is
re-taken every 60 s. Build switch `DOORBELL_LPCD` (default 1); ST25R3916 only.
The C6 itself is still awake — deep sleep is stage 2. Driver API:
`measureAntenna`, `startWakeUpMode(period, ampDelta, phaseDelta, d_res)`,
`takeWakeUpEvents`, `stopWakeUpMode` (sequence and registers from ST's RFAL,
`rfalWakeUpModeStart`, AN5320). Hard-won on the NFC 4 Click:

1. **The amplitude reading saturates at full drive** — 255, the top of the A/D.
   A phone could not pull it into range: two minutes of taps, zero wake-ups. Wake-up
   mode now runs with a weaker field (TX_DRIVER 0x28, `d_res`), restored to full
   drive for polling. A sweep at first arm picks the strongest drive whose reading
   is ≤ 160 (was 200, see below): `0:255 1:255 2:217 3:139 4:139 5–14:75 15:42` → **d_res 3, ~139**,
   identical on repeat boots.
2. **Take the reference from wake-up mode itself.** RFAL takes it with the
   measure command, which disagreed with the mode's own readings (238–251 vs a
   steady 251, near saturation) so every arm fired on its first sample. Now:
   arm insensitive (delta 15), let the timer take three readings, adopt their
   average, then tighten the delta, without leaving the mode.
3. **Phase reads 0 throughout** on this board, so detection is amplitude-only
   (RFAL's default is too).
   **Ferrite changes this (production Click, 2026-10-08):** with an NFC ferrite
   sheet behind the antenna, full drive reads only ~180, so the 200 cap chose
   full drive — and it false-woke every 1–2 s: each re-arm right after polling
   took a reference the reading crept 3 steps above. Capped at 160 → d_res 2
   (~146): 0 false wake-ups idle or after taps, 3/3 taps found 178–234 ms after
   wake-up.
4. **A phone *raises* the amplitude** here — ~+3 at detection distance, to ~204
   lying on the reader (rest 139). The comparison is symmetric. Delta **3**:
   2 fired on noise, and at-rest readings sit within 1–2 steps.
5. **Short listen window for the ST25R3916** (`ST25R3916_LISTEN_MS` = 25). The
   base's 500 ms is a PN532 figure (a ceiling there); the ST25R3916 driver waits
   it out while the phone is not answering yet, so a wake-up took ~585 ms to find
   the card. With 25 ms it repeats ECP + WUPA every few tens of ms: **111 ms**.

Results: **12 minutes idle with zero false wake-ups**; 3 of 3 taps woke the
reader, card found 111–112 ms after wake-up, authenticated in 93–94 ms. The drive
sweep takes ~6 s, so it runs once per power-up; stage 2 keeps the chosen `d_res`
in RTC memory.

### Deep sleep between events — stage 2, 2026-10-06

The C6 now sleeps (7 µA) while the ST25R3916 sits in wake-up mode. It wakes on
**D1 high** (reader IRQ: a phone), **D2 low** (button) or the **heartbeat timer**,
does that one job and sleeps again. Build switch `DOORBELL_SLEEP` (default 1).

- **What survives sleep** (`RtcState`, `RTC_DATA_ATTR`, POD only): pinned base
  MAC and channel, ECP frame, listen/poll timings, `d_res`, the reader's
  wake-up reference (`St25r3916Reader::WakeUpPersist`), last heartbeat / arm
  times, wake counters. Time is `gettimeofday`, which keeps counting through
  deep sleep (`esp_timer` restarts at 0). A cold boot clears it all.
- **Waking from the reader**: `resumeInWakeUpMode()` attaches to the chip
  *without* resetting it (a reset would lose the event), reads the IRQ, then polls.
  Card announced ~336–339 ms after the C6 boots; auth 87–89 ms; Express animation
  confirmed. The reader is re-armed as soon as the field is clear.
- **Heartbeat** every hour (`HEARTBEAT_SLEEP_US`) with an ECP request; it must be
  acked (any frame from the base) within 300 ms, 3 tries. If not, a short search
  (`findBase(2)`); if that fails too, keep the pinned base and channel and retry
  in 5 min (`HEARTBEAT_RETRY_US`) — never stay awake hunting. Normal heartbeat
  wake: **~290 ms**. Exercised by accident while the base rebooted: 7.9 s, then
  the retry succeeded. The base's silence alarm is 3 h (`HEARTBEAT_TIMEOUT_MS`).
- **Sleep is refused** while: no base or ECP, reader not in wake-up mode, a tag
  is present or the field not yet clear, the button is held, IRQ is high, a
  heartbeat is unacked, or a frame left < 50 ms ago. The reference is re-taken
  hourly (on a wake), not every minute as when awake.
- **Pins in sleep**: IDF holds LP pad pulls; button (D2) pull-up, IRQ (D1) pull-down, ext1 per
  pin level (`esp_sleep_enable_ext1_wakeup_io`). After waking,
  `rtc_gpio_hold_dis` + `rtc_gpio_deinit` on both before normal GPIO use.
- **Working on it over USB**: the default build *does not sleep while a USB host
  is attached* (`usb_serial_jtag_is_connected()`), so a doorbell on a computer
  behaves as before. Bench-test sleep with the separate build
  `idf.py -B build-sleeptest -DDOORBELL_SLEEP_ON_USB=1 build`: heartbeat 120 s,
  retry 30 s, 3 s minimum awake per wake so the port can be caught. Every cold
  boot stays awake 20 s (`COLD_BOOT_AWAKE_US`), the window for flashing — wait
  for the port to appear, then flash. The last log line before sleep needs
  `fflush` + `fsync` + 200 ms or it is lost.
- The relay link key is printed only when generated or with the button held at
  power-up — never on every wake.
- **Base restarted on another channel** (its AP moved): the doorbell cannot
  know until something goes unanswered. Heartbeat: after 3 tries it searches,
  then **re-sends the heartbeat** so the battery reaches the base (before
  2026-10-08 the Ping that found it carried no battery and the dashboard said
  "not fitted" for an hour). **Tap**: after 4 unanswered announcements (~1 s,
  the loop waits 50 ms instead of 1 s while one is outstanding) it searches
  once per tap (`researchBase`, up to 30 s, last channel first), then
  deselects the phone and **reads it afresh** (no field-clear wait). Re-announcing
  the old card failed in the 10-08 test — the phone's session had gone stale
  during the search ("Not a HomeKey tag") and only a re-detection 2 s later
  rescued it (tap → unlock ~9 s, incl. a slow 4.4 s lock connect). **Button**: the base never
  replies to a press, so delivery is checked with ESP-NOW's own send report
  (`sendConfirmed`); undelivered → search → resend. Measured on 10-08: a base
  restart at 15:31 was only noticed at the 16:23 heartbeat (52 min) — taps in
  between would have failed.
- **No base at power-up** (base down or out of range): on battery the search is
  bounded — 2 sweeps of 13 channels (~7 s), stretched to the 20 s cold-boot
  window — then `sleepWithoutBase()`, waking on the back-off timer (1, 2, 5,
  10, then every 15 min; `BASE_SEARCH_BACKOFF_S`; 30 s in the bench build), the
  button, **or a tap**: the reader stays in wake-up mode, a tap wakes the
  doorbell, it searches (last known channel first, usually one ping; for up to
  30 s after a tap or button press, `BASE_SEARCH_ATTENDED_US`, since the base may
  still be starting), and with
  the reader *resumed* rather than reset the wake-up event survives, so the
  phone still on the reader is served on that same wake once the ECP frame
  arrives. (Until 2026-10-08 the reader was powered down and only the timer —
  up to 60 min — or the button woke it: after a base outage taps did nothing
  for up to an hour.) Without a working reader it is still powered down. A wake with
  `RtcState::baseLost` searches again like a cold boot. Tested by holding the
  base in its bootloader: 20 s search, sleep, 7 s retries, and found on the first
  retry after the base came back. On a computer's USB it still searches without
  limit, as before.

**XIAO ESP32C6 RF switch — must be driven.** The board routes its antenna
through an RF switch: **GPIO3 low enables it, GPIO14 low selects the built-in
ceramic antenna** (high: U.FL). Seeed's Arduino board package sets them; plain
IDF and the generic `esp32c6` Arduino variant do not, so both boards ran with
them floating. It worked by luck until deep sleep (GPIO3 is an LP pin), after
which the base heard the doorbell at −94 dBm instead of −51…−67 and the doorbell
could not hear the base at all. Fixed: the doorbell's `selectBuiltInAntenna()`
runs first in `app_main`; the base sets them in `setup()` under
`CONFIG_XIAO_ESP32C6_RF_SWITCH` (default y; `..._EXTERNAL_ANTENNA` for U.FL),
and GPIO3/14 are restricted pins. After the fix: base Wi-Fi −75 → **−51 dBm**,
relay link → **−64 dBm**. If RSSI ever drops ~20–30 dB for no reason, check this
first.

### Express mode (the tap-without-Wallet animation) — hard-won

`Pn532Reader::healthCheck()` writes **CIU_BitFraming (`0x633D`) = 0 before every
poll**. It reads like a liveness probe; it is not. Anticollision sends REQA/WUPA as
7-bit short frames and can leave `TxLastBits = 7`, so the next `InCommunicateThru`
sends the ECP frame's last byte as 7 bits, the iPhone fails the CRC and ignores it.
Symptom: taps only work with the Home Key opened in Wallet; the frame looks perfect
from the host (header, reader GID, CRC all verified), encryption/supply/rate are
innocent. The doorbell now does that write in `pollOnce()` before the ECP transmit.
It broke when polling was inverted (the base stopped sending `HealthReq`, which had
been doing the write for it) — not when encryption landed, though both were the
same evening.
- **ST25R3916** is the intended production reader: ~3 µA wake-up (low-power card
  detection) versus a PN532 that costs milliamps just to look for a card.
- **The doorbell carries both drivers** behind the base's `INfcReader`
  (`relay-doorbell/main/DoorbellPn532Reader.hpp`, `St25r3916Reader.*` copied
  verbatim from the base). Selection: compile default `DOORBELL_DEFAULT_READER`
  (0 = PN532, **1 = ST25R3916, the default since 2026-10-06**), overridable from
  NVS `relay/reader` (u8). On boot the doorbell probes the reader's address on
  its own short-lived bus and logs who answers (`I2C: reader answers at 0x50`, or
  the full bus contents if not) before the driver claims the pins. Both sit
  on SDA→D4 (GPIO22), SCL→D5 (GPIO23); the ST25R3916 driver was written against
  the M5Stack Unit NFC at I2C 0x50 and polls the chip's IRQ registers over I2C,
  so it works with **no IRQ wire** — that only matters for deep-sleep wake.
  - *M5Stack Unit NFC (bench):* Grove red→XIAO **5V** (USB only), black→GND,
    white (SDA)→D4, yellow (SCL)→D5. No IRQ on the Grove connector.
  - *MikroE NFC 4 Click (production, 57.15 × 25.4 mm):* 3.3V, GND, SDA→D5,
    SCL→D4, **IRQ→D1 (GPIO1)** — a low-power pin, so it can wake the
    C6 from deep sleep. Flip the `COMM SEL` SMD jumpers to I2C (ships in SPI).
    Rejected: ELECHOUSE board (40.2 mm wide, cavity is 36), NFC 5 Click
    (ST25R3918, a cut-down 3916 at the same price).
- Reader type 0 = PN532 SPI (upstream defaults SS=18/MOSI=19/MISO=20/SCK=21),
  1 = PN7160, 2 = ST25R3916, **3 = PN532 I2C**, **4 = Relay (ESP-NOW doorbell)**.

## Build and flash

```bash
. ~/esp/esp-idf/export.sh          # ESP-IDF v5.5.5
idf.py set-target esp32c6 && idf.py build          # base firmware
cd relay-doorbell && idf.py build                  # doorbell firmware
```

The app outgrew the two-slot OTA layout (2.2 MB), so `single_app.csv` gives it one
2.875 MB app partition and **1 MB for the web UI at `0x300000`**: **OTA is
disabled; flash over USB.** To keep pairing, keys and settings, flash the parts
individually and never the merged image:

```bash
esptool --chip esp32c6 -p PORT write_flash \
  0x0 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x20000 build/HomeKey-ESP32.bin \
  0x300000 build/spiffs.bin        # NVS at 0x9000 is untouched
```

**Do not shrink the UI partition.** The UI is ~120 KB; with a 128 KB partition
LittleFS silently truncated every file (`components.js` served 9,956 of 45,799
bytes) and the page *still rendered* from a browser cache — new fields simply never
appeared. If a UI change "doesn't show up", compare served sizes against
`data/dist/assets/*.gz` before debugging the Svelte.

**Back up NVS (`0x9000`, length `0x10000`) before repurposing a board** — reflashing
a board with different firmware erases the HomeKit pairing, HomeKey keys, WiFi
credentials and the lock's offline key.

## Configuration

All through the device web page (or its JSON API). **The Yale offline key is entered
there, is masked on read-back, and is never logged or committed.** It lives in NVS,
which is *not encrypted* — enable flash encryption before deploying.

- Hardware tab: reader type (and I2C pins for types 2/3).
- HomeKey section: "Always Unlock" (required for tap-to-unlock), Yale BLE toggle,
  lock MAC, key slot, offline key.
- The lock's MAC, slot and offline key come from Home Assistant's `yalexs_ble`
  integration; they are deliberately not recorded in this repository.

## Known issues / next steps

1. **Lock state is tracked as of 2026-10-08** (see the Yale section): reads +
   the Yale cloud state. Still to do: point HA's automations at the base's MQTT lock and
   disable `yalexs_ble`, so the base is the lock's only BLE client; and the HA
   automation that forwards the Yale cloud state to `<clientId>/yale/cloud`.
2. **The base follows its AP's channel** and the AP roams. A sleeping doorbell
   finds out when a tap or button press goes unanswered and searches then
   (`researchBase`, see the deep-sleep section), so a tap after a channel change
   costs a few seconds rather than failing.
   (The re-scan itself was dead code until 2026-09-16: it sat below the
   `pollOnce(); continue;` in the main loop, so a polling doorbell never reached
   it and stayed parked on a dead channel until power-cycled. Anything that must
   run every loop goes *above* that `continue`.)
3. **Taps are dropped while the lock link is busy — by design, and that is
   correct.** The link is busy because an unlock is already on its way, so the
   door opens anyway. Running the card exchange *concurrently* with a connect
   corrupted it, so the two never overlap.
   **Do not re-add tap pre-emption.** From 2026-09-16 to 10-06 a tap cancelled an
   in-flight connect (`abortLinkAttempt`) so it could authenticate. It never
   helped anyone through the door — in range it threw away connect progress to
   redo the same unlock, out of range the new attempt was just as doomed — and
   with the ST25R3916 it broke single taps: the same phone, still on the reader,
   was re-detected ~300 ms later, aborted the unlock its first detection had
   started, and its own unlock then fell inside the 2 s de-dupe window
   (`DEDUPE_MS`). One tap, zero unlocks. The doorbell now also announces each
   physical tap once (see the relay protocol section). `waitFor()` still returns
   early on `ConnectFailed`/`DiscDone`, which is just correct.
   History of the busy window: it used to be 4 s direct connect + 30 s scan,
   during which announcements were *queued*, then acted on late. Now: the scan is
   8 s (`SCAN_MS`) and **is skipped entirely while the address is cached** until
   three direct connects fail in a row (`DIRECT_FAILURES_BEFORE_SCAN`) — a failed
   direct connect with a known address means "out of range", and scanning only
   held the radio to learn the same thing. The counter **resets when the scan
   runs**, so out of range it scans once per three failures; until 2026-10-06 it
   reset only on success, and every failure after the third paid 4 s + an 8 s
   scan. Out of range the window is now ~4 s.
   In range it is connect + handshake + the lock's ack, ~0.7–1.2 s cold and
   ~0.1 s on a lingering session; the ~1.8 s the motor takes is no longer held
   (the unlock returns on the ack, see the Yale section).
   The base *drops* announcements that arrive meanwhile instead of queueing them,
   and a queued announcement older than 1.5 s is discarded. The 5 s linger after
   a command does **not** hold the radio (`BusyGuard` clears it on return), so a
   second tap right after unlocking goes through. Before this, four stale
   taps were "detected" the instant a failed scan ended and each ran a 0-byte
   transaction against a phone that had left half a minute earlier. Symptom to
   recognise: "works on one tap, then nothing for a while, then a burst".
4. **Lock sharing.** The lock accepts very few simultaneous BLE connections, and
   Home Assistant connects to refresh state after each unlock. At normal tap spacing
   and range this costs at most a retry; if collisions appear in daily use, the fix
   is for HA to take front-door state from this reader over MQTT and stop connecting
   itself. The reader and HA also share one offline key and slot: safe per session
   (each connection derives fresh session keys) but a key rotation would break both
   at once, and unlocks cannot be attributed to a person. The reader should get its
   own slot — create a dedicated account, invite it to the lock, open the lock with
   it once over Bluetooth in person so the key is *loaded*, then extract key and slot.
5. **Relay round trips run 28–160 ms**, above ESP-NOW's usual 5–15 ms; the largest
   is the iPhone's own crypto rather than the link. Against a directly attached
   reader the relay adds ~40 ms to a whole transaction, so this is low priority.
6. **Still to do for a battery doorbell:** the battery divider on A0 and the
   button (both untested on hardware; button wake untested); measure consumption
   by battery voltage over days plus a multimeter in series (no PPK2). A tap
   wake measured 2026-10-06: card announced 336 ms after boot, base auth 91 ms,
   lock accepted the unlock 1.5 s later; the doorbell was done after 2.1 s
   (mostly waiting for the phone to leave the reader, ~0.4 s re-taking the
   wake-up reference).
7. Diagnostics still compiled in: I2C bus scan, BLE scan reports, relay statistics,
   the doorbell's poll-cycle rate (every 1000 cycles) and the ECP frame on push.
8. **Web UI authentication is off by default**; turn it on before leaving a device
   running. Also enable flash encryption — the lock's offline key and the relay
   link key sit in plain NVS.
9. The doorbell's USB console goes quiet after a reset until the port re-enumerates;
   reopen the port rather than assuming the board has crashed. Unplugging USB also
   kills any serial capture that was attached.

## Status

Working end to end (2026-09-16): iPhone Home Key **Express** tap on the doorbell
(phone locked, no Wallet) → relayed over encrypted, MAC-pinned ESP-NOW → base
authenticates in ~190 ms → Yale unlocks over BLE, ~3.2 s next to the lock.

**2026-10-06: the doorbell runs on the MikroE NFC 4 Click (ST25R3916)**, reworked
to I2C (three 0 Ω COMM SEL links moved, R1 removed to kill the power LED,
headers replaced by wires; see `docs/nfc4-click-rework.pdf`). It identified as
`IC_IDENTITY 0x2A (type 0x05 rev 2)` at 0x50 with no driver changes, and the
relayed **endpoint authentication dropped to 86–96 ms**, about half the PN532's.
**Express mode confirmed on the Click** (phone locked, no Wallet: the Home Key
animation appears) - the ST25R3916's ECP path works on our hardware.
IRQ on D1 wakes the doorbell from deep sleep (stage 2, see Hardware). Both
boards recover pairing on their own after either restarts or the AP changes
channel. Dashboard shows pairing, link RSSI, reader-ready and doorbell battery;
MQTT publishes the button and battery with HA discovery. Doorbell button and
battery divider are implemented but untested on real hardware (no button wired,
no divider fitted yet). HA's `yalexs_ble` stays enabled for lock *state*; this
device does the *commands*.
