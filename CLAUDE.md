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

**Behaviour: unlock only, fire and forget.** The reader does not track lock state;
HA and the lock's own HomeKit module do that. A mortise nexTouch relocks itself.

## Relay protocol (`main/RelayProtocol.hpp`)

ESP-NOW, 10-byte header, fragmented above 240 bytes, one request in flight.
Unencrypted for now — see "Known issues".

- `Ping`/`Pong` — pairing, and it works from **either** side: the doorbell walks the
  WiFi channels until a base answers, and a base with no doorbell advertises every
  3 s. Either box can restart without breaking the link. No MAC is configured;
  whoever answers first is adopted (trust on first contact).
- `EcpReq`/`EcpSet` — the base hands the doorbell the 18-byte ECP frame and a poll
  interval, so the doorbell can drive its own reader.
- **`TagEvent`** — the doorbell announces a card. **The base sends nothing between
  taps**: polling was inverted so a battery doorbell can sleep and so the radio is
  free for BLE. The doorbell holds the card until `ReleaseReq` (or 5 s).
- `ApduReq/Rsp`, `PresentReq/Rsp`, `ReleaseReq/Rsp` — the transaction itself.
- `HealthRsp` — unsolicited heartbeat every 30 s; liveness is inferred from traffic
  received rather than polled for (`HEARTBEAT_TIMEOUT_MS`).
- `ButtonPress` — the doorbell button.

**Tap announcements have their own queue** on the base. Sharing one queue with
request/response replies made the polling wait swallow them.

### Doorbell button and battery

- **Button on D1** (any momentary switch to GND; internal pull-up, debounced 50 ms).
  D1 is a low-power pin, so in a battery build the same wire wakes the chip from
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
  tolerant), SDA→D4 (GPIO22), SCL→D5 (GPIO23), DIP switches to I2C.
- **ST25R3916** is the intended production reader: ~3 µA wake-up (low-power card
  detection) versus a PN532 that costs milliamps just to look for a card.
- Reader type 0 = PN532 SPI (upstream defaults SS=18/MOSI=19/MISO=20/SCK=21),
  1 = PN7160, 2 = ST25R3916, **3 = PN532 I2C**, **4 = Relay (ESP-NOW doorbell)**.

## Build and flash

```bash
. ~/esp/esp-idf/export.sh          # ESP-IDF v5.5.5
idf.py set-target esp32c6 && idf.py build          # base firmware
cd relay-doorbell && idf.py build                  # doorbell firmware
```

The app outgrew the two-slot OTA layout (2.1 MB), so `single_app.csv` gives it one
3.75 MB partition: **OTA is disabled; flash over USB.** To keep pairing, keys and
settings, flash the parts individually and never the merged image:

```bash
esptool --chip esp32c6 -p PORT write_flash \
  0x0 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x20000 build/HomeKey-ESP32.bin \
  0x3e0000 build/spiffs.bin        # NVS at 0x9000 is untouched
```

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

1. **The relay link is unencrypted and unauthenticated, and pairing is trust-on-
   first-contact.** No MAC is configured, so any ESP-NOW device in range can claim
   to be the doorbell. It cannot forge an unlock (the base verifies the phone
   cryptographically) but it can disrupt. Next: an optional "Doorbell MAC" setting
   plus encrypted ESP-NOW peers with a shared key.
2. **Lock sharing.** The lock accepts very few simultaneous BLE connections, and
   Home Assistant connects to refresh state after each unlock. At normal tap spacing
   and range this costs at most a retry; if collisions appear in daily use, the fix
   is for HA to take front-door state from this reader over MQTT and stop connecting
   itself. The reader and HA also share one offline key and slot: safe per session
   (each connection derives fresh session keys) but a key rotation would break both
   at once, and unlocks cannot be attributed to a person. The reader should get its
   own slot — create a dedicated account, invite it to the lock, open the lock with
   it once over Bluetooth in person so the key is *loaded*, then extract key and slot.
3. **Relay round trips run 28–160 ms**, above ESP-NOW's usual 5–15 ms; the largest
   is the iPhone's own crypto rather than the link. Against a directly attached
   reader the relay adds ~40 ms to a whole transaction, so this is low priority.
4. **Still to do for a battery doorbell:** ST25R3916 in place of the PN532 (its
   low-power card detection is the ~3 µA that makes idling possible), deep sleep
   between taps with wake on the reader's IRQ and on the button, and a measurement
   with a Nordic PPK2 rather than an estimate.
5. Diagnostics still compiled in: I2C bus scan, BLE scan reports, relay statistics.
6. **Web UI authentication is off by default**; turn it on before leaving a device
   running. Also enable flash encryption — the lock's offline key sits in plain NVS.
7. The doorbell's USB console goes quiet after a reset until the port re-enumerates;
   reopen the port rather than assuming the board has crashed.

## Status

Working end to end: iPhone Home Key tap on the doorbell → relayed over ESP-NOW →
base authenticates → Yale unlocks over BLE, ~3.2 s next to the lock. Both boards
recover pairing on their own after either restarts. Doorbell button and battery
reporting are implemented but untested on real hardware (no button wired, no
divider fitted yet).
