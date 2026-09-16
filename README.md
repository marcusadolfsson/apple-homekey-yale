# apple-homekey-yale

Tap an iPhone (Apple Home Key, Express Mode) on a reader and a **Yale nexTouch**
mortise lock unlocks over Bluetooth — no cloud, no hub and no Yale app in the
unlock path. The reader can also be split in two, so the part by the door holds
no credentials and can run on a battery.

Built on [rednblkx/HomeKey-ESP32](https://github.com/rednblkx/HomeKey-ESP32),
which provides the HomeKit and Home Key side. Its documentation still applies to
everything below — see [their docs](https://rednblkx.github.io/HomeKey-ESP32/) and
the original README, kept here as [README.upstream.md](README.upstream.md).

## What this repository adds

| Addition | Where |
|---|---|
| **Yale/August BLE lock driver** — a C++ port of `yalexs-ble`: offline-key handshake, unlock, GATT cache in NVS | `main/YaleBleLock.*` |
| **PN532 over I2C** (upstream supports SPI only) | `main/Pn532I2cTransport.*`, reader type 3 |
| **Relay reader** — the NFC front end lives on a second board, reached over ESP-NOW | `main/RemoteNfcReader.*`, reader type 4 |
| **Relay doorbell firmware** — holds no keys, never joins WiFi, announces taps | `relay-doorbell/` |
| Single 3.75 MB app partition (the BLE stack outgrew the OTA layout) | `single_app.csv` |
| Doorbell link status (paired / signal strength) on the dashboard | web UI |

**[CLAUDE.md](CLAUDE.md) is the real documentation**: architecture, the Yale wire
protocol, measurements, hard-won gotchas, and what is still unfinished.

## Measured

Tap to door open, reader next to the lock: **~3.2 s** — HomeKey exchange 307 ms,
BLE connect 722 ms, handshake 367 ms, and **1.77 s of that is the lock's own
mechanism**. Apple's Home Key tolerates the relay: exchanges of 42–158 ms still
complete. Range matters more than anything else — the same firmware takes 9.9 s
when the base is far from the lock.

## Build

ESP-IDF v5.5.5, target ESP32-C6:

```bash
. ~/esp/esp-idf/export.sh
idf.py set-target esp32c6 && idf.py build      # base / single-device firmware
cd relay-doorbell && idf.py build              # doorbell firmware
```

OTA is disabled, so flash over USB — and flash the parts individually rather than
the merged image, or you will erase the HomeKit pairing and keys in NVS. See
CLAUDE.md for the exact command and for backing up NVS first.

## Security notes

The lock's offline key is entered in the device's web page, masked on read-back,
and never logged or committed. It lives in NVS, which is **not encrypted** by
default — turn on flash encryption and the web UI login before deploying. The
ESP-NOW link between base and doorbell is currently unauthenticated.

MIT licensed, as upstream.
