#pragma once
// Wire format shared by the relay doorbell and the base station.
//
// One ESP-NOW frame carries at most 250 bytes, so payloads larger than the
// per-frame budget (APDUs can reach 255 bytes) are split into fragments that
// the receiver reassembles by (op, seq).
#include <cstdint>
#include <cstddef>

namespace relay {

constexpr uint8_t MAGIC0 = 'H';
constexpr uint8_t MAGIC1 = 'K';
constexpr uint8_t VERSION = 1;
constexpr size_t MAX_ESPNOW = 250;

enum class Op : uint8_t {
  Ping = 1,       // doorbell -> base (broadcast, channel discovery)
  Pong = 2,       // base -> doorbell
  PollReq = 3,    // base -> doorbell: ECP bytes + timeout, start polling
  PollRsp = 4,    // doorbell -> base: found flag, uid, atqa, sak
  ApduReq = 5,    // base -> doorbell: C-APDU
  ApduRsp = 6,    // doorbell -> base: R-APDU (PN532 status bytes stripped)
  PresentReq = 7,
  PresentRsp = 8,
  ReleaseReq = 9,
  ReleaseRsp = 10,
  HealthReq = 11,
  HealthRsp = 12,
};

struct __attribute__((packed)) Header {
  uint8_t magic0, magic1, version;
  uint8_t op;
  uint8_t seq;       // request id, echoed in the response
  uint8_t fragIdx;   // 0-based
  uint8_t fragCnt;   // 1 = unfragmented
  uint8_t flags;     // op-specific (e.g. PollRsp: 1 = tag found)
  uint16_t totalLen; // payload length across all fragments
};
constexpr size_t HDR = sizeof(Header);
constexpr size_t MAX_PAYLOAD = MAX_ESPNOW - HDR;

const char *opName(Op op);

}  // namespace relay
