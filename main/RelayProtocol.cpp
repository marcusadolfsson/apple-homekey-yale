#include "RelayProtocol.hpp"

namespace relay {
const char *opName(Op op) {
  switch (op) {
    case Op::Ping: return "ping";
    case Op::Pong: return "pong";
    case Op::PollReq: return "poll";
    case Op::PollRsp: return "poll-rsp";
    case Op::ApduReq: return "apdu";
    case Op::ApduRsp: return "apdu-rsp";
    case Op::PresentReq: return "present";
    case Op::PresentRsp: return "present-rsp";
    case Op::ReleaseReq: return "release";
    case Op::ReleaseRsp: return "release-rsp";
    case Op::HealthReq: return "health";
    case Op::HealthRsp: return "health-rsp";
    case Op::EcpReq: return "ecp-req";
    case Op::EcpSet: return "ecp-set";
    case Op::TagEvent: return "tag-event";
  }
  return "?";
}
}  // namespace relay
