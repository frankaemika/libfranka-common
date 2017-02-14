#pragma once

#include <cstdint>

namespace research_interface {

using Version = uint16_t;

constexpr Version kVersion = 1;
constexpr uint16_t kCommandPort = 1337;

enum class Function : uint32_t { kConnect = 0 };

struct ConnectRequest {
  ConnectRequest(uint16_t udp_port)
      : function(Function::kConnect), version(kVersion), udp_port(udp_port) {}

  const Function function;
  const Version version;
  const uint16_t udp_port;
};

struct ConnectReply {
  enum class Status : uint32_t {
    kSuccess = 0,
    kIncompatibleLibraryVersion = 1
  };

  ConnectReply(Status status) : status(status), version(kVersion) {}

  const Status status;
  const Version version;
};

}  // namespace research_interface
