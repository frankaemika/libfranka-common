#pragma once

#include <cstdint>

namespace research_interface {

using Version = uint16_t;

enum class Function : uint32_t {
  kConnect = 0
};

struct ConnectRequest {
  Function function;
  Version version;
  uint16_t udp_port;
};

struct ConnectReply {
  enum class Status : uint32_t {
    kSuccess = 0,
    kIncompatibleLibraryVersion = 1
  };
  Status status;
  Version version;
};

}  // namespace research_interface
