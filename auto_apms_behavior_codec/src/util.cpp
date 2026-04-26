#include "auto_apms_behavior_codec/util.hpp"

#include <cstdio>

namespace auto_apms_behavior_codec
{

std::string computeUniqueStringHash(const std::string & data)
{
  uint32_t hash = 2166136261u;
  for (unsigned char c : data) {
    hash ^= static_cast<uint32_t>(c);
    hash *= 16777619u;
  }
  char buf[9];
  std::snprintf(buf, sizeof(buf), "%08X", hash);
  return std::string(buf);
}

}  // namespace auto_apms_behavior_codec
