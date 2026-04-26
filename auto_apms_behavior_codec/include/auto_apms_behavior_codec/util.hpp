#pragma once

#include <cstdint>
#include <string>
#include <utility>

namespace auto_apms_behavior_codec
{

/**
 * @brief Abstract base class for codec channel messages encoded as `"TYPE:payload"` strings.
 *
 * Establishes the convention that all messages in the codec communication layer are represented
 * as a plain string of the form `"TYPE:payload"` where TYPE is an uppercase keyword identifying
 * the message variant and payload is variant-specific content. Both command (proxy → executor)
 * and feedback (executor → proxy) directions follow this convention, keeping the wire format
 * uniform and easy to forward over constrained links such as LoRa.
 *
 * Derived types must implement encode() and provide a static decode() factory method:
 *
 * @code
 * struct MyMessage : public TypePayloadStringMessage {
 *   std::string encode() const override { return format("TYPE", payload_); }
 *   static MyMessage decode(const std::string & msg) {
 *     auto [type, payload] = split(msg);
 *     // ...
 *   }
 * };
 * @endcode
 */
class TypePayloadStringMessage
{
public:
  virtual ~TypePayloadStringMessage() = default;

  /// Encode the message to a `"TYPE:payload"` wire string.
  virtual std::string encode() const = 0;

protected:
  /// Build a `"TYPE:payload"` string from its two components.
  static std::string format(const std::string & type, const std::string & payload)
  {
    return type + ":" + payload;
  }

  /// Split a `"TYPE:payload"` string into {type, payload}.
  /// Returns {"", ""} if the colon separator is absent.
  static std::pair<std::string, std::string> split(const std::string & msg)
  {
    const auto colon = msg.find(':');
    if (colon == std::string::npos) return {"", ""};
    return {msg.substr(0, colon), msg.substr(colon + 1)};
  }
};

/**
 * @brief Compute a compact 8 hex-character FNV-1a 32-bit fingerprint of @p data.
 *
 * Used to produce a short, unique identifier for a build request so that a REGISTERACK message
 * can be correlated with the specific build request that triggered it. At only 8 characters over
 * the wire this stays within tight LoRa bandwidth budgets.
 */
std::string computeUniqueStringHash(const std::string & data);

}  // namespace auto_apms_behavior_codec
