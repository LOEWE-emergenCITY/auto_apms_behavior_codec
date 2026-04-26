#pragma once

#include <string>
#include <utility>

#include "auto_apms_behavior_codec/util.hpp"

namespace auto_apms_behavior_codec
{

/**
 * @brief Command types sent to a remote executor via ExecutorCommandMessage.
 */
enum class ExecutorCommandType
{
  UNKNOWN, ///< Unrecognized command type (returned by ExecutorCommand::decode() on parse failure).
  START,   ///< Start tree execution. Payload: entry-point tree name.
  STOP,    ///< Stop tree execution (halts cleanly).
  PAUSE,   ///< Pause tree execution.
  RESUME,  ///< Resume a paused execution.
  CANCEL,  ///< Cancel (terminate) current tree execution. Payload is typically empty.
};

inline std::string executorCommandTypeToString(ExecutorCommandType type)
{
  switch (type) {
    case ExecutorCommandType::START:
      return "START";
    case ExecutorCommandType::STOP:
      return "STOP";
    case ExecutorCommandType::PAUSE:
      return "PAUSE";
    case ExecutorCommandType::RESUME:
      return "RESUME";
    case ExecutorCommandType::CANCEL:
      return "CANCEL";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief Executor command message using the `"TYPE:payload"` wire format.
 *
 * Commands are sent from a TreeEncoderExecutorProxy to a remote TreeDecoderExecutor.
 * Use the static factory methods to construct commands and call encode() to serialize:
 *
 * @code
 * ExecutorCommand::makeStart("MainTree").encode()  →  "START:MainTree"
 * ExecutorCommand::makeCancel().encode()            →  "CANCEL:"
 * @endcode
 */
struct ExecutorCommand : public TypePayloadStringMessage
{
  ExecutorCommandType type{ExecutorCommandType::UNKNOWN};
  std::string payload;

  ExecutorCommand() = default;
  ExecutorCommand(ExecutorCommandType t, std::string p = {})
  : type(t), payload(std::move(p))
  {
  }

  static ExecutorCommand makeStart(const std::string & entry_point)
  {
    return {ExecutorCommandType::START, entry_point};
  }
  static ExecutorCommand makeStop() { return {ExecutorCommandType::STOP}; }
  static ExecutorCommand makePause() { return {ExecutorCommandType::PAUSE}; }
  static ExecutorCommand makeResume() { return {ExecutorCommandType::RESUME}; }
  static ExecutorCommand makeCancel() { return {ExecutorCommandType::CANCEL}; }

  std::string encode() const override
  {
    return format(executorCommandTypeToString(type), payload);
  }

  /// Decode a `"TYPE:payload"` string. Returns a command with type UNKNOWN on parse failure.
  static ExecutorCommand decode(const std::string & msg)
  {
    auto [type_str, pay] = split(msg);
    if (type_str == "START")  return {ExecutorCommandType::START,  pay};
    if (type_str == "STOP")   return {ExecutorCommandType::STOP,   pay};
    if (type_str == "PAUSE")  return {ExecutorCommandType::PAUSE,  pay};
    if (type_str == "RESUME") return {ExecutorCommandType::RESUME, pay};
    if (type_str == "CANCEL") return {ExecutorCommandType::CANCEL, pay};
    return {};
  }
};

}  // namespace auto_apms_behavior_codec
