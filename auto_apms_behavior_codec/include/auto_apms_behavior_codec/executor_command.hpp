#pragma once

#include <string>

namespace auto_apms_behavior_codec
{

/**
 * @brief Command types sent to a remote executor via ExecutorCommandMessage.
 *
 * Commands are serialized as `"TYPE:payload"` strings where the type is the enum name
 * and the payload is command-specific (e.g. a tree name for START).
 */
enum class ExecutorCommandType
{
  START,   ///< Start tree execution. Payload: tree identity, e.g. `"TreeName(arg1=val1)"`.
  STOP,    ///< Stop tree execution (reserved for future use).
  PAUSE,   ///< Pause tree execution (reserved for future use).
  RESUME,  ///< Resume tree execution (reserved for future use).
  CANCEL   ///< Cancel current tree execution. Payload is typically empty.
};

/**
 * @brief Convert an ExecutorCommandType to its string representation.
 * @param type The command type.
 * @return Uppercase string name of the command (e.g. `"START"`).
 */
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
 * @brief Format an executor command string.
 * @param type The command type.
 * @param payload Command-specific payload (may be empty).
 * @return Formatted string `"TYPE:payload"`.
 */
inline std::string formatExecutorCommand(ExecutorCommandType type, const std::string & payload)
{
  return executorCommandTypeToString(type) + ":" + payload;
}

}  // namespace auto_apms_behavior_codec
