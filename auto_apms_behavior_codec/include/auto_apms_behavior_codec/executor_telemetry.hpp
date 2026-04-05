#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "auto_apms_behavior_codec/telemetry_message_builder.hpp"
#include "auto_apms_behavior_tree/executor/executor_base.hpp"

namespace auto_apms_behavior_codec
{

/// Alias for the upstream execution state enum from the behavior tree executor.
using ExecutionState = auto_apms_behavior_tree::TreeExecutorBase::ExecutionState;

/**
 * @brief Telemetry snapshot received from a remote executor via SerializedTelemetryMessage.
 *
 * Encapsulates the current execution state and the list of behavior trees registered on the
 * remote executor. Uses TelemetryMessageBuilder (CBOR) for compact serialization with the keys:
 *   - `"state"` — Current executor state as int32 (maps to TreeExecutorBase::ExecutionState).
 *   - `"behaviors"` — Names of behavior trees registered on the remote executor.
 */
struct ExecutorTelemetry
{
  /// Names of behavior trees registered on the remote executor.
  std::vector<std::string> behaviors;

  /// Current execution state of the remote executor.
  ExecutionState state{ExecutionState::IDLE};

  /**
   * @brief Encode this telemetry snapshot to a CBOR byte vector.
   * @return Serialized bytes suitable for a SerializedTelemetryMessage.
   */
  std::vector<uint8_t> encode() const
  {
    TelemetryMessageBuilder builder;
    builder.addIntEntry("state", static_cast<int32_t>(state));
    builder.addStringArrayEntry("behaviors", behaviors);
    return builder.getSerializedMessage();
  }

  /**
   * @brief Decode a telemetry snapshot from CBOR bytes.
   * @param data Serialized bytes from a SerializedTelemetryMessage.
   * @return Decoded telemetry. Returns defaults (IDLE, empty behaviors) on invalid input.
   */
  static ExecutorTelemetry decode(const std::vector<uint8_t> & data)
  {
    ExecutorTelemetry telemetry;
    if (data.empty()) return telemetry;

    TelemetryMessageBuilder builder;
    if (!builder.fromSerializedMessage(data)) return telemetry;

    const auto & entries = builder.getMessage().entries;

    auto state_it = entries.find("state");
    if (state_it != entries.end()) {
      if (auto * p = dynamic_cast<TelemetryMessage::TelemetryMessageEntryInt *>(state_it->second.get())) {
        telemetry.state = static_cast<ExecutionState>(p->value);
      }
    }

    auto behaviors_it = entries.find("behaviors");
    if (behaviors_it != entries.end()) {
      if (auto * p =
            dynamic_cast<TelemetryMessage::TelemetryMessageEntryStringArray *>(behaviors_it->second.get()))
      {
        telemetry.behaviors = p->value;
      }
    }

    return telemetry;
  }
};

}  // namespace auto_apms_behavior_codec
