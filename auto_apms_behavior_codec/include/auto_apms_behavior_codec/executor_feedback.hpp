#pragma once

#include <string>

#include "auto_apms_behavior_codec/util.hpp"
#include "auto_apms_behavior_tree/executor/executor_base.hpp"

namespace auto_apms_behavior_codec
{

/// Alias for the upstream execution state enum from the behavior tree executor.
using ExecutionState = auto_apms_behavior_tree::TreeExecutorBase::ExecutionState;

enum class ExecutorFeedbackType
{
  STATE,        ///< Periodic execution-state report: payload is the state as an integer.
  REGISTERACK,  ///< Build-request acknowledgement: payload is a short FNV-1a hash.
};

inline std::string executorFeedbackTypeToString(ExecutorFeedbackType type)
{
  switch (type) {
    case ExecutorFeedbackType::STATE:
      return "STATE";
    case ExecutorFeedbackType::REGISTERACK:
      return "REGISTERACK";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief Feedback message sent from a TreeDecoderExecutor to a TreeEncoderExecutorProxy.
 *
 * Two types share the same "TYPE:payload" wire format as ExecutorCommandMessage:
 *
 *   `STATE:{state_int}`       — periodic execution-state report, e.g. `STATE:2`
 *   `REGISTERACK:{hash}`      — build-request acknowledgement, e.g. `REGISTERACK:A3F210BC`
 *
 * The hash in REGISTERACK is an 8 hex-character FNV-1a 32-bit fingerprint of the canonical
 * decoded XML, allowing the proxy to correlate the acknowledgement with the build request it
 * transmitted.
 */
struct ExecutorFeedback : public TypePayloadStringMessage
{
  ExecutorFeedbackType type{ExecutorFeedbackType::STATE};

  /// Execution state (STATE messages only).
  ExecutionState state{ExecutionState::IDLE};

  /// Build-request fingerprint (REGISTERACK messages only).
  std::string build_hash;

  static ExecutorFeedback makeState(ExecutionState s)
  {
    ExecutorFeedback f;
    f.type = ExecutorFeedbackType::STATE;
    f.state = s;
    return f;
  }

  static ExecutorFeedback makeRegisterAck(const std::string & hash)
  {
    ExecutorFeedback f;
    f.type = ExecutorFeedbackType::REGISTERACK;
    f.build_hash = hash;
    return f;
  }

  std::string encode() const override
  {
    switch (type) {
      case ExecutorFeedbackType::STATE:
        return format("STATE", std::to_string(static_cast<int32_t>(state)));
      case ExecutorFeedbackType::REGISTERACK:
        return format("REGISTERACK", build_hash);
      default:
        return "";
    }
  }

  static ExecutorFeedback decode(const std::string & msg)
  {
    ExecutorFeedback feedback;
    auto [type_str, payload] = split(msg);

    if (type_str == "STATE") {
      feedback.type = ExecutorFeedbackType::STATE;
      try {
        feedback.state = static_cast<ExecutionState>(std::stoi(payload));
      } catch (...) {
      }
    } else if (type_str == "REGISTERACK") {
      feedback.type = ExecutorFeedbackType::REGISTERACK;
      feedback.build_hash = payload;
    }
    return feedback;
  }
};

}  // namespace auto_apms_behavior_codec
