#pragma once

#include <memory>
#include <string>
#include <vector>

#include "auto_apms_behavior_codec/behavior_tree_decoder_base.hpp"
#include "auto_apms_behavior_codec/decoder_executor_params.hpp"
#include "auto_apms_behavior_codec/executor_command.hpp"
#include "auto_apms_behavior_codec/executor_feedback.hpp"
#include "auto_apms_behavior_codec_interfaces/msg/executor_command_message.hpp"
#include "auto_apms_behavior_codec_interfaces/msg/executor_feedback_message.hpp"
#include "auto_apms_behavior_tree/executor/generic_executor_node.hpp"

namespace auto_apms_behavior_codec
{

/**
 * @brief Decoder-side executor that serves as the remote endpoint of a TreeEncoderExecutorProxy.
 *
 * Inherits BehaviorTreeDecoderBase for the CBOR-to-XML decoding pipeline and holds a
 * GenericTreeExecutorNode for behavior tree execution.  Together this enables transparent
 * remote behavior tree execution via the codec layer.
 *
 * ### Communication
 *
 * | Direction  | Topic (default)              | Message                         | Purpose                            |
 * |------------|------------------------------|---------------------------------|------------------------------------|
 * | Subscribe  | `serialized_tree_in`         | `SerializedTreeMessage`         | Receive CBOR-encoded tree from proxy |
 * | Subscribe  | `executor_command`           | `ExecutorCommandMessage`        | Receive START / CANCEL commands    |
 * | Publish    | `executor_feedback_out`      | `ExecutorFeedbackMessage`       | Report state + registered trees    |
 *
 * ### Interaction with TreeEncoderExecutorProxy
 *
 * The proxy's state machine transitions depend on this executor's telemetry:
 *
 * 1. **REGISTER_BEHAVIOR**: Proxy publishes encoded tree. This executor decodes it, stores
 *    the XML, and reports the registered tree names via telemetry.
 * 2. **COMMAND_EXECUTOR**: Proxy sends a `START` command. This executor starts tree execution.
 * 3. **AWAIT_RESULT**: Proxy monitors telemetry state. This executor reports execution state
 *    transitions (STARTING → RUNNING → IDLE). The proxy detects completion when the state
 *    returns to IDLE after having been active.
 *
 * ### Parameters
 *
 * Base decoder parameters (from BehaviorTreeDecoderBase / decoder_base_params):
 *
 * | Parameter              | Default                  | Description                                  |
 * |------------------------|--------------------------|----------------------------------------------|
 * | `node_manifest`        | `[]`                     | Node manifest identities for the dictionary  |
 * | `encoded_in_topic`     | `serialized_tree_in`     | Incoming serialized tree topic               |
 *
 * Additional parameters (decoder_executor_params):
 *
 * | Parameter                   | Default                    | Description                                |
 * |-----------------------------|----------------------------|--------------------------------------------||
 * | `executor_command_in_topic` | `executor_command`         | Incoming command topic                     |
 * | `feedback_out_topic`        | `executor_feedback_out`    | Outgoing feedback topic                    |
 * | `feedback_interval`         | `2.0`                      | Interval in seconds between state feedback publications (independent from `tick_rate`) |
 *
 * In addition, all GenericTreeExecutorNode parameters (tick_rate, groot2_port, etc.) are available.
 */
class TreeDecoderExecutor : public BehaviorTreeDecoderBase
{
public:
  explicit TreeDecoderExecutor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /// Called by BehaviorTreeDecoderBase when a tree has been successfully decoded.
  void onTreeDecoded(const std::string & xml_string, const std::string & encoded_bytes_hash) override;

private:
  /// Set up the executor member, command subscription, and telemetry interfaces.
  void setupExecutorAndCodecInterfaces();

  /// Callback for incoming executor commands (START, CANCEL, etc.).
  void commandCallback(
    const auto_apms_behavior_codec_interfaces::msg::ExecutorCommandMessage::SharedPtr msg);

  /// Timer callback that checks for state changes and publishes feedback.
  void publishStateFeedback();

  // Codec parameters (executor_command_in_topic, feedback_out_topic)
  decoder_executor_params::ParamListener param_listener_;

  // Behavior tree executor (initialized after construction via one-shot timer)
  std::unique_ptr<auto_apms_behavior_tree::GenericTreeExecutorNode> executor_;

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::Subscription<auto_apms_behavior_codec_interfaces::msg::ExecutorCommandMessage>::SharedPtr
    command_subscription_;
  rclcpp::Publisher<auto_apms_behavior_codec_interfaces::msg::ExecutorFeedbackMessage>::SharedPtr
    feedback_publisher_;
  rclcpp::TimerBase::SharedPtr state_feedback_timer_;
  ExecutionState last_published_state_{ExecutionState::IDLE};

  // Registered behaviors are kept in a TreeDocument
  auto_apms_behavior_tree::core::TreeDocument decoding_verification_doc_;
  auto_apms_behavior_tree::core::TreeDocument behavior_library_doc_;
};

}  // namespace auto_apms_behavior_codec
