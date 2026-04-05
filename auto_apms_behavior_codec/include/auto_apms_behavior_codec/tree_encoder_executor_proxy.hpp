#pragma once

#include "auto_apms_behavior_codec/behavior_tree_encoder_base.hpp"
#include "auto_apms_behavior_codec/encoder_executor_proxy_params.hpp"
#include "auto_apms_behavior_codec/executor_command.hpp"
#include "auto_apms_behavior_codec/executor_telemetry.hpp"
#include "auto_apms_behavior_codec_interfaces/msg/executor_command_message.hpp"
#include "auto_apms_behavior_codec_interfaces/msg/serialized_telemetry_message.hpp"
#include "auto_apms_interfaces/action/start_tree_executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace auto_apms_behavior_codec
{

/**
 * @brief Action-server proxy that bridges `StartTreeExecutor` goals to a remote executor via the codec layer.
 *
 * The proxy exposes the same `StartTreeExecutor` action interface that `TreeExecutorNode` provides, allowing
 * ground-station or operator nodes to start behavior-tree execution on a remote machine transparently. Internally
 * it encodes the received tree XML, publishes it over the codec's serialized-tree topic, and drives a state machine
 * that sends executor commands and monitors telemetry until execution completes.
 *
 * ### State Machine
 *
 * ```
 * IDLE ──▶ REGISTER_BEHAVIOR ──▶ COMMAND_EXECUTOR ──▶ AWAIT_RESULT ──▶ IDLE
 * ```
 *
 * | State               | Activity                                                                            |
 * |---------------------|-------------------------------------------------------------------------------------|
 * | IDLE                | No active goal. Waiting for an incoming action request.                             |
 * | REGISTER_BEHAVIOR   | Tree XML has been encoded and published. Waiting for the remote executor's           |
 * |                     | telemetry to list the expected tree name(s), confirming registration.                |
 * | COMMAND_EXECUTOR    | Sends a `START` command with the entry-point tree name to the remote executor.       |
 * | AWAIT_RESULT        | Monitors telemetry. The goal succeeds when the executor returns to IDLE after having |
 * |                     | been active. A cancel request triggers a `CANCEL` command.                           |
 *
 * If `build_request` is empty but `entry_point` is set, encoding is skipped and the proxy jumps directly to
 * COMMAND_EXECUTOR (useful when the tree is already registered on the remote side).
 *
 * ### Parameters (encoder_executor_proxy_params)
 *
 * | Parameter                          | Default                  | Description                           |
 * |------------------------------------|--------------------------|---------------------------------------|
 * | `start_tree_executor_action_name`  | `start_tree_executor`    | Action server name                    |
 * | `executor_command_topic`           | `executor_command`       | Topic to publish ExecutorCommandMessage|
 * | `telemetry_topic`                  | `serialized_telemetry_in`| Topic to subscribe to telemetry       |
 */
class TreeEncoderExecutorProxy : public BehaviorTreeEncoderBase
{
public:
  using StartTreeExecutor = auto_apms_interfaces::action::StartTreeExecutor;
  using GoalHandle = rclcpp_action::ServerGoalHandle<StartTreeExecutor>;

  /// Only this build handler is accepted; other values cause goal rejection.
  static constexpr const char * SUPPORTED_BUILD_HANDLER =
    "auto_apms_behavior_tree::TreeFromStringBuildHandler";

  /// Internal state of the proxy's execution lifecycle.
  enum class ProxyState : uint8_t
  {
    IDLE,               ///< No active goal.
    REGISTER_BEHAVIOR,  ///< Encoded tree published; awaiting remote registration via telemetry.
    COMMAND_EXECUTOR,   ///< Sending START command to the remote executor.
    AWAIT_RESULT        ///< Monitoring telemetry until execution finishes or is cancelled.
  };

  explicit TreeEncoderExecutorProxy(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// Decide whether to accept or reject an incoming goal.
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const StartTreeExecutor::Goal> goal);

  /// Decide whether to accept a cancellation request.
  rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandle> goal_handle);

  /// Called when a goal is accepted; starts the state machine timer.
  void handleAccepted(std::shared_ptr<GoalHandle> goal_handle);

  /// Periodic callback (100 ms) that drives the REGISTER → COMMAND → AWAIT state transitions.
  void stateMachineCallback();

  /// Reset state machine fields and cancel the timer.
  void resetStateMachine();

  /// Complete the active goal with a result code and message.
  void completeGoal(uint8_t tree_result, const std::string & message);

  /// Abort the active goal with an error message.
  void abortGoal(const std::string & message);

  /// Mark the active goal as cancelled.
  void cancelGoal(const std::string & message);

  /// Validate that every node in the encoded manifest is known to the dictionary.
  bool validateNodeManifest(const std::string & encoded_manifest);

  /// Subscription callback that deserializes incoming telemetry updates.
  void telemetryCallback(
    const auto_apms_behavior_codec_interfaces::msg::SerializedTelemetryMessage::SharedPtr msg);

  // Parameters
  encoder_executor_proxy_params::ParamListener param_listener_;

  // ROS interfaces
  rclcpp_action::Server<StartTreeExecutor>::SharedPtr action_server_;
  rclcpp::Publisher<auto_apms_behavior_codec_interfaces::msg::ExecutorCommandMessage>::SharedPtr command_publisher_;
  rclcpp::Subscription<auto_apms_behavior_codec_interfaces::msg::SerializedTelemetryMessage>::SharedPtr
    telemetry_subscription_;
  rclcpp::TimerBase::SharedPtr state_machine_timer_;

  // State machine
  ProxyState proxy_state_{ProxyState::IDLE};
  std::shared_ptr<GoalHandle> active_goal_handle_;
  std::vector<std::string> expected_tree_names_;
  std::string entry_point_;
  bool seen_execution_active_{false};
  bool cancel_command_sent_{false};
  ExecutorTelemetry latest_telemetry_;
};

}  // namespace auto_apms_behavior_codec
