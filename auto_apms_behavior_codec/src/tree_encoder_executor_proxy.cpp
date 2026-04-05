#include "auto_apms_behavior_codec/tree_encoder_executor_proxy.hpp"

#include <algorithm>

#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"

namespace auto_apms_behavior_codec
{

TreeEncoderExecutorProxy::TreeEncoderExecutorProxy(const rclcpp::NodeOptions & options)
: BehaviorTreeEncoderBase("tree_encoder_executor_proxy", options), param_listener_(this)
{
  const auto params = param_listener_.get_params();

  action_server_ = rclcpp_action::create_server<StartTreeExecutor>(
    this, params.start_tree_executor_action_name,
    std::bind(&TreeEncoderExecutorProxy::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TreeEncoderExecutorProxy::handleCancel, this, std::placeholders::_1),
    std::bind(&TreeEncoderExecutorProxy::handleAccepted, this, std::placeholders::_1));

  command_publisher_ = this->create_publisher<auto_apms_behavior_codec_interfaces::msg::ExecutorCommandMessage>(
    params.executor_command_topic, 10);

  telemetry_subscription_ =
    this->create_subscription<auto_apms_behavior_codec_interfaces::msg::SerializedTelemetryMessage>(
      params.telemetry_topic, 10,
      std::bind(&TreeEncoderExecutorProxy::telemetryCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(), "Executor proxy action server created on '%s', command topic '%s', telemetry topic '%s'",
    params.start_tree_executor_action_name.c_str(), params.executor_command_topic.c_str(),
    params.telemetry_topic.c_str());
}

bool TreeEncoderExecutorProxy::validateNodeManifest(const std::string & encoded_manifest)
{
  if (encoded_manifest.empty()) {
    return true;
  }

  try {
    auto manifest = auto_apms_behavior_tree::core::NodeManifest::decode(encoded_manifest);
    auto node_names = manifest.getNodeNames();
    auto dict = getDictionaryManager();

    for (const auto & name : node_names) {
      auto entry = dict->get_dictionary_info_by_name(name);
      if (!entry.supported) {
        RCLCPP_WARN(
          this->get_logger(), "Node '%s' from goal manifest is not known by the encoder dictionary", name.c_str());
        return false;
      }
    }
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to decode node manifest from goal: %s", e.what());
    return false;
  }
}

rclcpp_action::GoalResponse TreeEncoderExecutorProxy::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const StartTreeExecutor::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");

  if (proxy_state_ != ProxyState::IDLE) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal: proxy is busy with another goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->build_request.empty() && goal->entry_point.empty()) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal: both build_request and entry_point are empty");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!goal->build_request.empty()) {
    if (goal->build_handler != SUPPORTED_BUILD_HANDLER) {
      RCLCPP_WARN(
        this->get_logger(), "Rejecting goal: unsupported build_handler '%s' (expected '%s')",
        goal->build_handler.c_str(), SUPPORTED_BUILD_HANDLER);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!validateNodeManifest(goal->node_manifest)) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: node manifest contains unknown nodes");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TreeEncoderExecutorProxy::handleCancel(
  std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TreeEncoderExecutorProxy::handleAccepted(std::shared_ptr<GoalHandle> goal_handle)
{
  active_goal_handle_ = goal_handle;
  const auto goal = goal_handle->get_goal();
  entry_point_ = goal->entry_point;
  expected_tree_names_.clear();
  seen_execution_active_ = false;
  cancel_command_sent_ = false;

  if (!goal->build_request.empty()) {
    // Use TreeDocument for XML introspection to extract tree names
    auto_apms_behavior_tree::core::TreeDocument tree_doc;
    tree_doc.mergeString(goal->build_request, true);
    tree_doc.registerNodes(getNodeManifest());
    expected_tree_names_ = tree_doc.getAllTreeNames();

    RCLCPP_INFO(this->get_logger(), "Encoding tree with %zu sub-tree(s)", expected_tree_names_.size());

    // Encode and publish the serialized tree
    if (!encodeAndPublish(goal->build_request)) {
      abortGoal("Failed to encode tree XML");
      return;
    }

    proxy_state_ = ProxyState::REGISTER_BEHAVIOR;
  } else {
    // No build_request: skip to COMMAND_EXECUTOR (start previously registered tree)
    proxy_state_ = ProxyState::COMMAND_EXECUTOR;
  }

  // Start periodic state machine timer
  state_machine_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&TreeEncoderExecutorProxy::stateMachineCallback, this));
}

void TreeEncoderExecutorProxy::stateMachineCallback()
{
  if (!active_goal_handle_) {
    resetStateMachine();
    return;
  }

  const bool is_canceling = active_goal_handle_->is_canceling();

  switch (proxy_state_) {
    case ProxyState::REGISTER_BEHAVIOR: {
      if (is_canceling) {
        cancelGoal("Canceled during behavior registration");
        return;
      }

      // Check if all expected trees are registered on the remote executor
      bool all_registered = true;
      for (const auto & name : expected_tree_names_) {
        if (std::find(latest_telemetry_.behaviors.begin(), latest_telemetry_.behaviors.end(), name) ==
            latest_telemetry_.behaviors.end())
        {
          all_registered = false;
          break;
        }
      }

      if (all_registered) {
        RCLCPP_INFO(this->get_logger(), "All behavior trees registered on remote executor");
        if (entry_point_.empty()) {
          // Register-only mode: no entry_point means skip execution
          completeGoal(
            StartTreeExecutor::Result::TREE_RESULT_NOT_SET, "Behavior trees registered (register-only mode)");
        } else {
          proxy_state_ = ProxyState::COMMAND_EXECUTOR;
        }
      }
      break;
    }

    case ProxyState::COMMAND_EXECUTOR: {
      if (is_canceling) {
        cancelGoal("Canceled before starting execution");
        return;
      }

      // Send START command with the entry point
      auto cmd_msg = auto_apms_behavior_codec_interfaces::msg::ExecutorCommandMessage();
      cmd_msg.executor_command_message = formatExecutorCommand(ExecutorCommandType::START, entry_point_);
      command_publisher_->publish(cmd_msg);
      RCLCPP_INFO(this->get_logger(), "Sent start command: '%s'", cmd_msg.executor_command_message.c_str());

      proxy_state_ = ProxyState::AWAIT_RESULT;
      break;
    }

    case ProxyState::AWAIT_RESULT: {
      const auto state = latest_telemetry_.state;

      // Handle cancellation: send CANCEL command to remote executor
      if (is_canceling && !cancel_command_sent_) {
        auto cmd_msg = auto_apms_behavior_codec_interfaces::msg::ExecutorCommandMessage();
        cmd_msg.executor_command_message = formatExecutorCommand(ExecutorCommandType::CANCEL, "");
        command_publisher_->publish(cmd_msg);
        cancel_command_sent_ = true;
        RCLCPP_INFO(this->get_logger(), "Sent cancel command to remote executor");
      }

      // Track whether execution has been active
      if (state == ExecutionState::STARTING || state == ExecutionState::RUNNING ||
          state == ExecutionState::PAUSED || state == ExecutionState::HALTED)
      {
        seen_execution_active_ = true;
      }

      // Detect completion: state returns to IDLE after being active (or after cancel)
      if (state == ExecutionState::IDLE && (seen_execution_active_ || cancel_command_sent_)) {
        if (cancel_command_sent_) {
          cancelGoal("Execution canceled on remote executor");
        } else {
          completeGoal(StartTreeExecutor::Result::TREE_RESULT_NOT_SET, "Tree execution completed");
        }
      }
      break;
    }

    case ProxyState::IDLE:
      resetStateMachine();
      break;
  }
}

void TreeEncoderExecutorProxy::resetStateMachine()
{
  proxy_state_ = ProxyState::IDLE;
  active_goal_handle_.reset();
  expected_tree_names_.clear();
  entry_point_.clear();
  seen_execution_active_ = false;
  cancel_command_sent_ = false;
  if (state_machine_timer_) {
    state_machine_timer_->cancel();
  }
}

void TreeEncoderExecutorProxy::completeGoal(uint8_t tree_result, const std::string & message)
{
  auto result = std::make_shared<StartTreeExecutor::Result>();
  result->tree_result = tree_result;
  result->message = message;
  active_goal_handle_->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Goal completed: %s", message.c_str());
  resetStateMachine();
}

void TreeEncoderExecutorProxy::abortGoal(const std::string & message)
{
  auto result = std::make_shared<StartTreeExecutor::Result>();
  result->tree_result = StartTreeExecutor::Result::TREE_RESULT_FAILURE;
  result->message = message;
  active_goal_handle_->abort(result);
  RCLCPP_WARN(this->get_logger(), "Goal aborted: %s", message.c_str());
  resetStateMachine();
}

void TreeEncoderExecutorProxy::cancelGoal(const std::string & message)
{
  auto result = std::make_shared<StartTreeExecutor::Result>();
  result->tree_result = StartTreeExecutor::Result::TREE_RESULT_NOT_SET;
  result->message = message;
  active_goal_handle_->canceled(result);
  RCLCPP_INFO(this->get_logger(), "Goal canceled: %s", message.c_str());
  resetStateMachine();
}

void TreeEncoderExecutorProxy::telemetryCallback(
  const auto_apms_behavior_codec_interfaces::msg::SerializedTelemetryMessage::SharedPtr msg)
{
  latest_telemetry_ = ExecutorTelemetry::decode(msg->serialized_telemetry_message);
}

}  // namespace auto_apms_behavior_codec
