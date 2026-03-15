#include "auto_apms_behavior_codec/tree_decoder_executor_client.hpp"

namespace auto_apms_behavior_codec
{

TreeDecoderExecutorClient::TreeDecoderExecutorClient(const rclcpp::NodeOptions & options)
: BehaviorTreeDecoderBase("tree_decoder_executor_client", options), param_listener_(this)
{
  const auto params = param_listener_.get_params();
  action_client_ = rclcpp_action::create_client<StartTreeExecutor>(this, params.start_tree_executor_action_name);
  RCLCPP_INFO(this->get_logger(), "Action client created for '%s'", params.start_tree_executor_action_name.c_str());
}

void TreeDecoderExecutorClient::onTreeDecoded(const std::string & xml_string)
{
  if (!action_client_->action_server_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "Action server not available, skipping tree execution");
    return;
  }

  // Cancel any currently running goal
  if (current_goal_handle_) {
    RCLCPP_INFO(this->get_logger(), "Canceling previous goal before sending new one");
    action_client_->async_cancel_goal(current_goal_handle_);
    current_goal_handle_ = nullptr;
  }

  // Send new goal with the decoded XML
  auto goal_msg = StartTreeExecutor::Goal();
  goal_msg.build_request = xml_string;
  goal_msg.node_manifest = getNodeManifest().encode();

  auto send_goal_options = rclcpp_action::Client<StartTreeExecutor>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&TreeDecoderExecutorClient::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&TreeDecoderExecutorClient::resultCallback, this, std::placeholders::_1);

  RCLCPP_INFO(this->get_logger(), "Sending new tree execution goal");
  action_client_->async_send_goal(goal_msg, send_goal_options);
}

void TreeDecoderExecutorClient::goalResponseCallback(GoalHandle::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the tree executor");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Goal accepted by the tree executor");
  current_goal_handle_ = goal_handle;
}

void TreeDecoderExecutorClient::resultCallback(const GoalHandle::WrappedResult & result)
{
  current_goal_handle_ = nullptr;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(
        this->get_logger(), "Tree execution succeeded (result=%u): %s", result.result->tree_result,
        result.result->message.c_str());
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(this->get_logger(), "Tree execution aborted: %s", result.result->message.c_str());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Tree execution canceled: %s", result.result->message.c_str());
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
}

}  // namespace auto_apms_behavior_codec

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auto_apms_behavior_codec::TreeDecoderExecutorClient>());
  rclcpp::shutdown();
  return 0;
}
