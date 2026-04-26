#pragma once

#include "auto_apms_behavior_codec/behavior_tree_decoder_base.hpp"
#include "auto_apms_behavior_codec/decoder_executor_client_params.hpp"
#include "auto_apms_interfaces/action/start_tree_executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace auto_apms_behavior_codec
{

class TreeDecoderExecutorClient : public BehaviorTreeDecoderBase
{
public:
  using StartTreeExecutor = auto_apms_interfaces::action::StartTreeExecutor;
  using GoalHandle = rclcpp_action::ClientGoalHandle<StartTreeExecutor>;

  explicit TreeDecoderExecutorClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void onTreeDecoded(const std::string & xml_string, const std::string & encoded_bytes_hash) override;

private:
  void goalResponseCallback(GoalHandle::SharedPtr goal_handle);
  void resultCallback(const GoalHandle::WrappedResult & result);
  void sendGoal(const std::string & xml_string);

  decoder_executor_client_params::ParamListener param_listener_;
  rclcpp_action::Client<StartTreeExecutor>::SharedPtr action_client_;
  GoalHandle::SharedPtr current_goal_handle_;
  std::string pending_xml_;
};

}  // namespace auto_apms_behavior_codec
