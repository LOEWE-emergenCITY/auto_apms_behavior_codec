#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "auto_apms_behavior_codec/executor_command.hpp"
#include "auto_apms_behavior_codec/executor_feedback.hpp"
#include "auto_apms_behavior_codec/tree_encoder_executor_proxy.hpp"
#include "auto_apms_behavior_codec_interfaces/msg/executor_command_message.hpp"
#include "auto_apms_behavior_codec_interfaces/msg/executor_feedback_message.hpp"
#include "auto_apms_interfaces/action/start_tree_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using auto_apms_behavior_codec::ExecutionState;
using auto_apms_behavior_codec::ExecutorFeedback;
using StartTreeExecutor = auto_apms_interfaces::action::StartTreeExecutor;
using GoalHandleClient = rclcpp_action::ClientGoalHandle<StartTreeExecutor>;

class TreeEncoderExecutorProxyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    proxy_node_ = std::make_shared<auto_apms_behavior_codec::TreeEncoderExecutorProxy>();

    helper_node_ = rclcpp::Node::make_shared("test_helper");

    action_client_ = rclcpp_action::create_client<StartTreeExecutor>(helper_node_, "start_tree_executor");

    feedback_publisher_ =
      helper_node_->create_publisher<auto_apms_behavior_codec_interfaces::msg::ExecutorFeedbackMessage>(
        "executor_feedback_in", 10);

    received_commands_.clear();
    command_subscription_ =
      helper_node_->create_subscription<auto_apms_behavior_codec_interfaces::msg::ExecutorCommandMessage>(
        "executor_command", 10,
        [this](const auto_apms_behavior_codec_interfaces::msg::ExecutorCommandMessage::SharedPtr msg) {
          received_commands_.push_back(msg->executor_command_message);
        });

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(proxy_node_);
    executor_->add_node(helper_node_);
  }

  void TearDown() override
  {
    executor_->cancel();
    action_client_.reset();
    helper_node_.reset();
    proxy_node_.reset();
  }

  void publishStateFeedback(ExecutionState state)
  {
    auto msg = auto_apms_behavior_codec_interfaces::msg::ExecutorFeedbackMessage();
    msg.executor_feedback_message = ExecutorFeedback::makeState(state).encode();
    feedback_publisher_->publish(msg);
  }

  std::shared_future<GoalHandleClient::SharedPtr> sendGoal(
    const std::string & build_request, const std::string & build_handler, const std::string & entry_point,
    const std::string & node_manifest = "")
  {
    auto goal = StartTreeExecutor::Goal();
    goal.build_request = build_request;
    goal.build_handler = build_handler;
    goal.entry_point = entry_point;
    goal.node_manifest = node_manifest;
    return action_client_->async_send_goal(goal);
  }

  // Spin executor for the given duration
  void spinFor(std::chrono::milliseconds duration)
  {
    auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some(10ms);
      std::this_thread::sleep_for(1ms);
    }
  }

  // Spin until a future is ready or timeout
  template <typename FutureT>
  bool spinUntilReady(FutureT & future, std::chrono::seconds timeout = 10s)
  {
    auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some(10ms);
      if (future.wait_for(1ms) == std::future_status::ready) {
        return true;
      }
    }
    return false;
  }

  // Spin until a predicate is true or timeout
  bool spinUntil(std::function<bool()> predicate, std::chrono::seconds timeout = 10s)
  {
    auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some(10ms);
      if (predicate()) return true;
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  bool waitForActionServer()
  {
    return spinUntil([this]() { return action_client_->action_server_is_ready(); }, 10s);
  }

  std::shared_ptr<auto_apms_behavior_codec::TreeEncoderExecutorProxy> proxy_node_;
  rclcpp::Node::SharedPtr helper_node_;
  rclcpp_action::Client<StartTreeExecutor>::SharedPtr action_client_;
  rclcpp::Publisher<auto_apms_behavior_codec_interfaces::msg::ExecutorFeedbackMessage>::SharedPtr
    feedback_publisher_;
  rclcpp::Subscription<auto_apms_behavior_codec_interfaces::msg::ExecutorCommandMessage>::SharedPtr
    command_subscription_;
  std::vector<std::string> received_commands_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
};

TEST_F(TreeEncoderExecutorProxyTest, RejectGoalEmptyBuildRequestAndEntryPoint)
{
  ASSERT_TRUE(waitForActionServer()) << "Action server did not become available";

  auto goal_future = sendGoal("", "", "");
  ASSERT_TRUE(spinUntilReady(goal_future, 5s)) << "Timed out waiting for goal response";
  auto goal_handle = goal_future.get();
  EXPECT_EQ(goal_handle, nullptr);
}

TEST_F(TreeEncoderExecutorProxyTest, RejectGoalUnsupportedBuildHandler)
{
  ASSERT_TRUE(waitForActionServer());

  auto goal_future = sendGoal("<root/>", "wrong::BuildHandler", "main_tree");
  ASSERT_TRUE(spinUntilReady(goal_future, 5s));
  auto goal_handle = goal_future.get();
  EXPECT_EQ(goal_handle, nullptr);
}

TEST_F(TreeEncoderExecutorProxyTest, SkipEncodingSendsStartCommand)
{
  ASSERT_TRUE(waitForActionServer());

  publishStateFeedback(ExecutionState::IDLE);
  spinFor(200ms);

  auto goal_future = sendGoal("", "", "my_tree");
  ASSERT_TRUE(spinUntilReady(goal_future, 5s));
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr) << "Goal should be accepted";

  // Wait for the START command
  ASSERT_TRUE(spinUntil(
    [this]() {
      for (const auto & cmd : received_commands_) {
        if (cmd == "START:my_tree") return true;
      }
      return false;
    },
    5s))
    << "Expected START:my_tree command";

  // Complete the goal so proxy returns to IDLE
  publishStateFeedback(ExecutionState::RUNNING);
  spinFor(200ms);
  publishStateFeedback(ExecutionState::IDLE);

  auto result_future = action_client_->async_get_result(goal_handle);
  ASSERT_TRUE(spinUntilReady(result_future, 5s));
}

TEST_F(TreeEncoderExecutorProxyTest, FullStateMachineExecution)
{
  ASSERT_TRUE(waitForActionServer());

  publishStateFeedback(ExecutionState::IDLE);
  spinFor(200ms);

  auto goal_future = sendGoal("", "", "my_tree");
  ASSERT_TRUE(spinUntilReady(goal_future, 5s));
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  // Wait for START command
  ASSERT_TRUE(spinUntil(
    [this]() {
      for (const auto & cmd : received_commands_) {
        if (cmd == "START:my_tree") return true;
      }
      return false;
    },
    5s));

  // Simulate executor state transitions
  publishStateFeedback(ExecutionState::STARTING);
  spinFor(200ms);
  publishStateFeedback(ExecutionState::RUNNING);
  spinFor(200ms);

  // Execution completes
  publishStateFeedback(ExecutionState::IDLE);

  auto result_future = action_client_->async_get_result(goal_handle);
  ASSERT_TRUE(spinUntilReady(result_future, 10s)) << "Timed out waiting for result";

  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(wrapped_result.result->tree_result, StartTreeExecutor::Result::TREE_RESULT_NOT_SET);
}

TEST_F(TreeEncoderExecutorProxyTest, CancelDuringExecution)
{
  ASSERT_TRUE(waitForActionServer());

  publishStateFeedback(ExecutionState::IDLE);
  spinFor(200ms);

  auto goal_future = sendGoal("", "", "my_tree");
  ASSERT_TRUE(spinUntilReady(goal_future, 5s));
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  // Wait for state machine to send START and executor to be running
  spinFor(500ms);
  publishStateFeedback(ExecutionState::RUNNING);
  spinFor(200ms);

  // Request cancellation
  auto cancel_future = action_client_->async_cancel_goal(goal_handle);
  ASSERT_TRUE(spinUntilReady(cancel_future, 5s)) << "Timed out waiting for cancel response";

  // Wait for CANCEL command
  ASSERT_TRUE(spinUntil(
    [this]() {
      for (const auto & cmd : received_commands_) {
        if (cmd == "CANCEL:") return true;
      }
      return false;
    },
    5s))
    << "Expected CANCEL command";

  // Simulate executor returning to IDLE after cancel
  publishStateFeedback(ExecutionState::IDLE);

  auto result_future = action_client_->async_get_result(goal_handle);
  ASSERT_TRUE(spinUntilReady(result_future, 10s)) << "Timed out waiting for cancelled result";

  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(TreeEncoderExecutorProxyTest, RejectSecondGoalWhileBusy)
{
  ASSERT_TRUE(waitForActionServer());

  publishStateFeedback(ExecutionState::IDLE);
  spinFor(200ms);

  auto goal1_future = sendGoal("", "", "tree_1");
  ASSERT_TRUE(spinUntilReady(goal1_future, 5s));
  auto goal1_handle = goal1_future.get();
  ASSERT_NE(goal1_handle, nullptr);

  spinFor(200ms);

  // Second goal should be rejected
  auto goal2_future = sendGoal("", "", "tree_2");
  ASSERT_TRUE(spinUntilReady(goal2_future, 5s));
  auto goal2_handle = goal2_future.get();
  EXPECT_EQ(goal2_handle, nullptr) << "Second goal should be rejected while proxy is busy";

  // Clean up first goal
  publishStateFeedback(ExecutionState::RUNNING);
  spinFor(200ms);
  publishStateFeedback(ExecutionState::IDLE);

  auto result_future = action_client_->async_get_result(goal1_handle);
  spinUntilReady(result_future, 5s);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
