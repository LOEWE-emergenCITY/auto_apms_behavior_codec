#include "auto_apms_behavior_codec/tree_decoder_executor.hpp"

#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"

namespace auto_apms_behavior_codec
{

TreeDecoderExecutor::TreeDecoderExecutor(const rclcpp::NodeOptions & options)
: BehaviorTreeDecoderBase("tree_decoder_executor", options), param_listener_(this)
{
  // Defer executor initialization to after construction so shared_from_this() is valid.
  init_timer_ = this->create_wall_timer(std::chrono::nanoseconds(0), [this]() {
    init_timer_->cancel();
    setupExecutorAndCodecInterfaces();
  });
}

void TreeDecoderExecutor::setupExecutorAndCodecInterfaces()
{
  const auto params = param_listener_.get_params();

  // Using a one-shot timer, it is safe to call shared_from_this() — the owning shared_ptr is established.
  executor_ = std::make_unique<auto_apms_behavior_tree::GenericTreeExecutorNode>(
    this->shared_from_this(), auto_apms_behavior_tree::TreeExecutorNodeOptions()
                                .enableCommandAction(false)
                                .enableClearBlackboardService(false)
                                .setDefaultBuildHandler("auto_apms_behavior_tree::TreeFromStringBuildHandler"));

  // Subscribe to executor commands
  command_subscription_ = this->create_subscription<auto_apms_behavior_codec_interfaces::msg::ExecutorCommandMessage>(
    params.executor_command_in_topic, 10,
    std::bind(&TreeDecoderExecutor::commandCallback, this, std::placeholders::_1));

  // Publisher for feedback
  feedback_publisher_ = this->create_publisher<auto_apms_behavior_codec_interfaces::msg::ExecutorFeedbackMessage>(
    params.feedback_out_topic, 10);

  // Poll for state changes at the configured feedback rate; only publishes when state actually changes.
  const auto poll_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / params.feedback_rate));
  state_feedback_timer_ = this->create_wall_timer(
    poll_period, std::bind(&TreeDecoderExecutor::publishStateFeedback, this));

  // Prepare buffer tree document
  decoding_verification_doc_.registerNodes(getNodeManifest());
  behavior_library_doc_.registerNodes(getNodeManifest());

  RCLCPP_INFO(
    this->get_logger(), "Codec interfaces: command='%s', feedback='%s' (on state change)",
    params.executor_command_in_topic.c_str(), params.feedback_out_topic.c_str());
}

void TreeDecoderExecutor::onTreeDecoded(const std::string & xml_string, const std::string & encoded_bytes_hash)
{
  RCLCPP_INFO(this->get_logger(), "Tree decoded (%zu bytes XML)", xml_string.size());

  // Parse decoded XML into a temporary document to verify it's well-formed
  try {
    decoding_verification_doc_.mergeString(xml_string, false);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse decoded tree XML: %s", e.what());
    return;
  }

  // Verify the tree(s) can be run
  if (const auto res = decoding_verification_doc_.verify(); !res) {
    RCLCPP_ERROR(this->get_logger(), "Decoded tree XML failed verification: %s", res.error().c_str());
    return;
  }

  // Acknowledge successful registration using the hash of the received encoded bytes,
  // which matches the hash computed by the proxy from pending_encoded_data_.
  const std::string & hash = encoded_bytes_hash;

  // Store tree(s) in buffer document (behavior knowledge base for the executor)
  // Replace any existing trees with the same names, but keep previously registered trees that are not overwritten by
  // this new document.
  for (const auto & tree_name : decoding_verification_doc_.getAllTreeNames()) {
    const auto tree = decoding_verification_doc_.getTree(tree_name);
    if (behavior_library_doc_.hasTreeName(tree_name)) {
      RCLCPP_INFO(this->get_logger(), "Overwriting previously registered tree '%s' in behavior library", tree_name.c_str());
      behavior_library_doc_.removeTree(tree_name);
    } else {
      RCLCPP_INFO(this->get_logger(), "Registering new tree '%s' in behavior library", tree_name.c_str());
    }
    behavior_library_doc_.mergeTree(tree, false, false);
  }

  // Reset the decoding verification document to free memory; it will be reused for the next decoded tree.
  decoding_verification_doc_.reset(false);

  // Send REGISTERACK with the hash of the encoded bytes to acknowledge successful receipt and decoding of the tree.
  RCLCPP_INFO(this->get_logger(), "Sending REGISTERACK (hash=%s)", hash.c_str());
  auto ack_msg = auto_apms_behavior_codec_interfaces::msg::ExecutorFeedbackMessage();
  ack_msg.executor_feedback_message = ExecutorFeedback::makeRegisterAck(hash).encode();
  feedback_publisher_->publish(ack_msg);
}

void TreeDecoderExecutor::commandCallback(
  const auto_apms_behavior_codec_interfaces::msg::ExecutorCommandMessage::SharedPtr msg)
{
  const auto & cmd_str = msg->executor_command_message;
  RCLCPP_INFO(this->get_logger(), "Received command: '%s'", cmd_str.c_str());

  const auto cmd = ExecutorCommand::decode(cmd_str);

  using ControlCommand = auto_apms_behavior_tree::TreeExecutorBase::ControlCommand;

  switch (cmd.type) {
    case ExecutorCommandType::START: {
      if (behavior_library_doc_.getAllTreeNames().empty()) {
        RCLCPP_WARN(this->get_logger(), "No tree registered; ignoring START command");
        return;
      }
      if (executor_->isBusy()) {
        RCLCPP_WARN(this->get_logger(), "Executor is busy; ignoring START command");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Starting tree execution (entry_point='%s')", cmd.payload.c_str());
      executor_->startExecution(behavior_library_doc_.writeToString(), cmd.payload, getNodeManifest());
      break;
    }
    case ExecutorCommandType::CANCEL: {
      if (!executor_->isBusy()) {
        RCLCPP_WARN(this->get_logger(), "Executor not busy; ignoring CANCEL command");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Canceling tree execution");
      executor_->setControlCommand(ControlCommand::TERMINATE);
      break;
    }
    case ExecutorCommandType::PAUSE: {
      if (executor_->isBusy()) executor_->setControlCommand(ControlCommand::PAUSE);
      break;
    }
    case ExecutorCommandType::RESUME: {
      if (executor_->isBusy()) executor_->setControlCommand(ControlCommand::RUN);
      break;
    }
    case ExecutorCommandType::STOP: {
      if (executor_->isBusy()) executor_->setControlCommand(ControlCommand::HALT);
      break;
    }
    default:
      RCLCPP_WARN(this->get_logger(), "Unknown or invalid command: '%s'", cmd_str.c_str());
      break;
  }
}

void TreeDecoderExecutor::publishStateFeedback()
{
  const auto current_state = executor_->getExecutionState();
  if (current_state == last_published_state_) return;
  last_published_state_ = current_state;
  auto msg = auto_apms_behavior_codec_interfaces::msg::ExecutorFeedbackMessage();
  msg.executor_feedback_message = ExecutorFeedback::makeState(current_state).encode();
  feedback_publisher_->publish(msg);
}

}  // namespace auto_apms_behavior_codec

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_behavior_codec::TreeDecoderExecutor)
