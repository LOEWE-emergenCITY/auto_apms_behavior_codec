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
    this->shared_from_this(),
    auto_apms_behavior_tree::TreeExecutorNodeOptions()
      .enableCommandAction(false)
      .enableClearBlackboardService(false)
      .setDefaultBuildHandler("auto_apms_behavior_tree::TreeFromStringBuildHandler"));

  // Subscribe to executor commands
  command_subscription_ = this->create_subscription<auto_apms_behavior_codec_interfaces::msg::ExecutorCommandMessage>(
    params.executor_command_topic, 10, std::bind(&TreeDecoderExecutor::commandCallback, this, std::placeholders::_1));

  // Publisher for telemetry
  telemetry_publisher_ = this->create_publisher<auto_apms_behavior_codec_interfaces::msg::SerializedTelemetryMessage>(
    params.telemetry_topic, 10);

  // Periodic telemetry publication
  const double rate_hz = params.telemetry_rate;
  const auto period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate_hz));
  telemetry_timer_ = this->create_wall_timer(period, std::bind(&TreeDecoderExecutor::publishTelemetry, this));

  // Prepare buffer tree document
  decoding_verification_doc_.registerNodes(getNodeManifest());
  behavior_library_doc_.registerNodes(getNodeManifest());

  RCLCPP_INFO(
    this->get_logger(), "Codec interfaces: command='%s', telemetry_out='%s' (%.1f Hz)",
    params.executor_command_topic.c_str(), params.telemetry_topic.c_str(), rate_hz);
}

void TreeDecoderExecutor::onTreeDecoded(const std::string & xml_string)
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

  // Store tree(s) in buffer document (behavior knowledge base for the executor)
  behavior_library_doc_.mergeTreeDocument(decoding_verification_doc_, false);
  decoding_verification_doc_.reset(false);
}

void TreeDecoderExecutor::commandCallback(
  const auto_apms_behavior_codec_interfaces::msg::ExecutorCommandMessage::SharedPtr msg)
{
  const auto & cmd_str = msg->executor_command_message;
  RCLCPP_INFO(this->get_logger(), "Received command: '%s'", cmd_str.c_str());

  // Parse "TYPE:payload" format
  auto colon_pos = cmd_str.find(':');
  if (colon_pos == std::string::npos) {
    RCLCPP_WARN(this->get_logger(), "Invalid command format (missing ':'): '%s'", cmd_str.c_str());
    return;
  }

  std::string type_str = cmd_str.substr(0, colon_pos);
  std::string payload = cmd_str.substr(colon_pos + 1);

  using ControlCommand = auto_apms_behavior_tree::TreeExecutorBase::ControlCommand;

  if (type_str == executorCommandTypeToString(ExecutorCommandType::START)) {
    if (behavior_library_doc_.getAllTreeNames().empty()) {
      RCLCPP_WARN(this->get_logger(), "No tree registered; ignoring START command");
      return;
    }
    if (executor_->isBusy()) {
      RCLCPP_WARN(this->get_logger(), "Executor is busy; ignoring START command");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Starting tree execution (entry_point='%s')", payload.c_str());
    executor_->startExecution(behavior_library_doc_.writeToString(), payload, getNodeManifest());

  } else if (type_str == executorCommandTypeToString(ExecutorCommandType::CANCEL)) {
    if (!executor_->isBusy()) {
      RCLCPP_WARN(this->get_logger(), "Executor not busy; ignoring CANCEL command");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Canceling tree execution");
    executor_->setControlCommand(ControlCommand::TERMINATE);

  } else if (type_str == executorCommandTypeToString(ExecutorCommandType::PAUSE)) {
    if (!executor_->isBusy()) return;
    executor_->setControlCommand(ControlCommand::PAUSE);

  } else if (type_str == executorCommandTypeToString(ExecutorCommandType::RESUME)) {
    if (!executor_->isBusy()) return;
    executor_->setControlCommand(ControlCommand::RUN);

  } else if (type_str == executorCommandTypeToString(ExecutorCommandType::STOP)) {
    if (!executor_->isBusy()) return;
    executor_->setControlCommand(ControlCommand::HALT);

  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown command type: '%s'", type_str.c_str());
  }
}

void TreeDecoderExecutor::publishTelemetry()
{
  ExecutorTelemetry telemetry;
  telemetry.state = executor_->getExecutionState();
  telemetry.behaviors = behavior_library_doc_.getAllTreeNames();

  auto msg = auto_apms_behavior_codec_interfaces::msg::SerializedTelemetryMessage();
  msg.serialized_telemetry_message = telemetry.encode();
  telemetry_publisher_->publish(msg);
}

}  // namespace auto_apms_behavior_codec

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_behavior_codec::TreeDecoderExecutor)
