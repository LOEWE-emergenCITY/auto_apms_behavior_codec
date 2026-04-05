#pragma once

#include "auto_apms_behavior_codec/behavior_tree_encoder_base.hpp"
#include "auto_apms_behavior_codec/encoder_subscriber_params.hpp"
#include "auto_apms_behavior_codec_interfaces/msg/tree_xml_message.hpp"

namespace auto_apms_behavior_codec
{

// Receives XML encoded Behavior Trees on a configurable topic and publishes encoded messages.
class TreeEncoderSubscriber : public BehaviorTreeEncoderBase
{
public:
  explicit TreeEncoderSubscriber(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // callback for incoming XML messages, handles parsing and encoding
  void xml_in_callback(const auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage::SharedPtr msg);

  encoder_subscriber_params::ParamListener param_listener_;
  rclcpp::Subscription<auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage>::SharedPtr xml_subscription_;
};

}  // namespace auto_apms_behavior_codec