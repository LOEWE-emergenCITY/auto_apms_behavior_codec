#include "auto_apms_behavior_codec/tree_encoder_subscriber.hpp"

namespace auto_apms_behavior_codec
{

TreeEncoderSubscriber::TreeEncoderSubscriber(const rclcpp::NodeOptions & options)
: BehaviorTreeEncoderBase("behavior_tree_encoder", options), param_listener_(this)
{
  const auto params = param_listener_.get_params();

  // Set up subscription for incoming XML messages
  xml_subscription_ = this->create_subscription<auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage>(
    params.xml_in_topic, 10, std::bind(&TreeEncoderSubscriber::xml_in_callback, this, std::placeholders::_1));
}

void TreeEncoderSubscriber::xml_in_callback(
  const auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received XML message of length %zu", msg->tree_xml_message.size());
  if (!encodeAndPublish(msg->tree_xml_message)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to encode and publish XML message");
  }
}

}  // namespace auto_apms_behavior_codec

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_behavior_codec::TreeEncoderSubscriber)
