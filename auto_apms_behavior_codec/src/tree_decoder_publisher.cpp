#include "auto_apms_behavior_codec/tree_decoder_publisher.hpp"

namespace auto_apms_behavior_codec
{

TreeDecoderPublisher::TreeDecoderPublisher(const rclcpp::NodeOptions & options)
: BehaviorTreeDecoderBase("tree_decoder_publisher", options), param_listener_(this)
{
  const auto params = param_listener_.get_params();
  xml_publisher_ =
    this->create_publisher<auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage>(params.xml_out_topic, 10);
}

void TreeDecoderPublisher::onTreeDecoded(const std::string & xml_string)
{
  auto xml_msg = auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage();
  xml_msg.tree_xml_message = xml_string;
  xml_publisher_->publish(xml_msg);
  RCLCPP_INFO(this->get_logger(), "Published reconstructed XML message");
}

}  // namespace auto_apms_behavior_codec

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_behavior_codec::TreeDecoderPublisher)
