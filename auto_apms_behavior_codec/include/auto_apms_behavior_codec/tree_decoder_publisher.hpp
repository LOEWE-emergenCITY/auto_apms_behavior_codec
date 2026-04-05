#pragma once

#include "auto_apms_behavior_codec/behavior_tree_decoder_base.hpp"
#include "auto_apms_behavior_codec/decoder_publisher_params.hpp"
#include "auto_apms_behavior_codec_interfaces/msg/tree_xml_message.hpp"

namespace auto_apms_behavior_codec
{

class TreeDecoderPublisher : public BehaviorTreeDecoderBase
{
public:
  explicit TreeDecoderPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void onTreeDecoded(const std::string & xml_string) override;

private:
  decoder_publisher_params::ParamListener param_listener_;
  rclcpp::Publisher<auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage>::SharedPtr xml_publisher_;
};

}  // namespace auto_apms_behavior_codec
