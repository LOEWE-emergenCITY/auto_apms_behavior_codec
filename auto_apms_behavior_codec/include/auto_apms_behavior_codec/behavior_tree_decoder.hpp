#pragma once

#include <vector>
#include <string>
#include "auto_apms_behavior_codec/dictionary_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "auto_apms_behavior_codec/behavior_tree_representation.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"

#include "auto_apms_behavior_codec_interfaces/msg/tree_xml_message.hpp"
#include "auto_apms_behavior_codec_interfaces/msg/serialized_message.hpp"

namespace auto_apms_behavior_codec
{
  class BehaviorTreeDecoder : public rclcpp::Node
  {
  public:
    // constructor takes in the topic names for the XML output and the encoded input, as well as a shared pointer to a dictionary manager
    BehaviorTreeDecoder(std::string encoded_in, std::string xml_out, std::shared_ptr<DictionaryManager> dictionary_manager);

    ~BehaviorTreeDecoder() = default;

  private:
    std::string reconstructXML(const behavior_tree_representation::Document& document);
  };
}