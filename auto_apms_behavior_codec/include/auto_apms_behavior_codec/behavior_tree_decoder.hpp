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
  //TODO, still untested
  class BehaviorTreeDecoder : public rclcpp::Node
  {
  public:
    // constructor takes in the topic names for the XML output and the encoded input, as well as a shared pointer to a dictionary manager
    BehaviorTreeDecoder(std::string encoded_in, std::string xml_out, std::shared_ptr<DictionaryManager> dictionary_manager);

    ~BehaviorTreeDecoder() = default;

  private:

    // reconstruct an XML string from a behavior_tree_representation::Document
    std::string reconstructXML(const behavior_tree_representation::Document& document);

    // translates a behavior_tree_representation::Tree to a TreeElement
    auto_apms_behavior_tree::core::TreeDocument::TreeElement getTreeElementFromTree(const behavior_tree_representation::Tree& tree, auto_apms_behavior_tree::core::TreeDocument& tree_doc);

    // keep a reference to the dictionary for encoding
    std::shared_ptr<DictionaryManager> dictionary_manager_;

    //subscription for incoming encoded Trees
    rclcpp::Subscription<auto_apms_behavior_codec_interfaces::msg::SerializedMessage>::SharedPtr encoded_subscription_;

    //publisher for XML behavior trees
    rclcpp::Publisher<auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage>::SharedPtr xml_publisher_;

    //callback for incoming encoded messages, handles deserialization and XML reconstruction
    void encoded_in_callback(const auto_apms_behavior_codec_interfaces::msg::SerializedMessage::SharedPtr msg);

  };
}