#pragma once

#include <memory>
#include <string>
#include <vector>

#include "auto_apms_behavior_codec/behavior_tree_representation.hpp"
#include "auto_apms_behavior_codec/dictionary_manager.hpp"
#include "auto_apms_behavior_codec/encoder_subscriber_params.hpp"
#include "auto_apms_behavior_codec_interfaces/msg/serialized_tree_message.hpp"
#include "auto_apms_behavior_codec_interfaces/msg/tree_xml_message.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_codec
{
// handles encoding of behavior trees into a binary format, receives XML encoded Behavior Trees on a configurable topic
// and sends the encoded message to a configurable output topic encoding happens based on the dictionary built from the
// node_manifest parameter
class TreeEncoderSubscriber : public rclcpp::Node
{
public:
  TreeEncoderSubscriber(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~TreeEncoderSubscriber() = default;

  std::vector<uint8_t> encode(behavior_tree_representation::Document & document);

  // reads a tree definition from a TreeDocument and converts it to the internal representation
  bool readTreeDefinitionFromDocument(
    auto_apms_behavior_tree::core::TreeDocument & tree_doc,
    std::unique_ptr<behavior_tree_representation::Document> & document_out);

  // creates a auto_apms_behavior_tree::core::TreeDocument from a XML string using the node manifests known by the
  // dictionary manager.
  bool readTreeDefinitionFromXML(
    std::string tree_xml, std::unique_ptr<behavior_tree_representation::Document> & document_out);

private:
  // parameter listener for generate_parameter_library
  encoder_subscriber_params::ParamListener param_listener_;

  // keep a reference to the dictionary for encoding
  std::shared_ptr<DictionaryManager> dictionary_manager_;

  // subscription for incoming XML encoded Trees
  rclcpp::Subscription<auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage>::SharedPtr xml_subscription_;

  // publisher for binary encoded behavior trees
  rclcpp::Publisher<auto_apms_behavior_codec_interfaces::msg::SerializedTreeMessage>::SharedPtr encoded_publisher_;

  // helaper function to get a behavior_tree_representation::Node from a TreeDocument::NodeElement, this is used for
  // constructing the internal representation of the tree from the TreeDocument API
  behavior_tree_representation::Node getNodeFromElement(
    const auto_apms_behavior_tree::core::TreeDocument::NodeElement & node_element);

  // callback for incoming XML messages, handles parsing and encoding
  void xml_in_callback(const auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage::SharedPtr msg);
};

}  // namespace auto_apms_behavior_codec