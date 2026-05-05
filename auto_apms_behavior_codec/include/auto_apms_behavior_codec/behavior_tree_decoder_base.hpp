#pragma once

#include <memory>
#include <string>
#include <vector>

#include "auto_apms_behavior_codec/behavior_tree_representation.hpp"
#include "auto_apms_behavior_codec/decoder_base_params.hpp"
#include "auto_apms_behavior_codec/dictionary_manager.hpp"
#include "auto_apms_behavior_codec_interfaces/msg/serialized_tree_message.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_codec
{

class BehaviorTreeDecoderBase : public rclcpp::Node
{
public:
  BehaviorTreeDecoderBase(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~BehaviorTreeDecoderBase() override = default;

  /// Access the node manifest built from the base parameters.
  auto_apms_behavior_tree::core::NodeManifest getNodeManifest() const;

  /// Reconstruct XML from an internal Document representation.
  std::string reconstructXML(const behavior_tree_representation::Document & document);

protected:
  /// Called when a tree has been successfully decoded to XML.
  /// \param xml_string  The reconstructed XML.
  /// \param encoded_bytes_hash  FNV-1a hash of the raw encoded bytes (use for ACK correlation).
  virtual void onTreeDecoded(const std::string & xml_string, const std::string & encoded_bytes_hash) = 0;

private:
  void setupDecoder();

  // adds the tree described by the passed behavior_tree_representation::Tree to the TreeDocument and returns the tree element which was added
  auto_apms_behavior_tree::core::TreeDocument::TreeElement getTreeElementFromTree(
    const behavior_tree_representation::Tree & tree, auto_apms_behavior_tree::core::TreeDocument & tree_doc);

  void encodedInCallback(const auto_apms_behavior_codec_interfaces::msg::SerializedTreeMessage::SharedPtr msg);

  decoder_base_params::ParamListener param_listener_;
  std::shared_ptr<DictionaryManager> dictionary_manager_;
  rclcpp::Subscription<auto_apms_behavior_codec_interfaces::msg::SerializedTreeMessage>::SharedPtr
    encoded_subscription_;
};

}  // namespace auto_apms_behavior_codec
