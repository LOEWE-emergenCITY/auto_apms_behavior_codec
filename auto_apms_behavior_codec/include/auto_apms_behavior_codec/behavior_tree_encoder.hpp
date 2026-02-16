#pragma once

#include <vector>
#include <string>
#include "auto_apms_behavior_codec/dictionary_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "auto_apms_behavior_codec/behavior_tree_representation.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"

namespace auto_apms_behavior_codec
{
  // handles encoding of behavior trees into a binary format, currently only minimal skeleton in order to be able to run a Ros node and call the dictionary manager
  class BehaviorTreeEncoder : public rclcpp::Node
  {
  public:
      BehaviorTreeEncoder();
      ~BehaviorTreeEncoder() = default;
      std::vector<uint8_t> encode(const std::string& behavior_tree_yaml);

      std::vector<uint8_t> encode(behavior_tree_representation::Document& document);

      bool readTreeDefinition(std::string tree_xml, std::unique_ptr<behavior_tree_representation::Document>& document_out);

      bool readTreeDefinitionFromDocument(std::string tree_xml, std::unique_ptr<behavior_tree_representation::Document>& document_out);

      std::string reconstructXML(const behavior_tree_representation::Document& document);

  private:
      std::shared_ptr<DictionaryManager> dictionary_manager_;

      behavior_tree_representation::Tree tree;

      behavior_tree_representation::Node getNodeFromElement(const auto_apms_behavior_tree::core::TreeDocument::NodeElement& node_element);
  };

} 