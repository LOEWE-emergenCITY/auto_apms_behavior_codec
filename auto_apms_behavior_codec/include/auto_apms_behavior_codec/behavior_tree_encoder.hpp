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

      // reads a tree definition from a TreeDocument and converts it to the internal representation
      bool readTreeDefinitionFromDocument(auto_apms_behavior_tree::core::TreeDocument& tree_doc, std::unique_ptr<behavior_tree_representation::Document>& document_out);

      // TODO: parameter for node manifest to use
      bool readTreeDefinitionFromXML(std::string tree_xml, std::unique_ptr<behavior_tree_representation::Document>& document_out){
        // Validate that we have XML content
        if (tree_xml.empty()) {
            RCLCPP_ERROR(this->get_logger(), "XML string is empty");
          return false;
        }
        RCLCPP_DEBUG(this->get_logger(), "Parsing XML of length %zu", tree_xml.length());
    
        auto_apms_behavior_tree::core::TreeDocument tree_doc;
          // Merge the XML string into the tree document
          try {
            tree_doc.mergeString(tree_xml, true);
          } catch (const std::exception & parse_error) {
            return false;
          }
          
          // this still needs correct implementation
          tree_doc.registerNodes(this->dictionary_manager_->getNodeManifests().front());

          return readTreeDefinitionFromDocument(tree_doc, document_out);
        }

      std::string reconstructXML(const behavior_tree_representation::Document& document);

  private:
      std::shared_ptr<DictionaryManager> dictionary_manager_;

      behavior_tree_representation::Tree tree;

      behavior_tree_representation::Node getNodeFromElement(const auto_apms_behavior_tree::core::TreeDocument::NodeElement& node_element);
  };

} 