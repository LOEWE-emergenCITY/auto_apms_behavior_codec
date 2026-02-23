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
  // handles encoding of behavior trees into a binary format, receives XML encoded Behavior Trees on Topic passed in the constructor as "xml_in" and sends the encoded message to the topic passed as "encoded_out"
  // encoding happens based on the dictionary passed in the constructor
  class BehaviorTreeEncoder : public rclcpp::Node
  {
  public:
      // constructor takes in the topic names for the XML input and the encoded output, as well as a shared pointer to a dictionary manager
      BehaviorTreeEncoder(std::string xml_in, std::string encoded_out, std::shared_ptr<DictionaryManager> dictionary_manager);

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

      

  private:
      // keep a reference to the dictionary for encoding
      std::shared_ptr<DictionaryManager> dictionary_manager_;

      //subscription for incoming XML encoded Trees
      rclcpp::Subscription<auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage>::SharedPtr xml_subscription_;

      //publisher for binary encoded behavior trees
      rclcpp::Publisher<auto_apms_behavior_codec_interfaces::msg::SerializedMessage>::SharedPtr encoded_publisher_;

      behavior_tree_representation::Node getNodeFromElement(const auto_apms_behavior_tree::core::TreeDocument::NodeElement& node_element);

      void xml_in_callback(const auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage::SharedPtr msg);
  };

}