#include "auto_apms_behavior_codec/behavior_tree_decoder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "auto_apms_behavior_codec/dictionary_manager.hpp"

namespace auto_apms_behavior_codec
{

BehaviorTreeDecoder::BehaviorTreeDecoder(std::string encoded_in, std::string xml_out, std::shared_ptr<DictionaryManager> dictionary_manager)
    : rclcpp::Node("behavior_tree_decoder"),
      dictionary_manager_(dictionary_manager)
{
  // Set up subscription for incoming encoded messages
  encoded_subscription_ = this->create_subscription<auto_apms_behavior_codec_interfaces::msg::SerializedMessage>(
      encoded_in, 10, std::bind(&BehaviorTreeDecoder::encoded_in_callback, this, std::placeholders::_1));

  // Set up publisher for outgoing XML messages
  xml_publisher_ = this->create_publisher<auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage>(
      xml_out, 10);
}

std::string BehaviorTreeDecoder::reconstructXML(const behavior_tree_representation::Document& document) {
  try {
    tinyxml2::XMLDocument xml_doc;
    
    // Create the root element
    tinyxml2::XMLElement* root = xml_doc.NewElement("root");
    xml_doc.InsertFirstChild(root);
    
    // Add BTCPP format and main tree attributes
    root->SetAttribute("BTCPP_format", "4");
    if (!document.main_tree_to_execute.empty()) {
      root->SetAttribute("main_tree_to_execute", document.main_tree_to_execute.c_str());
    }
    
    // Function to recursively reconstruct nodes
    std::function<tinyxml2::XMLElement*(const behavior_tree_representation::Node&)> reconstruct_node;
    reconstruct_node = [&](const behavior_tree_representation::Node& node) -> tinyxml2::XMLElement* {
      tinyxml2::XMLElement* node_element = xml_doc.NewElement(node.type_name.c_str());
      
      // Add instance name if it differs from registration name
      if (!node.instance_name.empty() && node.instance_name != node.type_name) {
        node_element->SetAttribute("name", node.instance_name.c_str());
      }
      
      // Add ports as attributes
      for (const auto& port : node.ports) {
        std::string port_value;
        
        // Determine port type and convert value to string
        if (auto port_int = std::dynamic_pointer_cast<behavior_tree_representation::PortInt>(port)) {
          port_value = std::to_string(port_int->value);
        } else if (auto port_float = std::dynamic_pointer_cast<behavior_tree_representation::PortFloat>(port)) {
          port_value = std::to_string(port_float->value);
        } else if (auto port_bool = std::dynamic_pointer_cast<behavior_tree_representation::PortBool>(port)) {
          port_value = port_bool->value ? "true" : "false";
        } else if (auto port_string = std::dynamic_pointer_cast<behavior_tree_representation::PortString>(port)) {
          port_value = port_string->value;
        } else if (auto port_any = std::dynamic_pointer_cast<behavior_tree_representation::PortAnyTypeAllowed>(port)) {
          //TODO handle AnyTypeAllowed
        }
        
        // Find port name from dictionary
        DictionaryNode dict_node = this->dictionary_manager_->get_dictionary_info_by_name(node.type_name);
        if (port->getID() < dict_node.port_types.size()) {
          const auto& port_info = dict_node.port_types[port->getID()];
          node_element->SetAttribute(port_info.name.c_str(), port_value.c_str());
        }
      }
      
      // Recursively add child nodes
      for (const auto& child : node.children) {
        tinyxml2::XMLElement* child_element = reconstruct_node(*child);
        node_element->InsertEndChild(child_element);
      }
      
      return node_element;
    };
    
    // Reconstruct all trees
    for (const auto& tree : document.trees) {
      tinyxml2::XMLElement* tree_element = xml_doc.NewElement("BehaviorTree");
      tree_element->SetAttribute("ID", tree.name.c_str());
      
      // Add the root node of the tree as a child
      if (!tree.root.type_name.empty()) {
        tinyxml2::XMLElement* root_node = reconstruct_node(tree.root);
        tree_element->InsertEndChild(root_node);
      }
      
      root->InsertEndChild(tree_element);
    }
    
    // Convert to string
    tinyxml2::XMLPrinter printer;
    xml_doc.Print(&printer);
    
    RCLCPP_INFO(this->get_logger(), "Successfully reconstructed XML from document");
    return printer.CStr();
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to reconstruct XML: %s", e.what());
    return "";
  }
}

} // namespace auto_apms_behavior_codec