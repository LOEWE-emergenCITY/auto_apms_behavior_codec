#include "auto_apms_behavior_codec/behavior_tree_decoder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "auto_apms_behavior_tree_core/node/node_model_type.hpp"
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

//TODO change to TreeDocument API
std::string BehaviorTreeDecoder::reconstructXML(const behavior_tree_representation::Document& document) {
  auto_apms_behavior_tree::core::TreeDocument tree_doc;
  tree_doc.registerNodes(this->dictionary_manager_->getNodeManifest());
  for (const auto& tree : document.trees) {
    RCLCPP_DEBUG(this->get_logger(), "Processing tree '%s' for XML reconstruction", tree.name.c_str());
    if(tree.name == document.main_tree_to_execute){
      getTreeElementFromTree(tree, tree_doc).makeRoot();
    }
    else{
      getTreeElementFromTree(tree, tree_doc);
    }
    RCLCPP_DEBUG(this->get_logger(), "Finished processing tree '%s'", tree.name.c_str());
    RCLCPP_DEBUG(this->get_logger(), "Current state of TreeDocument after processing tree '%s':\n%s", tree.name.c_str(), tree_doc.writeToString().c_str());
  }

  return tree_doc.writeToString();
}

std::map<std::string, std::string> get_port_map(
  const behavior_tree_representation::Node& node,
  std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dictionary_manager) {
  std::map<std::string, std::string> port_map;
  
  // Get port info from dictionary
  auto dict_node = dictionary_manager->get_dictionary_info_by_name(node.type_name);
  
  for (const auto& port : node.ports) {
    if (!port) {
      RCLCPP_WARN(rclcpp::get_logger("behavior_tree_decoder"), "Null port pointer detected in node '%s'", node.type_name.c_str());
      continue;
    }
    
    std::string port_name;
    std::string port_value;
    uint16_t port_id = port->getID();
    
    // Try to get port name from dictionary using port ID as index
    if (port_id < dict_node.port_types.size()) {
      port_name = dict_node.port_types[port_id].name;
      RCLCPP_DEBUG(rclcpp::get_logger("behavior_tree_decoder"), "Port ID %u maps to name '%s' for node '%s'", port_id, port_name.c_str(), node.type_name.c_str());
    } else {
      RCLCPP_WARN(rclcpp::get_logger("behavior_tree_decoder"), "Port ID %u out of range for node '%s' (has %zu ports), disregard this warning for SubTrees", port_id, node.type_name.c_str(), dict_node.port_types.size());
      continue;
    }
    
    // Extract port value based on type
    if (auto port_string = std::dynamic_pointer_cast<behavior_tree_representation::PortString>(port)) {
      port_value = port_string->value;
    } else if (auto port_int = std::dynamic_pointer_cast<behavior_tree_representation::PortInt>(port)) {
      port_value = std::to_string(port_int->value);
    } else if (auto port_uint = std::dynamic_pointer_cast<behavior_tree_representation::PortUInt>(port)) {
      port_value = std::to_string(port_uint->value);
    } else if (auto port_float = std::dynamic_pointer_cast<behavior_tree_representation::PortFloat>(port)) {
      port_value = std::to_string(port_float->value);
    } else if (auto port_double = std::dynamic_pointer_cast<behavior_tree_representation::PortDouble>(port)) {
      port_value = std::to_string(port_double->value);
    } else if (auto port_bool = std::dynamic_pointer_cast<behavior_tree_representation::PortBool>(port)) {
      port_value = port_bool->value ? "true" : "false";
    } else if (auto port_any = std::dynamic_pointer_cast<behavior_tree_representation::PortAnyTypeAllowed>(port)) {
      port_value = port_any->value;
    } else if (auto port_subtree = std::dynamic_pointer_cast<behavior_tree_representation::PortSubTreeSpecial>(port)) {
      port_name = port_subtree->name;
      port_value = port_subtree->value;
      RCLCPP_DEBUG(rclcpp::get_logger("behavior_tree_decoder"), "Handled SubTreeSpecial port: name='%s', value='%s'", port_subtree->name.c_str(), port_subtree->value.c_str());
    } else if (auto port_invalid = std::dynamic_pointer_cast<behavior_tree_representation::PortInvalid>(port)) {
      // Invalid port: value is already the blackboard key string (e.g. "{z}"), port_name from dictionary
      port_value = port_invalid->value;
      RCLCPP_DEBUG(rclcpp::get_logger("behavior_tree_decoder"), "Handled Invalid port: name='%s', value='%s'", port_name.c_str(), port_value.c_str());
    } else {
      RCLCPP_WARN(rclcpp::get_logger("behavior_tree_decoder"), "Unknown port type in node '%s'", node.type_name.c_str());
      continue;
    }
    
    port_map[port_name] = port_value;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("behavior_tree_decoder"), "Port map for node '%s' has %zu entries", node.type_name.c_str(), port_map.size());
  return port_map;
};

auto_apms_behavior_tree::core::TreeDocument::NodeElement recursiveNodeConstruction(
  const behavior_tree_representation::Node& node,
  auto_apms_behavior_tree::core::TreeDocument::NodeElement& base_node,
  std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dictionary_manager)
{
  RCLCPP_DEBUG(rclcpp::get_logger("behavior_tree_decoder"), "Constructing node '%s'", node.type_name.c_str());
  if(node.type_name != "SubTree"){
    // add ports to the node element - bypass model validation by setting attributes directly
    auto port_map = get_port_map(node, dictionary_manager);
    auto xml_element = base_node.getXMLElement();
    for (const auto& [port_name, port_value] : port_map) {
      xml_element->SetAttribute(port_name.c_str(), port_value.c_str());
      RCLCPP_DEBUG(rclcpp::get_logger("behavior_tree_decoder"), "Set port attribute '%s'='%s' on node '%s'", port_name.c_str(), port_value.c_str(), node.type_name.c_str());
    }
  }
  else{
    RCLCPP_DEBUG(rclcpp::get_logger("behavior_tree_decoder"), "Node '%s' is a SubTree, Handeling Ports Specially...", node.type_name.c_str());
    auto port_map = get_port_map(node, dictionary_manager);
    // Wrap blackboard remapping values in curly brackets
    std::map<std::string, std::string> blackboard_remapping;
    for (const auto& [key, value] : port_map) {
      blackboard_remapping[key] = "{" + value + "}";
    }
    static_cast<auto_apms_behavior_tree::model::SubTree&>(base_node).setBlackboardRemapping(blackboard_remapping);
  }

  // recursively construct child nodes
  for (const auto& child : node.children) {
    if (!child) {
      RCLCPP_WARN(rclcpp::get_logger("behavior_tree_decoder"), "Null child node pointer detected in parent '%s'", node.type_name.c_str());
      continue;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("behavior_tree_decoder"), "Inserting child node '%s' into parent '%s'", child->type_name.c_str(), node.type_name.c_str());
    auto_apms_behavior_tree::core::TreeDocument::NodeElement child_element = base_node.insertNode(child->type_name);
    recursiveNodeConstruction(*child, child_element, dictionary_manager);
    RCLCPP_DEBUG(rclcpp::get_logger("behavior_tree_decoder"), "Finished constructing child node '%s'", child->type_name.c_str());
  }

  RCLCPP_DEBUG(rclcpp::get_logger("behavior_tree_decoder"), "Finished constructing node '%s'", node.type_name.c_str());
  return base_node;

}

auto_apms_behavior_tree::core::TreeDocument::TreeElement BehaviorTreeDecoder::getTreeElementFromTree(const behavior_tree_representation::Tree& tree, auto_apms_behavior_tree::core::TreeDocument& tree_doc) {
  RCLCPP_INFO(this->get_logger(), "Constructing TreeElement for tree '%s'", tree.name.c_str());
  auto_apms_behavior_tree::core::TreeDocument::TreeElement tree_element = tree_doc.newTree(tree.name);
  auto_apms_behavior_tree::core::TreeDocument::NodeElement root = tree_element.insertNode(tree.root.type_name);
  recursiveNodeConstruction(tree.root, root, dictionary_manager_);
  RCLCPP_INFO(this->get_logger(), "Finished constructing TreeElement for tree '%s'", tree.name.c_str());
  return tree_element;
}


//TODO, still untested
void BehaviorTreeDecoder::encoded_in_callback(const auto_apms_behavior_codec_interfaces::msg::SerializedMessage::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received encoded message of size %zu bytes", msg->serialized_message.size());
  
  // Deserialize the message into a Document
  behavior_tree_representation::Document document;
  bool ok = document.deserialize(msg->serialized_message, dictionary_manager_);
  
  if (ok) {
    RCLCPP_INFO(this->get_logger(), "Successfully deserialized message into Document");
    
    // Reconstruct XML from the Document
    std::string xml_string = reconstructXML(document);
    
    if (!xml_string.empty()) {
      // Publish the XML string
      auto xml_msg = auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage();
      xml_msg.tree_xml_message = xml_string;
      xml_publisher_->publish(xml_msg);
      
      RCLCPP_INFO(this->get_logger(), "Published reconstructed XML message, content:\n%s", xml_string.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to reconstruct XML from Document");
    }
    
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to deserialize message into Document");
  }
}
} // namespace auto_apms_behavior_codec


using namespace auto_apms_behavior_codec;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // create a fresh manager for encoding/decoding
  auto dict = std::make_shared<DictionaryManager>();

  //create the encoder, TODO: make topics configurable, currently this subscribes to the Encode out topic of the encoder
  auto node = std::make_shared<BehaviorTreeDecoder>("encoded_out", "xml_out", dict);
  
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


