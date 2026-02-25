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

//TODO change to TreeDocument API
std::string BehaviorTreeDecoder::reconstructXML(const behavior_tree_representation::Document& document) {
  auto_apms_behavior_tree::core::TreeDocument tree_doc;

  for (const auto& tree : document.trees) {
    if(tree.name == document.main_tree_to_execute){
      tree_doc.newTree(getTreeElementFromTree(tree)).makeRoot();
    }
    else{
      tree_doc.newTree(getTreeElementFromTree(tree));
    }
  }

  return tree_doc.writeToString();
}

std::map<std::string, std::string> get_port_map(const behavior_tree_representation::Node& node) {
  std::map<std::string, std::string> port_map;
  for (const auto& port : node.ports) {
    // Example: handle PortString, PortInt, PortFloat, PortBool, PortAnyTypeAllowed
    if (auto port_string = std::dynamic_pointer_cast<behavior_tree_representation::PortString>(port)) {
      port_map[port_string->getType()] = port_string->value;
    } else if (auto port_int = std::dynamic_pointer_cast<behavior_tree_representation::PortInt>(port)) {
      port_map[port_int->getType()] = std::to_string(port_int->value);
    } else if (auto port_float = std::dynamic_pointer_cast<behavior_tree_representation::PortFloat>(port)) {
      port_map[port_float->getType()] = std::to_string(port_float->value);
    } else if (auto port_bool = std::dynamic_pointer_cast<behavior_tree_representation::PortBool>(port)) {
      port_map[port_bool->getType()] = port_bool->value ? "true" : "false";
    } else if (auto port_any = std::dynamic_pointer_cast<behavior_tree_representation::PortAnyTypeAllowed>(port)) {
      port_map[port_any->getType()] = port_any->value;
    }
  }
  return port_map;
};

auto_apms_behavior_tree::core::TreeDocument::NodeElement recursiveNodeConstruction(
  const behavior_tree_representation::Node& node,
  auto_apms_behavior_tree::core::TreeDocument::NodeElement& base_node)
{
  // add ports to the node element
  auto port_map = get_port_map(node);
  base_node.setPorts(port_map);

  // recursively construct child nodes
  for (const auto& child : node.children) {
    auto_apms_behavior_tree::core::TreeDocument::NodeElement child_element = base_node.insertNode(child->type_name);
    recursiveNodeConstruction(*child, child_element);
  }

  return base_node;

}

auto_apms_behavior_tree::core::TreeDocument::TreeElement BehaviorTreeDecoder::getTreeElementFromTree(const behavior_tree_representation::Tree& tree){

  auto_apms_behavior_tree::core::TreeDocument working_tree_doc; // Tree Elements dont seem to be constructable independently from a Tree Document
  auto_apms_behavior_tree::core::TreeDocument::TreeElement tree_element = working_tree_doc.newTree(tree.name);
  auto_apms_behavior_tree::core::TreeDocument::NodeElement root = tree_element.insertNode(tree.root.type_name);
  recursiveNodeConstruction(tree.root, root);
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
      
      RCLCPP_INFO(this->get_logger(), "Published reconstructed XML message");
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


