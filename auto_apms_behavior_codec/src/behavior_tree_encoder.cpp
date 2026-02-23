#include "auto_apms_behavior_codec/behavior_tree_encoder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "auto_apms_behavior_codec/dictionary_manager.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace auto_apms_behavior_codec;

BehaviorTreeEncoder::BehaviorTreeEncoder(std::string xml_in, std::string encoded_out, std::shared_ptr<DictionaryManager> dictionary_manager)
    : rclcpp::Node("behavior_tree_encoder")
{
  // Initialize the dictionary manager
  this->dictionary_manager_ = dictionary_manager;

  std::cout << "BehaviorTreeEncoder will use the following dictionary:" << std::endl;
  this->dictionary_manager_->print_dictionary();

  // Set up subscription for incoming XML messages
  xml_subscription_ = this->create_subscription<auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage>(
    "xml_in", 10, std::bind(&BehaviorTreeEncoder::xml_in_callback, this, std::placeholders::_1));
  
  // Set up publisher for encoded messages
  encoded_publisher_ = this->create_publisher<auto_apms_behavior_codec_interfaces::msg::SerializedMessage>("encoded_out", 10);

}

void BehaviorTreeEncoder::xml_in_callback(const auto_apms_behavior_codec_interfaces::msg::TreeXmlMessage::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received XML message of length %zu", msg->tree_xml_message.size());
  
  std::unique_ptr<behavior_tree_representation::Document> document;

  //currently readTreeDefinitionFromXML uses a hacky workaround to register some node manifest, this still needs implementation
  bool ok = this->readTreeDefinitionFromXML(msg->tree_xml_message, document);

  if (ok && document) {
    //document->print();
    std::vector<uint8_t> encoded_data = this->encode(*document);

    std::cout << "Encoded data (hex): ";
    for (uint8_t byte : encoded_data) {
      std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
  std::cout << std::dec << std::endl; // Reset to decimal

  auto encoded_msg = auto_apms_behavior_codec_interfaces::msg::SerializedMessage();
  encoded_msg.serialized_message = encoded_data;
  encoded_publisher_->publish(encoded_msg);

  RCLCPP_INFO(this->get_logger(), "Published encoded message of length %zu", encoded_data.size());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse XML message");
  }
}


std::vector<uint8_t> BehaviorTreeEncoder::encode(const std::string& behavior_tree_yaml)
{
  // TODO: Implement behavior tree encoding logic
  return std::vector<uint8_t>();
}

behavior_tree_representation::Node BehaviorTreeEncoder::getNodeFromElement(const auto_apms_behavior_tree::core::TreeDocument::NodeElement& node_element){
  std::cout << "Processing node: " << node_element.getName() << " Type: " << node_element.getRegistrationName() << "" << std::endl;
  auto_apms_behavior_codec::DictionaryNode dict_entry = dictionary_manager_->get_dictionary_info_by_name(node_element.getRegistrationName());
  
  behavior_tree_representation::Node result;
  result.type_name = dict_entry.name;
  result.instance_name = node_element.getName();
  // special handling for subtree ports, work directly on xml
  // An additional question could be if sub trees can have child nodes? are they handled correctly
  if(dict_entry.name == "SubTree"){
    auto_apms_behavior_tree::core::TreeDocument::NodeElement copy_element = node_element;
    tinyxml2::XMLElement* node_xml = copy_element.getXMLElement();
    std::cout << "Handling SubTree, XML name: "<< node_xml->Name() << std::endl;
    const tinyxml2::XMLAttribute* attr = node_xml->FirstAttribute();
    uint attr_number = 0;
    while (attr)
    {
      if(std::string(attr->Name()) == "_autoremap"){
        std::cout<< "Handeling SubTree autoremap" << std::endl;
        result.ports.push_back(std::make_shared<behavior_tree_representation::PortBool>(attr->Value() == "true", result.ports.size()));

      }
      else{
        std::cout<<"Handling attribute: " << attr->Name() <<std::endl;
        result.ports.push_back(std::make_shared<behavior_tree_representation::PortSubTreeSpecial>(behavior_tree_representation::PortSubTreeSpecial(attr->Value(), attr->Name(), attr_number )));
        
      }
      attr_number++;
      attr = attr->Next();
    }
  }
  else{
    // Handle ports regularly based on the dictionary entry
    std::map<std::string, std::string> port_values = node_element.getPorts();

    //For some reason the map returned by getPorts seems to be empty
    std::cout << "Checking Ports, expecting: " << dict_entry.port_types.size() << " ports, found: " << port_values.size() << " ports" << std::endl;
    for(const auto& port_info : dict_entry.port_types){

      if(port_values.find(port_info.name) != port_values.end()){
        std::string port_value = port_values.at(port_info.name);
        std::cout << "Node: " << result.instance_name << ", Port: " << port_info.name << " = " << port_value << "type: "<< port_info.type<< std::endl;
        std::shared_ptr<behavior_tree_representation::Port> port_ptr;
        if(port_info.type == "int"){
          port_ptr = std::make_shared<behavior_tree_representation::PortInt>(std::stoi(port_value), result.ports.size()); //result.ports.size() results in the index of the port in the dictionary entry, this corresponds to the ports id, //TODO: figure out if this can be omitted
        }
        else if(port_info.type == "float"){
          port_ptr = std::make_shared<behavior_tree_representation::PortFloat>(std::stof(port_value), result.ports.size());
        }
        //handle AnyTypeALlowed as string, this might work
        else if(port_info.type == "std::string"){
          port_ptr = std::make_shared<behavior_tree_representation::PortString>(port_value, result.ports.size());
        }
        else if(port_info.type == "bool"){
          port_ptr = std::make_shared<behavior_tree_representation::PortBool>(port_value == "true", result.ports.size());
        }
        else if(port_info.type == "BT::AnyTypeAllowed"){
          port_ptr = std::make_shared<behavior_tree_representation::PortAnyTypeAllowed>(port_value, result.ports.size());
        }

        if(port_ptr){
          result.ports.push_back(port_ptr);
        }
      }
    }
  }

  // Recursively process children
  if(node_element.hasChildren()){
    auto_apms_behavior_tree::core::TreeDocument::NodeElement::ChildIterator child_it = node_element.begin();
    for(child_it; child_it != node_element.end(); ++child_it){
      behavior_tree_representation::Node child_node = getNodeFromElement(*child_it);
      result.children.push_back(std::make_shared<behavior_tree_representation::Node>(child_node));
    }
  }

  return result;
}


//attempt at using the TreeDocument API to read the tree definition, somehow not all information seems to be easilie retrivable from the tree document, especially regarding ports of nodes
bool BehaviorTreeEncoder::readTreeDefinitionFromDocument(auto_apms_behavior_tree::core::TreeDocument& tree_doc, std::unique_ptr<behavior_tree_representation::Document>& document_out){
  try {
    
    std::cout << "read document: "<< std::endl << tree_doc.writeToString() << std::endl;
    document_out = std::make_unique<behavior_tree_representation::Document>();
    
    // Get all trees from the document
    std::vector<std::string> tree_names = tree_doc.getAllTreeNames();
    RCLCPP_INFO(this->get_logger(), "Found %zu trees in document", tree_names.size());
    
    for (const auto & tree_name : tree_names) {
      std::cout << "Processing tree: " << tree_name << std::endl;
      auto tree_element = tree_doc.getTree(tree_name);
      
      // Create a new tree in our representation
      document_out->trees.push_back(behavior_tree_representation::Tree());
      document_out->trees.back().name = tree_name;
      
      // Initialize root node from tree element
      behavior_tree_representation::Node root_node;

      //get root node of the tree
      auto_apms_behavior_tree::core::TreeDocument::NodeElement root_node_element = tree_element.getFirstNode();
      
      std::cout << "Root node: " << root_node_element.getName() << ", Type: " << root_node_element.getRegistrationName() << std::endl;

      // construct the representation of the root node, this function will recurese throug the entire tree -> TODO
      root_node = getNodeFromElement(root_node_element);

      document_out->trees.back().root = root_node;
    }
    
    // Set the main tree to execute
    std::string root_tree_name = tree_doc.getRootTreeName();
    document_out->main_tree_to_execute = root_tree_name;

    // reorder trees in document in order to ensure the main tree to execute comes first, this simplyfies encoding
    if(!root_tree_name.empty()){
      auto it = std::find_if(document_out->trees.begin(), document_out->trees.end(), [&](const behavior_tree_representation::Tree& tree){
        return tree.name == root_tree_name;
      });
      if(it != document_out->trees.end()){
        std::iter_swap(document_out->trees.begin(), it);
      }
    }

    document_out->print();
    
    RCLCPP_INFO(this->get_logger(), "Successfully parsed tree document with %zu trees using TreeDocument API", document_out->trees.size());
    return true;
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read tree definition using TreeDocument: %s", e.what());
    return false;
  }
}


std::vector<uint8_t> BehaviorTreeEncoder::encode(behavior_tree_representation::Document& document) {
  return document.serialize(dictionary_manager_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // create a fresh manager for encoding/decoding
  auto dict = std::make_shared<DictionaryManager>();

  //create the encoder, TODO: make topics configurable
  auto node = std::make_shared<BehaviorTreeEncoder>("xml_in", "encoded_out", dict);



  /*
  // --- Decoding test: deserialize the encoded bytes and print the reconstructed Document
  
    behavior_tree_representation::Document decoded_doc;
    
    bool ok_decode = decoded_doc.deserialize(encoded_data, dict);
    if(ok_decode){
      std::cout << "Decoded Document:" << std::endl;
      decoded_doc.print();
    } else {
      std::cerr << "Decoding test failed" << std::endl;
    }
  
  //Test XML reconstruction
  //std::string reconstructed_xml = node->reconstructXML(*document);
  //RCLCPP_INFO(node->get_logger(), "Reconstructed XML (first 500 chars):\n%s", reconstructed_xml.substr(0, 500).c_str());
  */
  
  rclcpp::shutdown();
  return 0;
}
