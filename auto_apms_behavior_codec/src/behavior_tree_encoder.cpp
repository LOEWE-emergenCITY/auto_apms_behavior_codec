#include "auto_apms_behavior_codec/behavior_tree_encoder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "auto_apms_behavior_codec/dictionary_manager.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace auto_apms_behavior_codec;

BehaviorTreeEncoder::BehaviorTreeEncoder()
    : rclcpp::Node("behavior_tree_encoder")
{
  // Initialize the dictionary manager
  this->dictionary_manager_ = std::make_unique<DictionaryManager>();

  this->dictionary_manager_->print_dictionary();
}

std::vector<uint8_t> BehaviorTreeEncoder::encode(const std::string& behavior_tree_yaml)
{
  // TODO: Implement behavior tree encoding logic
  return std::vector<uint8_t>();
}


bool BehaviorTreeEncoder::readTreeDefinition(std::string tree_xml, std::unique_ptr<behavior_tree_representation::Document>& document_out){
  try {
    tinyxml2::XMLDocument xml_doc;
    if (xml_doc.Parse(tree_xml.c_str()) != tinyxml2::XML_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse XML: %s", xml_doc.ErrorStr());
      return false;
    }
    bool contains_unsuported_nodes = false;

    document_out = std::make_unique<behavior_tree_representation::Document>();
    // Find all BehaviorTree elements
    tinyxml2::XMLElement * root = xml_doc.RootElement();
    if (!root) {
      RCLCPP_ERROR(this->get_logger(), "No root element found in XML");
      return false;
    }

    // Get the main tree to execute from the root element
    const char* main_tree = root->Attribute("main_tree_to_execute");
    if (main_tree) {
      document_out->main_tree_to_execute = main_tree;
    }

    // Function to recursively process nodes and populate the tree representation
    std::function<behavior_tree_representation::Node(const tinyxml2::XMLElement *)> process_node;
    process_node = [&](const tinyxml2::XMLElement * ele) -> behavior_tree_representation::Node {
      behavior_tree_representation::Node node;
      node.registration_name = ele->Name();
      
      // Get the instance name from the "name" attribute, or use registration name if not present
      const char* instance_name = ele->Attribute("name");
      node.instance_name = instance_name ? instance_name : ele->Name();

      // Get dictionary info for this node type
      DictionaryNode dictionary_node_entry = this->dictionary_manager_->get_dictionary_info_by_name(ele->Name());
      
      if (!dictionary_node_entry.supported) {
        RCLCPP_WARN(this->get_logger(), "Node type '%s' is not supported for encoding/decoding", ele->Name());
        contains_unsuported_nodes = true;
      }

      // Process all ports for this node
      for (const auto & port_info : dictionary_node_entry.port_types) {
        const char* port_value = ele->Attribute(port_info.name.c_str());
        if (port_value) {
          // Parse the port value based on its type and create appropriate Port object
          std::shared_ptr<behavior_tree_representation::Port> port_ptr;
          
          if (port_info.type == "int32_t" || port_info.type == "int") {
            try {
              int32_t value = std::stoi(port_value);
              int16_t port_id = static_cast<int16_t>(node.ports.size());
              port_ptr = std::make_shared<behavior_tree_representation::PortInt>(value, port_id);
            } catch (const std::exception& e) {
              RCLCPP_WARN(this->get_logger(), "Failed to parse int port '%s': %s", port_info.name.c_str(), e.what());
            }
          } else if (port_info.type == "float") {
            try {
              float value = std::stof(port_value);
              int16_t port_id = static_cast<int16_t>(node.ports.size());
              port_ptr = std::make_shared<behavior_tree_representation::PortFloat>(value, port_id);
            } catch (const std::exception& e) {
              RCLCPP_WARN(this->get_logger(), "Failed to parse float port '%s': %s", port_info.name.c_str(), e.what());
            }
          } else if (port_info.type == "bool") {
            bool value = (std::string(port_value) == "true" || std::string(port_value) == "1");
            int16_t port_id = static_cast<int16_t>(node.ports.size());
            port_ptr = std::make_shared<behavior_tree_representation::PortBool>(value, port_id);
          } else if (port_info.type == "std::string") {
            int16_t port_id = static_cast<int16_t>(node.ports.size());
            port_ptr = std::make_shared<behavior_tree_representation::PortString>(port_value, port_id);
          } else if (port_info.type == "BT::AnyTypeAllowed") {
            //TODO handle AnyType allowed
          } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported port type '%s' for port '%s' in node '%s'", port_info.type.c_str(), port_info.name.c_str(), ele->Name());
          }
          
          if (port_ptr) {
            node.ports.push_back(port_ptr);
          }
        }
      }

      // Recursively process all child nodes
      for (const tinyxml2::XMLElement * child = ele->FirstChildElement(); child != nullptr;
           child = child->NextSiblingElement()) {
        node.children.push_back(std::make_shared<behavior_tree_representation::Node>(process_node(child)));
      }

      return node;
    };

    // Iterate through all BehaviorTree elements
    for (const tinyxml2::XMLElement * tree_ele = root->FirstChildElement("BehaviorTree"); tree_ele != nullptr;
         tree_ele = tree_ele->NextSiblingElement("BehaviorTree")) {
      const char * tree_name = tree_ele->Attribute("ID");
      RCLCPP_DEBUG(this->get_logger(), "Processing tree: %s", tree_name ? tree_name : "unknown");
      
      document_out->trees.push_back(behavior_tree_representation::Tree());
      document_out->trees.back().name = tree_name ? tree_name : "unknown";
      
      // Process all child nodes of this tree
      for (const tinyxml2::XMLElement * child = tree_ele->FirstChildElement(); child != nullptr;
           child = child->NextSiblingElement()) {
        document_out->trees.back().root = process_node(child);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Successfully parsed tree document with %zu trees", document_out->trees.size());
    return !contains_unsuported_nodes;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read tree definition: %s", e.what());
    return false;
  }
}

//attempt at using the TreeDocument API to read the tree definition, somehow not all information seems to be easylie retrivable from the tree document, especially regarding ports of nodes
bool BehaviorTreeEncoder::readTreeDefinitionFromDocument(std::string tree_xml, std::unique_ptr<behavior_tree_representation::Document>& document_out){
  try {
    // Validate that we have XML content
    if (tree_xml.empty()) {
      RCLCPP_ERROR(this->get_logger(), "XML string is empty");
      return false;
    }
    
    //TODO: subtree needs special handeling, as it has a dynamic number of ports, and a name not included in the NodeModelMap

    RCLCPP_DEBUG(this->get_logger(), "Parsing XML of length %zu", tree_xml.length());
    
    // Use the auto_apms TreeDocument API to parse the XML
    auto_apms_behavior_tree::core::TreeDocument tree_doc;
    
    // Merge the XML string into the tree document
    try {
      tree_doc.mergeString(tree_xml, true);
    } catch (const std::exception & parse_error) {
      return false;
    }
    std::cout << "read document: " << tree_doc.writeToString() << std::endl;
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
      root_node.registration_name = tree_element.getRegistrationName();
      root_node.instance_name = tree_element.getName();
      
      // Function to recursively process nodes
      std::function<behavior_tree_representation::Node(const auto_apms_behavior_tree::core::TreeDocument::NodeElement&)> process_node;
      process_node = [&](const auto_apms_behavior_tree::core::TreeDocument::NodeElement& node) -> behavior_tree_representation::Node {
        behavior_tree_representation::Node current_node;
        current_node.registration_name = node.getRegistrationName();
        current_node.instance_name = node.getName();
        
        // Process all ports of this node
        const auto & port_names = node.getPortNames();
      
        auto port_values = node.getPorts();
        for (const auto & port_name : port_names) {
          std::cout << "Node: " << current_node.instance_name << ", Port: " << port_name << std::endl;
          if (port_values.count(port_name) > 0) {
            const std::string & port_value = port_values.at(port_name);
            RCLCPP_INFO(this->get_logger(), "Node: %s, Port: %s = %s", 
              current_node.instance_name.c_str(), port_name.c_str(), port_value.c_str());
          }
        }
        
        // Process direct children (the immediate descendants)
        if (node.hasChildren()) {
          try {
            // Get the first child
            auto first_child = node.getFirstNode();
            current_node.children.push_back(std::make_shared<behavior_tree_representation::Node>(process_node(first_child)));
          } catch (const std::exception &) {
            // No children found, which is fine
          }
        }
        
        return current_node;
      };
      
      // Process the tree's first child if it exists
      if (tree_element.hasChildren()) {
        try {
          auto first_child = tree_element.getFirstNode();
          root_node.children.push_back(std::make_shared<behavior_tree_representation::Node>(process_node(first_child)));
        } catch (const std::exception &) {
          // Tree has no children, which is fine
        }
      }
      
      document_out->trees.back().root = root_node;
    }
    
    // Set the main tree to execute
    std::string root_tree_name = tree_doc.getRootTreeName();
    document_out->main_tree_to_execute = root_tree_name;
    document_out->print();
    
    RCLCPP_INFO(this->get_logger(), "Successfully parsed tree document with %zu trees using TreeDocument API", document_out->trees.size());
    return true;
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read tree definition using TreeDocument: %s", e.what());
    return false;
  }
}

std::string BehaviorTreeEncoder::reconstructXML(const behavior_tree_representation::Document& document) {
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
      tinyxml2::XMLElement* node_element = xml_doc.NewElement(node.registration_name.c_str());
      
      // Add instance name if it differs from registration name
      if (!node.instance_name.empty() && node.instance_name != node.registration_name) {
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
        DictionaryNode dict_node = this->dictionary_manager_->get_dictionary_info_by_name(node.registration_name);
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
      if (!tree.root.registration_name.empty()) {
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

std::vector<uint8_t> BehaviorTreeEncoder::encode(behavior_tree_representation::Document& document) {
  return document.serialize(dictionary_manager_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BehaviorTreeEncoder>();

    // Read example XML and test readTreeDefinition
  const std::string xml_path = "/home/hiwi/orcas-ws/src/pkg/auto_apms_behavior_codec/auto_apms_behavior_codec_examples/behavior/hello_world.xml";
  std::ifstream in(xml_path);
  if (in) {
    std::stringstream ss; ss << in.rdbuf();
    const std::string xml = ss.str();
    
    std::unique_ptr<behavior_tree_representation::Document> document;
    bool ok = node->readTreeDefinition(xml, document);
    RCLCPP_INFO(node->get_logger(), "readTreeDefinition returned: %s", ok ? "true" : "false");
    
    if (ok && document) {
      document->print();
      std::vector<uint8_t> encoded_data = node->encode(*document);

      std::cout << "Encoded data (hex): ";
      for (uint8_t byte : encoded_data) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
      }
      std::cout << std::dec << std::endl; // Reset to decimal
      // Test XML reconstruction
      std::string reconstructed_xml = node->reconstructXML(*document);
      RCLCPP_INFO(node->get_logger(), "Reconstructed XML (first 500 chars):\n%s", reconstructed_xml.substr(0, 500).c_str());
    }
    
    std::unique_ptr<behavior_tree_representation::Document> document2;
    //bool ok2 = node->readTreeDefinitionFromDocument(xml, document2);
    //RCLCPP_INFO(node->get_logger(), "readTreeDefinitionFromDocument returned: %s", ok2 ? "true" : "false");
  } else {
    RCLCPP_WARN(node->get_logger(), "Could not open example XML: %s", xml_path.c_str());
  }
  
  rclcpp::shutdown();
  return 0;
}
