#include "behavior_tree_encoder.hpp"
#include "rclcpp/rclcpp.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "dictionary_manager.hpp"
#include <fstream>
#include <sstream>

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
      std::cout << "No root element found." << std::endl;
      return false;
    }
    behavior_tree_representation::Node working_root; // this will be the root of our tree representation, we will fill it while traversing the xml tree

    //function to work on nodes recursively and print their name and attributes (parameters) -> will in the future add the node to the tree representation
      std::function<bool(const tinyxml2::XMLElement *, int)> print_xml_node;
      print_xml_node = [&](const tinyxml2::XMLElement * ele, int depth) {
        //fist look only for formating
        for (int i = 0; i < depth; ++i) std::cout << "  ";

        // Print element name and all attributes (parameters)
        std::cout << ele->Name();

        DictionaryNode dictionary_node_entry =this->dictionary_manager_->get_dictionary_info_by_name(ele->Name());

        std::cout<< ", Supported: " << dictionary_node_entry.supported << ", corresponding id: " << dictionary_node_entry.id << std::endl;
        std::cout << "Expected ports for this node type:" << std::endl;

        //iterate through this nodes ports
        for (const auto & port : dictionary_node_entry.port_types) {
          //print the name and type of the port
          std::cout << "  Port Name: " << port.name << ", Type: " << port.type << std::endl;

          //print the value of the port
          const char* port_value = ele->Attribute(port.name.c_str());
          if (port_value) {
            std::cout << "    Value: " << port_value << std::endl;
          } else {
            std::cout << "    Value: not specified in XML" << std::endl;
          }
        }

        for (const tinyxml2::XMLAttribute * attr = ele->FirstAttribute(); attr != nullptr;
             attr = attr->Next()) {
          std::cout << " " << attr->Name() << "=\"" << attr->Value() << "\"";
        }
        std::cout << std::endl;

        // Recurse through all child elements (siblings too)
        bool found_unsupported_node = false;
        for (const tinyxml2::XMLElement * child = ele->FirstChildElement(); child != nullptr;
             child = child->NextSiblingElement()) {
          found_unsupported_node |= print_xml_node(child, depth + 1);
          
        }
        return found_unsupported_node || !dictionary_node_entry.supported;
      
    };

    // Iterate through all BehaviorTree elements
    for (const tinyxml2::XMLElement * tree_ele = root->FirstChildElement("BehaviorTree"); tree_ele != nullptr;
         tree_ele = tree_ele->NextSiblingElement("BehaviorTree")) {
      const char * tree_name = tree_ele->Attribute("ID");
      std::cout << "Tree: " << (tree_name ? tree_name : "unknown") << std::endl;
      document_out->trees.push_back(behavior_tree_representation::Tree());
      document_out->trees.back().name = tree_name ? tree_name : "unknown";
      // Print all child nodes of this tree
      for (const tinyxml2::XMLElement * child = tree_ele->FirstChildElement(); child != nullptr;
           child = child->NextSiblingElement()) {
        contains_unsuported_nodes |= print_xml_node(child, 1);
      }
      document_out->trees.back().root = working_root; // in the future, we will fill the working_root while traversing the xml and then set it as root of our tree representation here
    }
    //set the main_tree_to_execute to the one spefied in the XML
    //document_out->main_tree_to_execute = xml_doc.RootElement()->Attribute("main_tree_to_execute");
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
            current_node.children.push_back(process_node(first_child));
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
          root_node.children.push_back(process_node(first_child));
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
    
    std::unique_ptr<behavior_tree_representation::Document> document2;
    bool ok2 = node->readTreeDefinitionFromDocument(xml, document2);
    RCLCPP_INFO(node->get_logger(), "readTreeDefinitionFromDocument returned: %s", ok2 ? "true" : "false");
  } else {
    RCLCPP_WARN(node->get_logger(), "Could not open example XML: %s", xml_path.c_str());
  }
  rclcpp::spin(node);

  
  rclcpp::shutdown();
  return 0;
}
