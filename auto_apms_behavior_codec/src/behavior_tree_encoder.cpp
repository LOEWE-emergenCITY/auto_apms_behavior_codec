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


bool BehaviorTreeEncoder::readTreeDefinition(std::string tree_xml){
  try {
    tinyxml2::XMLDocument xml_doc;
    if (xml_doc.Parse(tree_xml.c_str()) != tinyxml2::XML_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse XML: %s", xml_doc.ErrorStr());
      return false;
    }
    bool contains_unsuported_nodes = false;
    // Find all BehaviorTree elements
    tinyxml2::XMLElement * root = xml_doc.RootElement();
    if (!root) {
      std::cout << "No root element found." << std::endl;
      return false;
    }
      //function to work on nodes recursively and print their name and attributes (parameters) -> will in the future add the node to the tree representation
      std::function<bool(const tinyxml2::XMLElement *, int)> print_xml_node;
      print_xml_node = [&](const tinyxml2::XMLElement * ele, int depth) {
        for (int i = 0; i < depth; ++i) std::cout << "  ";
        // Print element name and all attributes (parameters)
        std::cout << ele->Name();
        DictionaryNode dictionary_node_entry =this->dictionary_manager_->get_dictionary_info_by_name(ele->Name());
        std::cout<< ", Supported: " << dictionary_node_entry.supported << ", corresponding id: " << dictionary_node_entry.id << std::endl;
        std::cout << "Expected ports for this node type:" << std::endl;
        for (const auto & port : dictionary_node_entry.port_types) {
          std::cout << "  Port Name: " << port.name << ", Type: " << port.type << std::endl;
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

      // Print all child nodes of this tree
      for (const tinyxml2::XMLElement * child = tree_ele->FirstChildElement(); child != nullptr;
           child = child->NextSiblingElement()) {
        contains_unsuported_nodes |= print_xml_node(child, 1);
      }
    }

    return !contains_unsuported_nodes;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read tree definition: %s", e.what());
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
    bool ok = node->readTreeDefinition(xml);
    RCLCPP_INFO(node->get_logger(), "readTreeDefinition returned: %s", ok ? "true" : "false");
  } else {
    RCLCPP_WARN(node->get_logger(), "Could not open example XML: %s", xml_path.c_str());
  }

  rclcpp::spin(node);

  
  rclcpp::shutdown();
  return 0;
}
