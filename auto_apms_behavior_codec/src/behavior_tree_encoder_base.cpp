#include "auto_apms_behavior_codec/behavior_tree_encoder_base.hpp"

#include <iomanip>
#include <sstream>

#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"

namespace auto_apms_behavior_codec
{

BehaviorTreeEncoderBase::BehaviorTreeEncoderBase(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options), param_listener_(this)
{
  setupEncoder();
}

void BehaviorTreeEncoderBase::setupEncoder()
{
  const auto params = param_listener_.get_params();

  // Construct dictionary manager from node_manifest parameter
  std::vector<auto_apms_behavior_tree::core::NodeManifestResourceIdentity> manifest_ids(
    params.node_manifest.begin(), params.node_manifest.end());
  dictionary_manager_ = std::make_shared<DictionaryManager>(manifest_ids);

  RCLCPP_INFO(
    this->get_logger(), "Encoder will use the following dictionary:\n%s",
    dictionary_manager_->print_dictionary_to_string().c_str());

  // Set up publisher for encoded messages
  encoded_publisher_ = this->create_publisher<auto_apms_behavior_codec_interfaces::msg::SerializedTreeMessage>(
    params.encoded_out_topic, 10);
}

std::shared_ptr<DictionaryManager> BehaviorTreeEncoderBase::getDictionaryManager() const { return dictionary_manager_; }

auto_apms_behavior_tree::core::NodeManifest BehaviorTreeEncoderBase::getNodeManifest() const
{
  return dictionary_manager_->getNodeManifest();
}

std::vector<uint8_t> BehaviorTreeEncoderBase::encode(behavior_tree_representation::Document & document)
{
  return document.serialize(dictionary_manager_);
}

behavior_tree_representation::Node BehaviorTreeEncoderBase::getNodeFromElement(
  const auto_apms_behavior_tree::core::TreeDocument::NodeElement & node_element)
{
  auto dict_entry = dictionary_manager_->get_dictionary_info_by_name(node_element.getRegistrationName());

  behavior_tree_representation::Node result;
  result.type_name = dict_entry.name;
  result.instance_name = node_element.getName();

  // special handling for subtree ports, work directly on xml
  if (dict_entry.name == "SubTree") {
    auto_apms_behavior_tree::core::TreeDocument::NodeElement copy_element = node_element;
    tinyxml2::XMLElement * node_xml = copy_element.getXMLElement();
    const tinyxml2::XMLAttribute * attr = node_xml->FirstAttribute();
    uint attr_number = 0;
    while (attr) {
      if (std::string(attr->Name()) == "_autoremap") {
        result.ports.push_back(
          std::make_shared<behavior_tree_representation::PortBool>(
            std::string(attr->Value()) == "true", result.ports.size()));
      } else {
        result.ports.push_back(
          std::make_shared<behavior_tree_representation::PortSubTreeSpecial>(
            behavior_tree_representation::PortSubTreeSpecial(attr->Value(), attr->Name(), attr_number)));
      }
      attr_number++;
      attr = attr->Next();
    }
  } else {
    // Handle ports regularly based on the dictionary entry
    std::map<std::string, std::string> port_values = node_element.getPorts();

    for (const auto & port_info : dict_entry.port_types) {
      if (port_values.find(port_info.name) != port_values.end()) {
        std::string port_value = port_values.at(port_info.name);
        std::shared_ptr<behavior_tree_representation::Port> port_ptr;
        try {
          if (port_info.type == "int") {
            port_ptr =
              std::make_shared<behavior_tree_representation::PortInt>(std::stoi(port_value), result.ports.size());
          } else if (port_info.type == "unsigned int") {
            port_ptr = std::make_shared<behavior_tree_representation::PortUInt>(
              static_cast<uint32_t>(std::stoul(port_value)), result.ports.size());
          } else if (port_info.type == "float") {
            port_ptr =
              std::make_shared<behavior_tree_representation::PortFloat>(std::stof(port_value), result.ports.size());
          } else if (port_info.type == "double") {
            port_ptr =
              std::make_shared<behavior_tree_representation::PortDouble>(std::stod(port_value), result.ports.size());
          } else if (port_info.type == "std::string") {
            port_ptr = std::make_shared<behavior_tree_representation::PortString>(port_value, result.ports.size());
          } else if (port_info.type == "bool") {
            port_ptr =
              std::make_shared<behavior_tree_representation::PortBool>(port_value == "true", result.ports.size());
          } else if (port_info.type == "BT::AnyTypeAllowed") {
            port_ptr =
              std::make_shared<behavior_tree_representation::PortAnyTypeAllowed>(port_value, result.ports.size());
          } else if (port_info.type == "BT::NodeStatus") {
            port_ptr = std::make_shared<behavior_tree_representation::PortNodeStatus>(port_value, result.ports.size());
          } else if (port_info.type == "BT::Any") {
            port_ptr = std::make_shared<behavior_tree_representation::PortAny>(port_value, result.ports.size());
          } else {
            port_ptr = std::make_shared<behavior_tree_representation::PortInvalid>(port_value, result.ports.size());
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Failed to convert port value '%s' for port '%s' of type '%s': %s. Including it as invalid.",
            port_value.c_str(), port_info.name.c_str(), port_info.type.c_str(), e.what());
          port_ptr = std::make_shared<behavior_tree_representation::PortInvalid>(port_value, result.ports.size());
        }
        if (port_ptr) {
          result.ports.push_back(port_ptr);
        }
      }
    }
  }

  //handle additional parameters, detected as attributes in the xml which are not ports according to the dictionary, but still should be encoded as string key value pairs for potential use in the behavior tree execution environment
  auto_apms_behavior_tree::core::TreeDocument::NodeElement copy_element = node_element;
  tinyxml2::XMLElement * node_xml = copy_element.getXMLElement();
  const tinyxml2::XMLAttribute * attr = node_xml->FirstAttribute();
  while (attr) {
    std::cout << "Processing attribute: " << attr->Name() << " = " << attr->Value() << std::endl;
    std::string attr_name = attr->Name();
    // check if this attribute name is not already handled as a port
    bool is_port = false;
    for (const auto & port_info : dict_entry.port_types) {
      if (port_info.name == attr_name) {
        is_port = true;
        break;
      }
    }
    if (!is_port) {
      // this is an additional parameter, add it to the node's additional_parameters vector
      result.additional_parameters.push_back({attr_name, attr->Value()});
      RCLCPP_WARN(
        this->get_logger(),
        "Attribute '%s' with value '%s' on node '%s' is not defined as a port in the dictionary. Adding it as an additional parameter.",
        attr_name.c_str(), attr->Value(), result.type_name.c_str());
    }
    attr = attr->Next();
  }

  // Recursively process children
  if (node_element.hasChildren()) {
    auto child_it = node_element.begin();
    for (; child_it != node_element.end(); ++child_it) {
      behavior_tree_representation::Node child_node = getNodeFromElement(*child_it);
      result.children.push_back(std::make_shared<behavior_tree_representation::Node>(child_node));
    }
  }

  return result;
}

bool BehaviorTreeEncoderBase::readTreeDefinitionFromDocument(
  auto_apms_behavior_tree::core::TreeDocument & tree_doc,
  std::unique_ptr<behavior_tree_representation::Document> & document_out)
{
  try {
    document_out = std::make_unique<behavior_tree_representation::Document>();

    std::vector<std::string> tree_names = tree_doc.getAllTreeNames();
    RCLCPP_INFO(this->get_logger(), "Found %zu trees in document", tree_names.size());

    for (const auto & tree_name : tree_names) {
      auto tree_element = tree_doc.getTree(tree_name);

      document_out->trees.push_back(behavior_tree_representation::Tree());
      document_out->trees.back().name = tree_name;

      auto root_node_element = tree_element.getFirstNode();
      behavior_tree_representation::Node root_node = getNodeFromElement(root_node_element);
      document_out->trees.back().root = root_node;
    }

    // Set the main tree to execute
    std::string root_tree_name;
    try {
      root_tree_name = tree_doc.getRootTreeName();
      document_out->main_tree_to_execute = root_tree_name;
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to get root tree name from document: %s. Setting main_tree_to_execute to empty string.", e.what());
      document_out->main_tree_to_execute = "";
    }

    // Reorder trees to ensure the main tree comes first
    if (!root_tree_name.empty()) {
      auto it = std::find_if(
        document_out->trees.begin(), document_out->trees.end(),
        [&](const behavior_tree_representation::Tree & tree) { return tree.name == root_tree_name; });
      if (it != document_out->trees.end()) {
        std::iter_swap(document_out->trees.begin(), it);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Successfully parsed tree document with %zu trees", document_out->trees.size());
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read tree definition: %s", e.what());
    return false;
  }
}

bool BehaviorTreeEncoderBase::readTreeDefinitionFromXML(
  std::string tree_xml, std::unique_ptr<behavior_tree_representation::Document> & document_out)
{
  if (tree_xml.empty()) {
    RCLCPP_ERROR(this->get_logger(), "XML string is empty");
    return false;
  }

  auto_apms_behavior_tree::core::TreeDocument tree_doc;
  try {
    tree_doc.mergeString(tree_xml, true);
  } catch (const std::exception & parse_error) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse XML: %s", parse_error.what());
    return false;
  }

  tree_doc.registerNodes(dictionary_manager_->getNodeManifest());

  return readTreeDefinitionFromDocument(tree_doc, document_out);
}

bool BehaviorTreeEncoderBase::encodeAndPublish(const std::string & tree_xml)
{
  try {
    publishEncoded(encodeToBytes(tree_xml));
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "encodeAndPublish failed: %s", e.what());
    return false;
  }
}

std::vector<uint8_t> BehaviorTreeEncoderBase::encodeToBytes(const std::string & tree_xml)
{
  std::unique_ptr<behavior_tree_representation::Document> document;
  if (!readTreeDefinitionFromXML(tree_xml, document) || !document) {
    throw std::runtime_error("Failed to parse or encode XML");
  }
  return encode(*document);
}

void BehaviorTreeEncoderBase::publishEncoded(const std::vector<uint8_t> & encoded_data)
{
  auto msg = auto_apms_behavior_codec_interfaces::msg::SerializedTreeMessage();
  msg.serialized_tree_message = encoded_data;
  encoded_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Published encoded message of length %zu", encoded_data.size());
}

}  // namespace auto_apms_behavior_codec
