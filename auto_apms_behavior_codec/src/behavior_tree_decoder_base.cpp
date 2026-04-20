#include "auto_apms_behavior_codec/behavior_tree_decoder_base.hpp"

#include "auto_apms_behavior_tree_core/node/node_model_type.hpp"

namespace auto_apms_behavior_codec
{

BehaviorTreeDecoderBase::BehaviorTreeDecoderBase(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options), param_listener_(this)
{
  setupDecoder();
}

auto_apms_behavior_tree::core::NodeManifest BehaviorTreeDecoderBase::getNodeManifest() const
{
  return dictionary_manager_->getNodeManifest();
}

void BehaviorTreeDecoderBase::setupDecoder()
{
  const auto params = param_listener_.get_params();

  std::vector<auto_apms_behavior_tree::core::NodeManifestResourceIdentity> manifest_ids(
    params.node_manifest.begin(), params.node_manifest.end());
  dictionary_manager_ = std::make_shared<DictionaryManager>(manifest_ids);

  encoded_subscription_ = this->create_subscription<auto_apms_behavior_codec_interfaces::msg::SerializedTreeMessage>(
    params.encoded_in_topic, 10, std::bind(&BehaviorTreeDecoderBase::encodedInCallback, this, std::placeholders::_1));
}

static std::map<std::string, std::string> get_port_map(
  const behavior_tree_representation::Node & node,
  std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dictionary_manager)
{
  std::map<std::string, std::string> port_map;

  auto dict_node = dictionary_manager->get_dictionary_info_by_name(node.type_name);

  //iterate throug all ports of the node
  for (const auto & port : node.ports) {
    if (!port) {
      RCLCPP_WARN(
        rclcpp::get_logger("behavior_tree_decoder"), "Null port pointer detected in node '%s'", node.type_name.c_str());
      continue;
    }

    //get Port name and value as string

    std::string port_name;
    std::string port_value;
    uint16_t port_id = port->getID();

    // get port name by checking dictionary entry with corresponding ID
    if (port_id < dict_node.port_types.size()) {
      port_name = dict_node.port_types[port_id].name;
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_tree_decoder"), "Port ID %u maps to name '%s' for node '%s'", port_id,
        port_name.c_str(), node.type_name.c_str());
    } 

    // if the port id is out of the range of port ids, check if the node is a SubTree, which allows for special port handling 
    else if (node.type_name == "SubTree") {
      // sub trees have one bool port called "_autoremap", check if the port currently beeing handled is this one. 
      // This check might need to be more robust
      if (std::dynamic_pointer_cast<behavior_tree_representation::PortBool>(port)) {
        port_name = "_autoremap";
      } 
      // otherwise use the "PortSubTreeSpecial" to handle the dynamic ports of Subtrees
      else if (
        auto port_subtree = std::dynamic_pointer_cast<behavior_tree_representation::PortSubTreeSpecial>(port)) {
        port_name = port_subtree->name;
      } 
      // the following code should not be reached, would log a warning
      else {
        RCLCPP_WARN(
          rclcpp::get_logger("behavior_tree_decoder"),
          "SubTree port ID %u out of range and not a known SubTree port type", port_id);
        continue;
      }
    } 
    
    // if non "SubTree" nodes have unexpected Ports, this is an error
    else {
      RCLCPP_WARN(
        rclcpp::get_logger("behavior_tree_decoder"), "Port ID %u out of range for node '%s' (has %zu ports)", port_id,
        node.type_name.c_str(), dict_node.port_types.size());
      continue;
    }

    //construct port value as string, depending on actual port type, as detected by its Runtime Type information. 
    // This could require a robust rework
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
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_tree_decoder"), "Handled SubTreeSpecial port: name='%s', value='%s'",
        port_subtree->name.c_str(), port_subtree->value.c_str());
    } else if (auto port_invalid = std::dynamic_pointer_cast<behavior_tree_representation::PortInvalid>(port)) {
      port_value = port_invalid->value;
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_tree_decoder"), "Handled Invalid port: name='%s', value='%s'", port_name.c_str(),
        port_value.c_str());
    } else if (auto port_node_status = std::dynamic_pointer_cast<behavior_tree_representation::PortNodeStatus>(port)) {
      port_value = port_node_status->string_value;
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_tree_decoder"), "Handled NodeStatus port: name='%s', value='%s'",
        port_name.c_str(), port_value.c_str());
    } else if (auto port_any_bt_any = std::dynamic_pointer_cast<behavior_tree_representation::PortAny>(port)) {
      port_value = port_any_bt_any->value;
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_tree_decoder"), "Handled BT::Any port: name='%s', value='%s'", port_name.c_str(),
        port_value.c_str());
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("behavior_tree_decoder"), "Unknown port type in node '%s'", node.type_name.c_str());
      continue;
    }
    // lastly add port name and value to the map
    port_map[port_name] = port_value;
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("behavior_tree_decoder"), "Port map for node '%s' has %zu entries", node.type_name.c_str(),
    port_map.size());
  return port_map;
}

static auto_apms_behavior_tree::core::TreeDocument::NodeElement recursiveNodeConstruction(
  const behavior_tree_representation::Node & node, auto_apms_behavior_tree::core::TreeDocument::NodeElement & base_node,
  std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dictionary_manager)
{
  RCLCPP_DEBUG(rclcpp::get_logger("behavior_tree_decoder"), "Constructing node '%s'", node.type_name.c_str());
  // get port map for the current node
  auto port_map = get_port_map(node, dictionary_manager);

  // construct XML element for the current node
  auto xml_element = base_node.getXMLElement();

  // transfer port map to XML element
  for (const auto & [port_name, port_value] : port_map) {
    xml_element->SetAttribute(port_name.c_str(), port_value.c_str());
    RCLCPP_DEBUG(
      rclcpp::get_logger("behavior_tree_decoder"), "Set port attribute '%s'='%s' on node '%s'", port_name.c_str(),
      port_value.c_str(), node.type_name.c_str());
  }

  // recursively construct child nodes
  for (const auto & child : node.children) {
    if (!child) {
      RCLCPP_WARN(
        rclcpp::get_logger("behavior_tree_decoder"), "Null child node pointer detected in parent '%s'",
        node.type_name.c_str());
      continue;
    }
    RCLCPP_DEBUG(
      rclcpp::get_logger("behavior_tree_decoder"), "Inserting child node '%s' into parent '%s'",
      child->type_name.c_str(), node.type_name.c_str());
    auto child_element = base_node.insertNode(child->type_name);
    recursiveNodeConstruction(*child, child_element, dictionary_manager);
    RCLCPP_DEBUG(
      rclcpp::get_logger("behavior_tree_decoder"), "Finished constructing child node '%s'", child->type_name.c_str());
  }

  RCLCPP_DEBUG(rclcpp::get_logger("behavior_tree_decoder"), "Finished constructing node '%s'", node.type_name.c_str());
  return base_node;
}

std::string BehaviorTreeDecoderBase::reconstructXML(const behavior_tree_representation::Document & document)
{
  //generate TreeDocument to conatain the decoded behavior tree(s)
  auto_apms_behavior_tree::core::TreeDocument tree_doc;
  tree_doc.registerNodes(this->dictionary_manager_->getNodeManifest());

  // add each tree in the behavior_tree_representation::Document to the output TreeDocument
  for (const auto & tree : document.trees) {
    RCLCPP_DEBUG(this->get_logger(), "Processing tree '%s' for XML reconstruction", tree.name.c_str());
    // if the tree beeing handled is the main tree to execute, make it the root tree in the TreeDocument
    if (tree.name == document.main_tree_to_execute) {
      getTreeElementFromTree(tree, tree_doc).makeRoot();
    } else {
      getTreeElementFromTree(tree, tree_doc);
    }
    RCLCPP_DEBUG(this->get_logger(), "Finished processing tree '%s'", tree.name.c_str());
    RCLCPP_DEBUG(
      this->get_logger(), "Current state of TreeDocument after processing tree '%s':\n%s", tree.name.c_str(),
      tree_doc.writeToString().c_str());
  }
  
  return tree_doc.writeToString();
}

auto_apms_behavior_tree::core::TreeDocument::TreeElement BehaviorTreeDecoderBase::getTreeElementFromTree(
  const behavior_tree_representation::Tree & tree, auto_apms_behavior_tree::core::TreeDocument & tree_doc)
{
  RCLCPP_INFO(this->get_logger(), "Constructing TreeElement for tree '%s'", tree.name.c_str());
  auto tree_element = tree_doc.newTree(tree.name);
  auto root = tree_element.insertNode(tree.root.type_name);
  recursiveNodeConstruction(tree.root, root, dictionary_manager_);
  RCLCPP_INFO(this->get_logger(), "Finished constructing TreeElement for tree '%s'", tree.name.c_str());
  return tree_element;
}

void BehaviorTreeDecoderBase::encodedInCallback(
  const auto_apms_behavior_codec_interfaces::msg::SerializedTreeMessage::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received encoded message of size %zu bytes", msg->serialized_tree_message.size());

  behavior_tree_representation::Document document;
  bool ok = document.deserialize(msg->serialized_tree_message, dictionary_manager_);

  if (ok) {
    RCLCPP_INFO(this->get_logger(), "Successfully deserialized message into Document");

    std::string xml_string = reconstructXML(document);

    if (!xml_string.empty()) {
      RCLCPP_INFO(this->get_logger(), "Reconstructed XML:\n%s", xml_string.c_str());
      onTreeDecoded(xml_string);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to reconstruct XML from Document");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to deserialize message into Document");
  }
}

}  // namespace auto_apms_behavior_codec
