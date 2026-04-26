#include "auto_apms_behavior_codec/dictionary_manager.hpp"

#include <iostream>
#include <map>
#include <set>

#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace auto_apms_behavior_codec;

bool DictionaryManager::build_dictionary(
  const std::vector<auto_apms_behavior_tree::core::NodeManifestResourceIdentity> & manifest_ids)
{
  // iterate through all node ids and get their models
  uint32_t id_counter = 0;
  uint16_t unsupported_nodes = 0;

  // getting native nodes seems to be required seperately, do this first
  auto_apms_behavior_tree::NodeModelMap nodeModels = auto_apms_behavior_tree::core::getNativeNodeModel();
  RCLCPP_INFO(rclcpp::get_logger("DictionaryManager"), "Found %zu native nodes.", nodeModels.size());

  // iterrate trough native nodes and add them to dictionary
  std::map<std::string, auto_apms_behavior_tree::NodeModel>::iterator nodes_iterator;
  for (nodes_iterator = nodeModels.begin(); nodes_iterator != nodeModels.end(); nodes_iterator++) {
    bool has_unsupported_parameter = false;
    auto_apms_behavior_tree::NodeModel model = nodes_iterator->second;
    std::vector<auto_apms_behavior_tree::NodePortInfo> port_infos = model.port_infos;
    std::vector<NodePortType> port_types;

    // iterate through ports, check if they are supported and add them to the node dictionary entry
    for (const auto & port_info : port_infos) {
      NodePortType node_port_type;
      node_port_type.name = port_info.port_name;
      if (supported_parameter_types_.find(port_info.port_type) == supported_parameter_types_.end()) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("DictionaryManager"),
          "Node %s has unsupported parameter type %s for port %s. Will encode as 'invalid' port.",
          nodes_iterator->first.c_str(), port_info.port_type.c_str(), port_info.port_name.c_str());
        // Mark unsupported ports with the "invalid" type for fallback encoding
        node_port_type.type = "invalid";
        node_port_type.supported = false;
        has_unsupported_parameter = true;
      } else {
        node_port_type.type = port_info.port_type;
        node_port_type.supported = true;
        RCLCPP_DEBUG(
          rclcpp::get_logger("DictionaryManager"), "Node %s has supported parameter type %s for port %s.",
          nodes_iterator->first.c_str(), port_info.port_type.c_str(), port_info.port_name.c_str());
      }
      if(has_unsupported_parameter) {
        unsupported_nodes++;
      }
      port_types.push_back(node_port_type);
      // for debugging, print port info
      // std::cout << "  Port Name: " << port_info.port_name << ", Type: " << port_info.port_type << ", Default: " <<
      // port_info.port_default << ", Has Default: " << port_info.port_has_default << ", Description: " <<
      // port_info.port_description << ", Direction: " << static_cast<int>(port_info.port_direction) << std::endl;
    }
    // std::cout << "Node Type: " << model.type<< std::endl;
    // Always add nodes to dictionary, even if they have unsupported ports (they will be encoded using "invalid" type)
    DictionaryNode info(!has_unsupported_parameter, id_counter++, nodes_iterator->first, port_types);
    // std::cout << "Added node to dictionary: " << info.name << " with ID " << info.id << " and " <<
    // info.number_int_params << " int params, " << info.number_float_params << " float params, " <<
    // info.number_string_params << " string params, " << info.number_bool_params << " bool params" << std::endl;
    dictionary_map_.insert({info.name, info});
  }

  // now handle non native nodes

  std::cout << "Building dictionary from " << manifest_ids.size() << " node manifests." << std::endl;

  for (const auto & manifest_id : manifest_ids) {
    try {
      // get node manifest resource for id
      auto_apms_behavior_tree::core::NodeManifestResource manifest_resource =
        auto_apms_behavior_tree::core::NodeManifestResource(manifest_id);

      this->manifests.push_back(manifest_resource.getNodeManifest());

      // get node models from manifest
      auto node_models = manifest_resource.getNodeModel();
      std::map<std::string, auto_apms_behavior_tree::NodeModel>::iterator nodes_iterator;

      // iterate through node models and add to dictionary
      for (nodes_iterator = node_models.begin(); nodes_iterator != node_models.end(); nodes_iterator++) {
        bool has_unsupported_parameter = false;
        auto_apms_behavior_tree::NodeModel model = nodes_iterator->second;
        std::vector<auto_apms_behavior_tree::NodePortInfo> port_infos = model.port_infos;
        std::vector<NodePortType> port_types;

        // iterate through ports, check if they are supported and add them to the node dictionary entry
        for (const auto & port_info : port_infos) {
          NodePortType node_port_type;
          node_port_type.name = port_info.port_name;
          
          if (supported_parameter_types_.find(port_info.port_type) == supported_parameter_types_.end()) {
            RCLCPP_WARN(
              rclcpp::get_logger("DictionaryManager"),
                "Node %s has unsupported parameter type %s for port %s. Will encode as 'invalid' port.",
              nodes_iterator->first.c_str(), port_info.port_type.c_str(), port_info.port_name.c_str());
            // Mark unsupported ports with the "invalid" type for fallback encoding
            node_port_type.type = "invalid";
            node_port_type.supported = false;
            has_unsupported_parameter = true;
          } else {
            node_port_type.type = port_info.port_type;
            node_port_type.supported = true;
            RCLCPP_DEBUG(
              rclcpp::get_logger("DictionaryManager"), "Node %s has supported parameter type %s for port %s.",
              nodes_iterator->first.c_str(), port_info.port_type.c_str(), port_info.port_name.c_str());
          }
          if(has_unsupported_parameter) {
            unsupported_nodes++;
          }
          port_types.push_back(node_port_type);
          // for debugging, print port info
          // std::cout << "  Port Name: " << port_info.port_name << ", Type: " << port_info.port_type << ", Default: "
          // << port_info.port_default << ", Has Default: " << port_info.port_has_default << ", Description: " <<
          // port_info.port_description << ", Direction: " << static_cast<int>(port_info.port_direction) << std::endl;
        }
        // std::cout << "Node Type: " << model.type<< std::endl;
        // Always add nodes to dictionary, even if they have unsupported ports (they will be encoded using "invalid" type)
        DictionaryNode info(!has_unsupported_parameter, id_counter++, nodes_iterator->first, port_types);
        // std::cout << "Added node to dictionary: " << info.name << " with ID " << info.id << " and " <<
        // info.number_int_params << " int params, " << info.number_float_params << " float params, " <<
        // info.number_string_params << " string params, " << info.number_bool_params << " bool params" << std::endl;
        dictionary_map_.insert({info.name, info});
      }

    } catch (const std::exception & e) {
      std::cerr << "Error loading node manifest " << std::endl;
    }
  }
  RCLCPP_INFO(
    rclcpp::get_logger("DictionaryManager"),
    "Finished building dictionary. Total nodes: %u, Unsupported nodes: %u, Unsupported nodes will be encoded using 'invalid' port fallback.",
    id_counter - 1, unsupported_nodes);

  // next get node modells for all ids
  return false;
}

DictionaryManager::DictionaryManager(
  const std::vector<auto_apms_behavior_tree::core::NodeManifestResourceIdentity> & manifest_ids)
{
  // constructor must build the dictionary
  RCLCPP_INFO(rclcpp::get_logger("DictionaryManager"), "Initializing DictionaryManager and building dictionary...");
  if (manifest_ids.empty()) {
    std::set<auto_apms_behavior_tree::core::NodeManifestResourceIdentity> all_registered_manifests =
      auto_apms_behavior_tree::core::getNodeManifestResourceIdentities();
    build_dictionary(
      std::vector<auto_apms_behavior_tree::core::NodeManifestResourceIdentity>(
        all_registered_manifests.begin(), all_registered_manifests.end()));
  } else {
    build_dictionary(manifest_ids);
  }
}

DictionaryNode DictionaryManager::get_dictionary_info_by_name(const std::string & dictionary_name)
{
  auto it = dictionary_map_.find(dictionary_name);
  if (it != dictionary_map_.end()) {
    return it->second;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("DictionaryManager"), "Node name %s not found in dictionary.", dictionary_name.c_str());
    return DictionaryNode(false, 0, dictionary_name);  // return unsupported node with id 0
  }
}

DictionaryNode DictionaryManager::get_dictionary_info_by_id(uint32_t dictionary_id)
{
  for (const auto & entry : this->dictionary_map_) {
    if (entry.second.id == dictionary_id) {
      return entry.second;
    }
  }
  RCLCPP_WARN(rclcpp::get_logger("DictionaryManager"), "Dictionary id %u not found.", dictionary_id);
  return DictionaryNode(false, 0, std::string("unknown"));
}

void DictionaryManager::print_dictionary()
{
  std::cout << print_dictionary_to_string();
}

std::string DictionaryManager::print_dictionary_to_string() const
{
  std::ostringstream oss;
  oss << "Dictionary contents (" << dictionary_map_.size() << " nodes):\n";
  for (const auto & [name, node] : dictionary_map_) {
    oss << "  [" << node.id << "] " << node.name
        << " (supported=" << (node.supported ? "true" : "false")
        << ", " << node.port_types.size() << " port(s))\n";
    for (size_t i = 0; i < node.port_types.size(); ++i) {
      oss << "    port[" << i << "] name='" << node.port_types[i].name
          << "' type='" << node.port_types[i].type << "'\n";
    }
  }
  return oss.str();
}

DictionaryNode::DictionaryNode(bool supported, uint32_t id, std::string name, std::vector<NodePortType> port_types)
{
  this->supported = supported;
  this->id = id;
  this->name = name;
  this->port_types = port_types;
}

std::vector<auto_apms_behavior_tree::core::NodeManifest> DictionaryManager::getNodeManifests()
{
  return this->manifests;
}

auto_apms_behavior_tree::core::NodeManifest DictionaryManager::getNodeManifest()
{
  auto_apms_behavior_tree::core::NodeManifest result;
  for (const auto & entry : this->manifests) {
    result.merge(entry, true);
  }
  return result;
}