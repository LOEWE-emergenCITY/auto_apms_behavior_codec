#include "dictionary_manager.hpp"

#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include <set>
#include <iostream>
#include <map>
#include "rclcpp/rclcpp.hpp"

using namespace auto_apms_behavior_codec;

//currently called 2 times, needs debugging
bool DictionaryManager::build_dictionary()
{
  // begin by getting all known node types from the core
  std::set<auto_apms_behavior_tree::core::NodeManifestResourceIdentity> known_node_manifests = auto_apms_behavior_tree::core::getNodeManifestResourceIdentities();

  // for debugging, print the number of known node types -> currently only 0 -> why?
  std::cout << "Building dictionary from " << known_node_manifests.size() << " known node manifests." << std::endl;

  // iterate through all node ids and get their models
  uint32_t id_counter = 0;
  uint16_t unsupported_nodes = 0;
  for (const auto & node_id : known_node_manifests) {
    try {
      auto_apms_behavior_tree::core::NodeManifestResource manifest_resource = auto_apms_behavior_tree::core::NodeManifestResource(node_id);
      auto node_models = manifest_resource.getNodeModel();
      std::map<std::string , auto_apms_behavior_tree::NodeModel>::iterator nodes_iterator;
      for(nodes_iterator = node_models.begin(); nodes_iterator != node_models.end(); nodes_iterator++) {
        bool has_unsupported_parameter = false;
        uint16_t number_int_params = 0;
        uint16_t number_float_params = 0;
        uint16_t number_string_params = 0;
        uint16_t number_bool_params = 0;
        auto_apms_behavior_tree::NodeModel model = nodes_iterator->second;
        std::vector<auto_apms_behavior_tree::NodePortInfo> port_infos = model.port_infos;
        for (const auto & port_info : port_infos) {
          if(supported_parameter_types_.find(port_info.port_type) == supported_parameter_types_.end()) {
            RCLCPP_WARN(rclcpp::get_logger("DictionaryManager"), "Node %s has unsupported parameter type %s for port %s. This node will not be supported for encoding/decoding.", nodes_iterator->first.c_str(), port_info.port_type.c_str(), port_info.port_name.c_str());
            has_unsupported_parameter = true;
            unsupported_nodes++;
            break;
          }
          else {
            if(port_info.port_type == "int") {
              number_int_params++;
            }
            else if(port_info.port_type == "float") {
              number_float_params++;
            }
            else if(port_info.port_type == "std::string") {
              number_string_params++;
            }
            else if(port_info.port_type == "bool") {
              number_bool_params++;
            }
          }
          // for debugging, print port info
          //std::cout << "  Port Name: " << port_info.port_name << ", Type: " << port_info.port_type << ", Default: " << port_info.port_default << ", Has Default: " << port_info.port_has_default << ", Description: " << port_info.port_description << ", Direction: " << static_cast<int>(port_info.port_direction) << std::endl;
        }
        //std::cout << "Node Type: " << model.type<< std::endl;
        DictionaryInfo info(!has_unsupported_parameter, id_counter++, nodes_iterator->first, number_int_params, number_float_params, number_string_params, number_bool_params);
        //std::cout << "Added node to dictionary: " << info.name << " with ID " << info.id << " and " << info.number_int_params << " int params, " << info.number_float_params << " float params, " << info.number_string_params << " string params, " << info.number_bool_params << " bool params" << std::endl;
        dictionary_map_.insert({info.name, info});
      }
      //auto manifest = auto_apms_behavior_tree::core::NodeManifest::fromResource(node_id);
      //std::vector<std::string> node_names= manifest.getNodeNames();
      //for (const auto & node_name : node_names) {
      //  DictionaryInfo info;
      //  info.id = id_counter++;
      //  info.name = node_name;
      //  dictionary_map_[info.name] = info;
        //todo parameters
      //  std::cout << "Added node to dictionary: " << info.name << " with ID " << info.id << std::endl;
      //}
    } catch (const std::exception & e) {
      std::cerr << "Error loading node manifest " << std::endl;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("DictionaryManager"), "Finished building dictionary. Total nodes: %u, Unsupported nodes: %u", id_counter-1, unsupported_nodes);

  //next get node modells for all ids
  return false;
}

DictionaryManager::DictionaryManager()
{
  //constructor must build the dictionary
  build_dictionary();
}

DictionaryInfo::DictionaryInfo(bool supported, uint32_t id, std::string name, int number_int_params, int number_float_params, int number_string_params, int number_bool_params)
{
  this->supported = supported;
  this->id = id;
  this->name = name;
  this->number_int_params = number_int_params;
  this->number_float_params = number_float_params;
  this->number_string_params = number_string_params;
  this->number_bool_params = number_bool_params;
}