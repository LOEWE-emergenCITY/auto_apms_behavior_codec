#include "dictionary_manager.hpp"

#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include <set>
#include <iostream>

using namespace auto_apms_behavior_codec;

bool DictionaryManager::build_dictionary()
{
  // begin by getting all known node types from the core
  std::set<auto_apms_behavior_tree::core::NodeManifestResourceIdentity> known_node_ids = auto_apms_behavior_tree::core::getNodeManifestResourceIdentities();

  // for debugging, print the number of known node types -> currently only 0 -> why?
  std::cout << "Building dictionary with " << known_node_ids.size() << " known node IDs." << std::endl;

  // iterate through all node ids and get their models
  uint32_t id_counter = 0;
  for (const auto & node_id : known_node_ids) {
    try {
      auto manifest = auto_apms_behavior_tree::core::NodeManifest::fromResource(node_id);
      std::vector<std::string> node_names= manifest.getNodeNames();
      for (const auto & node_name : node_names) {
        DictionaryInfo info;
        info.id = id_counter++;
        info.name = node_name;
        dictionary_map_[info.name] = info;
        //todo parameters
        std::cout << "Added node to dictionary: " << info.name << " with ID " << info.id << std::endl;
      }
    } catch (const std::exception & e) {
      std::cerr << "Error loading node manifest " << std::endl;
    }
  }

  //next get node modells for all ids
  return false;
}

DictionaryManager::DictionaryManager()
{
  //constructor must build the dictionary
  build_dictionary();
}

DictionaryInfo::DictionaryInfo()
{}