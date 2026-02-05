#include "dictionary_manager.hpp"

#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include <set>

using namespace auto_apms_behavior_codec;

bool DictionaryManager::build_dictionary()
{
  // begin by getting all known node types from the core
  std::set<auto_apms_behavior_tree::core::NodeManifestResourceIdentity> known_node_ids = auto_apms_behavior_tree::core::getNodeManifestResourceIdentities();

  // for debugging, print the number of known node types -> currently only 0 -> why?
  std::cout << "Building dictionary with " << known_node_ids.size() << " known node IDs." << std::endl;

  return false;
}

DictionaryManager::DictionaryManager()
{
  //constructor must build the dictionary
  build_dictionary();
}