#pragma once
#include <map>
#include <string>
#include <list>
#include <vector>

#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
namespace auto_apms_behavior_codec
{
  //set to lookup supported types when analysing port infos
  static std::set<std::string> supported_parameter_types_ = {"int", "unsigned int", "float", "double", "std::string", "bool", "BT::AnyTypeAllowed", "BT::Any" , "BT::NodeStatus"};

  //each node can have multiple ports, this struct represents the name and type of a port for a node in the dictionary
  //the id of the port is its index in the vector of ports for the node, this is may differ between ports with same name/type, if they belong to different nodes
  struct NodePortType{
    public:
      std::string name;
      std::string type;
  };

  // This Class represents a node in the dictionary, i.e., a node type of the tree with its parameters
  class DictionaryNode
  {
  public:
      bool supported;
      DictionaryNode(bool supported, uint32_t id, std::string name, std::vector<NodePortType> port_types = {});
      ~DictionaryNode() = default;
      uint32_t id;
      std::string name;
      std::vector<NodePortType> port_types;
  };

  // This Class manages the dictionary of known node types
  class DictionaryManager
  {
  public:
      /**
       * @brief Constructor.
       * @param manifest_ids Node manifests to use for the dictionary. If empty, will use all registered node manifests.
       */
      DictionaryManager(const std::vector<auto_apms_behavior_tree::core::NodeManifestResourceIdentity> & manifest_ids = {});
      ~DictionaryManager() = default;

      // function to get a DictionaryNode by its ID -> required for decoding
      DictionaryNode get_dictionary_info_by_id(uint32_t dictionary_id);

      // function to get a DictionaryNode by its name -> required for encoding
      DictionaryNode get_dictionary_info_by_name(const std::string& dictionary_name);

      void print_dictionary();

      // get all node manifests used to build this dictionary
      std::vector<auto_apms_behavior_tree::core::NodeManifest> getNodeManifests();

      //get a mereged manifest of all nodes handled by this dictionary
      auto_apms_behavior_tree::core::NodeManifest getNodeManifest();

    private:
      // gets known node types and builds dictionary
      bool build_dictionary(const std::vector<auto_apms_behavior_tree::core::NodeManifestResourceIdentity> & manifest_ids);

      //store dictionary entries
      std::map<std::string, DictionaryNode> dictionary_map_;

      //stores the node manifests used to build the dictionary
      std::vector<auto_apms_behavior_tree::core::NodeManifest> manifests;
  };

}  // namespace auto_apms_behavior_codec