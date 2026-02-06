#pragma once
#include <map>
#include <string>
#include <list>

#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
namespace auto_apms_behavior_codec
{
  //set to lookup supported types when analysing port infos
  static std::set<std::string> supported_parameter_types_ = {"int", "float", "std::string", "bool"};

  // This Class represents a node in the dictionary, i.e., a node type of the tree with its parameters
  class DictionaryInfo
  {
  public:
      bool supported;
      DictionaryInfo(bool supported, uint32_t id, std::string name, int number_int_params = 0, int number_float_params = 0, int number_string_params = 0, int number_bool_params = 0);
      ~DictionaryInfo() = default;
      uint32_t id;
      std::string name;
      uint16_t number_int_params;
      uint16_t number_float_params;
      uint16_t number_string_params;
      uint16_t number_bool_params;
      std::list<uint32_t> int_params;
      std::list<float> float_params;
      std::list<std::string> string_params;
      std::list<bool> bool_params;


  };

  // This Class manages the dictionary of known node types
  class DictionaryManager
  {
  public:
      DictionaryManager();
      ~DictionaryManager() = default;

      // function to get a DictionaryInfo by its ID -> required for decoding
      DictionaryInfo get_dictionary_info_by_id(uint32_t dictionary_id);

      // function to get a DictionaryInfo by its name -> required for encoding
      DictionaryInfo get_dictionary_info_by_name(const std::string& dictionary_name);

  private:
    // gets known node types and builds dictionary
    bool build_dictionary();

    //store dictionary entries
    std::map<std::string, DictionaryInfo> dictionary_map_;
  };

}  // namespace auto_apms_behavior_codec