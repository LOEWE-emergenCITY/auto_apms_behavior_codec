#include "auto_apms_behavior_codec/behavior_tree_representation.hpp"
#include "auto_apms_behavior_codec/dictionary_manager.hpp"
#include "cbor.h"

using namespace behavior_tree_representation;

// helper to parse a node recursively
static bool parse_node(CborValue* nodeVal, std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dict_mgr, Node& out_node)
{
  // nodeVal points to the node array element in the parent container
  if(!cbor_value_is_array(nodeVal)){
    std::cerr << "Expected node to be an array" << std::endl;
    return false;
  }

  size_t nodeArrayLen = 0;
  if(cbor_value_get_array_length(nodeVal, &nodeArrayLen) != CborNoError){
    std::cerr << "Failed to get node array length" << std::endl;
    return false;
  }

  CborValue nodeIt;
  if(cbor_value_enter_container(nodeVal, &nodeIt) != CborNoError){
    std::cerr << "Failed to enter node array" << std::endl;
    return false;
  }

  // first element: node type id (uint)
  uint64_t typeId = 0;
  if(cbor_value_get_uint64(&nodeIt, &typeId) != CborNoError){
    std::cerr << "Failed to read node type id" << std::endl;
    return false;
  }

  // lookup dictionary entry
  auto dict_entry = dict_mgr->get_dictionary_info_by_id(static_cast<uint32_t>(typeId));
  out_node.type_name = dict_entry.name;

  // advance nodeIt to the next element (ports array)
  if(cbor_value_advance(&nodeIt) != CborNoError){
    std::cerr << "Failed to advance to ports array" << std::endl;
    return false;
  }

  // parse ports array
  if(!cbor_value_is_array(&nodeIt)){
    std::cerr << "Expected ports array in node" << std::endl;
    return false;
  }

  size_t portsCount = 0;
  if(cbor_value_get_array_length(&nodeIt, &portsCount) != CborNoError){
    std::cerr << "Failed to get ports array length" << std::endl;
    return false;
  }

  CborValue portsIt;
  if(cbor_value_enter_container(&nodeIt, &portsIt) != CborNoError){
    std::cerr << "Failed to enter ports array" << std::endl;
    return false;
  }

  for(size_t pi = 0; pi < portsCount; ++pi){
    // first enter array for port elements
    if(!cbor_value_is_array(&portsIt)){
      std::cerr << "Port element is not an array" << std::endl;
      return false;
    }

    size_t portArrayLen = 0;
    if(cbor_value_get_array_length(&portsIt, &portArrayLen) != CborNoError){
      std::cerr << "Failed to get port array length" << std::endl;
      return false;
    }

    CborValue portIt;
    if(cbor_value_enter_container(&portsIt, &portIt) != CborNoError){
      std::cerr << "Failed to enter port array" << std::endl;
      return false;
    }

    // first element of port array always: port id
    uint64_t portId = 0;
    if(cbor_value_get_uint64(&portIt, &portId) != CborNoError){
      std::cerr << "Failed to read port id" << std::endl;
      return false;
    }
    if(cbor_value_advance(&portIt) != CborNoError){
      std::cerr << "Failed to advance after port id" << std::endl;
      return false;
    }

    // SubTree special encoding: [id, name, value] and only for SubTree nodes, if we are in a sub tree, we first decode the name and then the value as string
    if(portArrayLen == 3 && out_node.type_name == "SubTree"){
      // name
      size_t nLen = 0;
      if(cbor_value_get_string_length(&portIt, &nLen) != CborNoError){
        std::cerr << "Failed to get SubTree attribute name length" << std::endl;
        return false;
      }
      std::string attrName;
      attrName.resize(nLen);
      if(cbor_value_copy_text_string(&portIt, &attrName[0], &nLen, &portIt) != CborNoError){
        std::cerr << "Failed to read SubTree attribute name" << std::endl;
        return false;
      }

      // value (string)
      size_t vLen = 0;
      if(cbor_value_get_string_length(&portIt, &vLen) != CborNoError){
        std::cerr << "Failed to get SubTree attribute value length" << std::endl;
        return false;
      }
      std::string attrValue;
      attrValue.resize(vLen);
      if(cbor_value_copy_text_string(&portIt, &attrValue[0], &vLen, &portIt) != CborNoError){
        std::cerr << "Failed to read SubTree attribute value" << std::endl;
        return false;
      }

      out_node.ports.push_back(std::make_shared<PortSubTreeSpecial>(attrValue, attrName, static_cast<int16_t>(portId)));

      // leave port array container (updates portsIt)
      if(cbor_value_leave_container(&portsIt, &portIt) != CborNoError){
        std::cerr << "Failed to leave SubTree port container" << std::endl;
        return false;
      }

      continue;
    }

    // normal port: determine port type using dictionary entry
    std::string portType = "";
    if(portId < dict_entry.port_types.size()){
      portType = dict_entry.port_types[portId].type;
    }

    if(portType == "int"){
      int64_t v = 0;
      if(cbor_value_get_int64(&portIt, &v) != CborNoError){
        std::cerr << "Failed to read int port value" << std::endl;
        return false;
      }
      out_node.ports.push_back(std::make_shared<PortInt>(static_cast<int32_t>(v), static_cast<int16_t>(portId)));
    }
    else if(portType == "float"){
      double dv = 0.0;
      if(cbor_value_get_double(&portIt, &dv) != CborNoError){
        std::cerr << "Failed to read float port value" << std::endl;
        return false;
      }
      out_node.ports.push_back(std::make_shared<PortFloat>(static_cast<float>(dv), static_cast<int16_t>(portId)));
    }
    
    else if(portType == "std::string"){
      size_t svalLen = 0;
      if(cbor_value_get_string_length(&portIt, &svalLen) != CborNoError){
        std::cerr << "Failed to get string port length" << std::endl;
        return false;
      }
      std::string sval;
      sval.resize(svalLen);
      if(cbor_value_copy_text_string(&portIt, &sval[0], &svalLen, &portIt) != CborNoError){
        std::cerr << "Failed to read string port value" << std::endl;
        return false;
      }
      out_node.ports.push_back(std::make_shared<PortString>(sval, static_cast<int16_t>(portId)));
    }
    else if(portType == "BT::AnyTypeAllowed"){
      // The encoder always writes AnyTypeAllowed as a text string; read it as such.
      size_t svalLen = 0;
      if(cbor_value_get_string_length(&portIt, &svalLen) != CborNoError){
        std::cerr << "Failed to get AnyTypeAllowed string length" << std::endl;
        return false;
      }
      std::string sval;
      sval.resize(svalLen);
      if(cbor_value_copy_text_string(&portIt, &sval[0], &svalLen, &portIt) != CborNoError){
        std::cerr << "Failed to read AnyTypeAllowed string value" << std::endl;
        return false;
      }
      out_node.ports.push_back(std::make_shared<PortAnyTypeAllowed>(sval, static_cast<int16_t>(portId)));
    }
    else if(portType == "bool"){
      bool bv = false;
      if(cbor_value_get_boolean(&portIt, &bv) != CborNoError){
        std::cerr << "Failed to read bool port value" << std::endl;
        return false;
      }
      out_node.ports.push_back(std::make_shared<PortBool>(bv, static_cast<int16_t>(portId)));
    }
    else {
      // unknown type - try reading as text string, otherwise skip
      if(cbor_value_is_text_string(&portIt)){
        size_t svalLen = 0;
        if(cbor_value_get_string_length(&portIt, &svalLen) != CborNoError){
          std::cerr << "Failed to get unknown string-like port length" << std::endl;
          return false;
        }
        std::string sval;
        sval.resize(svalLen);
        if(cbor_value_copy_text_string(&portIt, &sval[0], &svalLen, &portIt) != CborNoError){
          std::cerr << "Failed to read unknown string-like port value" << std::endl;
          return false;
        }
        out_node.ports.push_back(std::make_shared<PortAnyTypeAllowed>(sval, static_cast<int16_t>(portId)));
      } else {
        // skip value
        if(cbor_value_advance(&portIt) != CborNoError){
          std::cerr << "Failed to skip unknown port value" << std::endl;
          return false;
        }
      }
    }

    // leave port array container (this advances portsIt)
    if(cbor_value_leave_container(&portsIt, &portIt) != CborNoError){
      std::cerr << "Failed to leave port container" << std::endl;
      return false;
    }
  }

  // leave ports array to advance nodeIt to next element (children or end)
  if(cbor_value_leave_container(&nodeIt, &portsIt) != CborNoError){
    std::cerr << "Failed to leave ports array" << std::endl;
    return false;
  }

  // now nodeIt points to next element; if it's an array treat as children
  if(cbor_value_is_array(&nodeIt)){
    size_t childrenCount = 0;
    if(cbor_value_get_array_length(&nodeIt, &childrenCount) != CborNoError){
      std::cerr << "Failed to get children array length" << std::endl;
      return false;
    }
    CborValue childrenIt;
    if(cbor_value_enter_container(&nodeIt, &childrenIt) != CborNoError){
      std::cerr << "Failed to enter children array" << std::endl;
      return false;
    }
    for(size_t ci = 0; ci < childrenCount; ++ci){
      Node child;
      if(!parse_node(&childrenIt, dict_mgr, child)){
        std::cerr << "Failed to parse child node" << std::endl;
        return false;
      }
      out_node.children.push_back(std::make_shared<Node>(child));
      // childrenIt is updated by parse_node (it consumes the child and advances to next)
    }
    // leave children array
    if(cbor_value_leave_container(&nodeIt, &childrenIt) != CborNoError){
      std::cerr << "Failed to leave children array" << std::endl;
      return false;
    }
  }

  // leave the node container to advance the original iterator (parent)
  if(cbor_value_leave_container(nodeVal, &nodeIt) != CborNoError){
    std::cerr << "Failed to leave node container" << std::endl;
    return false;
  }

  return true;
}

bool Document::deserialize(const std::vector<uint8_t>& data, std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dictionary_manager){
  CborParser parser;
  CborValue it;
  if(cbor_parser_init(data.data(), data.size(), 0, &parser, &it) != CborNoError){
    std::cerr << "Failed to init CBOR parser" << std::endl;
    return false;
  }

  if(!cbor_value_is_array(&it)){
    std::cerr << "Top-level CBOR value is not an array" << std::endl;
    return false;
  }

  size_t treesCount = 0;
  if(cbor_value_get_array_length(&it, &treesCount) != CborNoError){
    std::cerr << "Failed to get trees array length" << std::endl;
    return false;
  }

  CborValue treesIt;
  if(cbor_value_enter_container(&it, &treesIt) != CborNoError){
    std::cerr << "Failed to enter trees array" << std::endl;
    return false;
  }

  for(size_t ti = 0; ti < treesCount; ++ti){
    // each tree is an array [name, root]
    if(!cbor_value_is_array(&treesIt)){
      std::cerr << "Tree element is not an array" << std::endl;
      return false;
    }
    size_t treeLen = 0;
    if(cbor_value_get_array_length(&treesIt, &treeLen) != CborNoError){
      std::cerr << "Failed to get tree array length" << std::endl;
      return false;
    }

    CborValue treeIt;
    if(cbor_value_enter_container(&treesIt, &treeIt) != CborNoError){
      std::cerr << "Failed to enter tree array" << std::endl;
      return false;
    }

    // read name
    size_t nameLen = 0;
    if(cbor_value_get_string_length(&treeIt, &nameLen) != CborNoError){
      std::cerr << "Failed to get tree name length" << std::endl;
      return false;
    }
    std::string treeName;
    treeName.resize(nameLen);
    if(cbor_value_copy_text_string(&treeIt, &treeName[0], &nameLen, &treeIt) != CborNoError){
      std::cerr << "Failed to read tree name" << std::endl;
      return false;
    }

    Tree t;
    t.name = treeName;

    // next element is root node; parse_node will consume the node element and advance treeIt
    if(!parse_node(&treeIt, dictionary_manager, t.root)){
      std::cerr << "Failed to parse root node" << std::endl;
      return false;
    }

    // leave tree array to advance treesIt
    if(cbor_value_leave_container(&treesIt, &treeIt) != CborNoError){
      std::cerr << "Failed to leave tree array" << std::endl;
      return false;
    }

    trees.push_back(t);
  }

  // leaving trees array (not strictly necessary as parser will be discarded)
  // cbor_value_leave_container(&it, &treesIt);

  return true;
}
