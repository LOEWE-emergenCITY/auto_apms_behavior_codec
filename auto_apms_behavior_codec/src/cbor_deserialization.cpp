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
  std::cerr << "[CBOR DEBUG] parse_node: typeId=" << typeId << " name='" << out_node.type_name << "' nodeArrayLen=" << nodeArrayLen << std::endl;

  // advance nodeIt to the next element (ports array)
  if(cbor_value_advance(&nodeIt) != CborNoError){
    std::cerr << "Failed to advance to ports array" << std::endl;
    return false;
  }
  std::cerr << "[CBOR DEBUG] After advance past typeId, nodeIt type=" << cbor_value_get_type(&nodeIt) << " is_array=" << cbor_value_is_array(&nodeIt) << std::endl;

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
    std::cerr << "[CBOR DEBUG] Parsing port index " << pi << ", CBOR type: " << cbor_value_get_type(&portsIt) << std::endl;
    if (cbor_value_at_end(&portsIt)) {
      std::cerr << "[CBOR DEBUG] Reached end of ports array at index " << pi << std::endl;
      break;
    }
    if(!cbor_value_is_array(&portsIt)){
      std::cerr << "[CBOR DEBUG] Port element is not an array at index " << pi << ", CBOR type: " << cbor_value_get_type(&portsIt) << std::endl;
      return false;
    }
    size_t portArrayLen = 0;
    if(cbor_value_get_array_length(&portsIt, &portArrayLen) != CborNoError){
      std::cerr << "[CBOR DEBUG] Failed to get port array length at index " << pi << std::endl;
      return false;
    }
    std::cerr << "[CBOR DEBUG] Port array length at index " << pi << ": " << portArrayLen << std::endl;
    CborValue portIt;
    if(cbor_value_enter_container(&portsIt, &portIt) != CborNoError){
      std::cerr << "[CBOR DEBUG] Failed to enter port array at index " << pi << std::endl;
      return false;
    }
    CborType firstElemType = cbor_value_get_type(&portIt);
    std::cerr << "[CBOR DEBUG] Node '" << out_node.type_name << "' port index " << pi << ": first elem type=" << firstElemType << " array_len=" << portArrayLen << std::endl;
    if(firstElemType != CborIntegerType) {
      // [string, port_index] -- INVALID PORT
      size_t svalLen = 0;
      if(cbor_value_get_string_length(&portIt, &svalLen) != CborNoError){
        std::cerr << "[CBOR DEBUG] Failed to get invalid port string length at index " << pi << std::endl;
        cbor_value_leave_container(&portsIt, &portIt);
        return false;
      }
      std::string sval(svalLen, '\0');
      if(cbor_value_copy_text_string(&portIt, &sval[0], &svalLen, &portIt) != CborNoError){
        std::cerr << "[CBOR DEBUG] Failed to read invalid port string value at index " << pi << std::endl;
        cbor_value_leave_container(&portsIt, &portIt);
        return false;
      }
      int64_t portId = 0;
      CborType idxType = cbor_value_get_type(&portIt);
      if(idxType == CborIntegerType) {
        if(cbor_value_get_int64(&portIt, &portId) != CborNoError){
          std::cerr << "[CBOR DEBUG] Failed to read invalid port index value at index " << pi << std::endl;
          cbor_value_leave_container(&portsIt, &portIt);
          return false;
        }
        // MUST advance portIt past the integer so it reaches CborInvalidType before leave_container
        if(cbor_value_advance(&portIt) != CborNoError && !cbor_value_at_end(&portIt)){
          std::cerr << "[CBOR DEBUG] Failed to advance past invalid port index at index " << pi << std::endl;
          cbor_value_leave_container(&portsIt, &portIt);
          return false;
        }
      } else {
        std::cerr << "[CBOR DEBUG] Invalid port index is not an integer at index " << pi << std::endl;
        cbor_value_leave_container(&portsIt, &portIt);
        return false;
      }
      std::cerr << "[CBOR DEBUG] Decoded invalid port: '" << sval << "', index: " << portId << std::endl;
      out_node.ports.push_back(std::make_shared<PortInvalid>(sval, static_cast<int16_t>(portId)));
      // leave port array container - portIt must be at CborInvalidType (end of container)
      if(cbor_value_leave_container(&portsIt, &portIt) != CborNoError){
        std::cerr << "[CBOR DEBUG] Failed to leave port container at index " << pi << std::endl;
        return false;
      }
      std::cerr << "[CBOR DEBUG] After leave_container (invalid), next CBOR type at port index " << (pi+1) << ": " << cbor_value_get_type(&portsIt) << std::endl;
      // No advance or continue here! Let the loop advance naturally for all port types
      continue;
    } else {
      // first element of valid port array always: port id
      uint64_t portId = 0;
      if(cbor_value_get_uint64(&portIt, &portId) != CborNoError){
        std::cerr << "[CBOR DEBUG] Failed to read port id at index " << pi << std::endl;
        return false;
      }
      if(cbor_value_advance(&portIt) != CborNoError){
        std::cerr << "[CBOR DEBUG] Failed to advance after port id at index " << pi << std::endl;
        return false;
      }

      // SubTree special encoding:
      // - [id, name, value] (3-element): named attribute (e.g. ID, x, y, z, drone, ...)
      // - [id, bool] (2-element): _autoremap flag encoded as PortBool
      if(out_node.type_name == "SubTree"){
        if(portArrayLen == 3){
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
          std::cerr << "[CBOR DEBUG] Decoded SubTree port: name='" << attrName << "' value='" << attrValue << "' id=" << portId << std::endl;
        } else {
          // 2-element SubTree port: must be the _autoremap bool flag
          bool bv = false;
          if(cbor_value_get_boolean(&portIt, &bv) != CborNoError){
            std::cerr << "Failed to read SubTree _autoremap bool value" << std::endl;
            return false;
          }
          if(cbor_value_advance(&portIt) != CborNoError){
            std::cerr << "Failed to advance after SubTree _autoremap bool" << std::endl;
            return false;
          }
          out_node.ports.push_back(std::make_shared<PortBool>(bv, static_cast<int16_t>(portId)));
          std::cerr << "[CBOR DEBUG] Decoded SubTree _autoremap port: value=" << bv << " id=" << portId << std::endl;
        }
        // leave port array container (updates portsIt)
        if(cbor_value_leave_container(&portsIt, &portIt) != CborNoError){
          std::cerr << "Failed to leave SubTree port container" << std::endl;
          return false;
        }
        continue; // already left container above, skip shared leave_container at bottom
      } else {
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
          if(cbor_value_advance(&portIt) != CborNoError){
            std::cerr << "Failed to advance after int port value" << std::endl;
            return false;
          }
          out_node.ports.push_back(std::make_shared<PortInt>(static_cast<int32_t>(v), static_cast<int16_t>(portId)));
        }
        else if(portType == "unsigned int"){
          uint64_t v = 0;
          if(cbor_value_get_uint64(&portIt, &v) != CborNoError){
            std::cerr << "Failed to read unsigned int port value" << std::endl;
            return false;
          }
          if(cbor_value_advance(&portIt) != CborNoError){
            std::cerr << "Failed to advance after unsigned int port value" << std::endl;
            return false;
          }
          out_node.ports.push_back(std::make_shared<PortUInt>(static_cast<uint32_t>(v), static_cast<int16_t>(portId)));
        }
        else if(portType == "float"){
          double dv = 0.0;
          if(cbor_value_get_double(&portIt, &dv) != CborNoError){
            std::cerr << "Failed to read float port value" << std::endl;
            return false;
          }
          if(cbor_value_advance(&portIt) != CborNoError){
            std::cerr << "Failed to advance after float port value" << std::endl;
            return false;
          }
          out_node.ports.push_back(std::make_shared<PortFloat>(static_cast<float>(dv), static_cast<int16_t>(portId)));
        }
        else if(portType == "double"){
          double dv = 0.0;
          if(cbor_value_get_double(&portIt, &dv) != CborNoError){
            std::cerr << "Failed to read double port value" << std::endl;
            return false;
          }
          if(cbor_value_advance(&portIt) != CborNoError){
            std::cerr << "Failed to advance after double port value" << std::endl;
            return false;
          }
          out_node.ports.push_back(std::make_shared<PortDouble>(dv, static_cast<int16_t>(portId)));
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
        else if(portType == "BT::Any"){
          // The encoder always writes BT::Any as a text string; read it as such.
          size_t svalLen = 0;
          if(cbor_value_get_string_length(&portIt, &svalLen) != CborNoError){
            std::cerr << "Failed to get BT::Any string length" << std::endl;
            return false;
          }
          std::string sval;
          sval.resize(svalLen);
          if(cbor_value_copy_text_string(&portIt, &sval[0], &svalLen, &portIt) != CborNoError){
            std::cerr << "Failed to read BT::Any string value" << std::endl;
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
          if(cbor_value_advance(&portIt) != CborNoError){
            std::cerr << "Failed to advance after bool port value" << std::endl;
            return false;
          }
          out_node.ports.push_back(std::make_shared<PortBool>(bv, static_cast<int16_t>(portId)));
        }
        else if(portType == "BT::NodeStatus"){
          uint64_t value = 10; //initialize invalid
          if(cbor_value_get_uint64(&portIt, &value) != CborNoError){
            std::cerr << "Failed to read NodeStatus port value" << std::endl;
            return false;
          }
          if(cbor_value_advance(&portIt) != CborNoError){
            std::cerr << "Failed to advance after NodeStatus port value" << std::endl;
            return false;
          }
          out_node.ports.push_back(std::make_shared<PortNodeStatus>(PortNodeStatus::getEnumString(static_cast<BT::NodeStatus>(value)), static_cast<int16_t>(portId)));
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
      }
    }

    // leave port array container (always, for both valid and invalid ports)
    if(cbor_value_leave_container(&portsIt, &portIt) != CborNoError){
      std::cerr << "[CBOR DEBUG] Failed to leave port container at index " << pi << std::endl;
      return false;
    }
    std::cerr << "[CBOR DEBUG] After leave_container, next CBOR type at port index " << (pi+1) << ": " << cbor_value_get_type(&portsIt) << std::endl;
    // No advance or continue here! Let the loop and iterator advance naturally for all port types.
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

bool Document::deserialize(const std::vector<uint8_t>& data, std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dictionary_manager) {
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

  size_t topArrayLen = 0;
  if(cbor_value_get_array_length(&it, &topArrayLen) != CborNoError){
    std::cerr << "Failed to get top-level array length" << std::endl;
    return false;
  }
  if(topArrayLen < 1){
    std::cerr << "Top-level array must have at least one element (the boolean)" << std::endl;
    return false;
  }

  CborValue arrIt;
  if(cbor_value_enter_container(&it, &arrIt) != CborNoError){
    std::cerr << "Failed to enter top-level array" << std::endl;
    return false;
  }

  // First element: boolean
  bool was_main_tree_empty_cbor = false;
  if(!cbor_value_is_boolean(&arrIt)){
    std::cerr << "Expected boolean as first element in CBOR array" << std::endl;
    return false;
  }
  if(cbor_value_get_boolean(&arrIt, &was_main_tree_empty_cbor) != CborNoError){
    std::cerr << "Failed to read first boolean (was_main_tree_empty_cbor)" << std::endl;
    return false;
  }
  std::cout << "Decoded CBOR: was_main_tree_empty = " << was_main_tree_empty_cbor << std::endl;

  // Advance to the first tree
  if(cbor_value_advance(&arrIt) != CborNoError){
    std::cerr << "Failed to advance to first tree after boolean" << std::endl;
    return false;
  }

  // Now decode each tree (remaining elements)
  for(size_t ti = 1; ti < topArrayLen; ++ti){
    if(!cbor_value_is_array(&arrIt)){
      std::cerr << "Tree element is not an array" << std::endl;
      return false;
    }
    size_t treeLen = 0;
    if(cbor_value_get_array_length(&arrIt, &treeLen) != CborNoError){
      std::cerr << "Failed to get tree array length" << std::endl;
      return false;
    }
    CborValue treeIt;
    if(cbor_value_enter_container(&arrIt, &treeIt) != CborNoError){
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
    // leave tree array to advance arrIt
    if(cbor_value_leave_container(&arrIt, &treeIt) != CborNoError){
      std::cerr << "Failed to leave tree array" << std::endl;
      return false;
    }
    trees.push_back(t);
    // the first tree in the serialized document is always the main tree to execute,
    // but only if was_main_tree_empty_cbor is false (i.e. there was a main tree set)
    if(ti == 1 && !was_main_tree_empty_cbor){
      main_tree_to_execute = treeName;
    }
  }

  // leaving trees array (not strictly necessary as parser will be discarded)
  // cbor_value_leave_container(&it, &treesIt);

  return true;
}
