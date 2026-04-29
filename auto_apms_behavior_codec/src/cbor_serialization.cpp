#include "auto_apms_behavior_codec/behavior_tree_representation.hpp"
#include "cbor.h"

using namespace behavior_tree_representation;

std::vector<uint8_t> Document::serialize(
  std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dictionary_manager) const
{
  CborEncoder * encoder = new CborEncoder();
  uint8_t buf[2048];  // allocate a large enough buffer
  cbor_encoder_init(encoder, buf, sizeof(buf), 0);

  // Top-level array: [was_main_tree_empty, tree1, tree2, ...]
  size_t topArraySize = 1 + trees.size();
  CborEncoder topArrayEncoder;
  cbor_encoder_create_array(encoder, &topArrayEncoder, topArraySize);
  bool was_main_tree_empty = main_tree_to_execute.empty();
  cbor_encode_boolean(&topArrayEncoder, was_main_tree_empty);
  std::cout << "Serializing Document: main_tree_to_execute is "
            << (was_main_tree_empty ? "empty" : main_tree_to_execute) << std::endl;
  for (const Tree & tree : trees) {
    tree.serialize(&topArrayEncoder, dictionary_manager);
  }
  // close the top-level array
  cbor_encoder_close_container_checked(encoder, &topArrayEncoder);

  size_t cborSize = cbor_encoder_get_buffer_size(encoder, buf);
  std::cout << "CBOR encoded data size: " << cborSize << " bytes" << std::endl;
  std::vector<uint8_t> result(buf, buf + cborSize);
  delete encoder;

  return result;
}

bool Node::serialize(
  CborEncoder * encoder, std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dictionary_manager) const
{
  CborEncoder arrayEncoder;
  uint8_t arrayBuf[2048];
  cbor_encoder_init(&arrayEncoder, arrayBuf, sizeof(arrayBuf), 0);

  // Calculate array size: type code (1) + ports array (1) + children array (1, always present) + additional_parameters array (1 if present)
  uint arraySize = 3;  // type code + ports array + children array (always present)
  if (additional_parameters.size() != 0) arraySize++;
  
  // create array containing the nodes basic information: type code, an array of ports, an array of children, and additional parameters (if present)
  cbor_encoder_create_array(encoder, &arrayEncoder, arraySize);
  cbor_encode_uint(&arrayEncoder, dictionary_manager->get_dictionary_info_by_name(type_name).id);

  CborEncoder portsArrayEncoder;
  uint8_t portsArrayBuf[2048];  // allocate a large enough buffer for the ports array
  cbor_encoder_init(&portsArrayEncoder, portsArrayBuf, sizeof(portsArrayBuf), 0);

  // create array for ports
  cbor_encoder_create_array(&arrayEncoder, &portsArrayEncoder, ports.size());
  for (std::shared_ptr<Port> port : ports) {
    std::cout << "Serializing port: " << port->getID() << std::endl;
    port->serialize(&portsArrayEncoder);
  }
  // close the ports array
  cbor_encoder_close_container_checked(&arrayEncoder, &portsArrayEncoder);

  // Always encode children array, even if empty
  CborEncoder childrenArrayEncoder;
  uint8_t childrenArrayBuf[2048];  // allocate a large enough buffer for the children array
  cbor_encoder_init(&childrenArrayEncoder, childrenArrayBuf, sizeof(childrenArrayBuf), 0);
  cbor_encoder_create_array(&arrayEncoder, &childrenArrayEncoder, children.size());
  for (std::shared_ptr<Node> child : children) {
    child->serialize(&childrenArrayEncoder, dictionary_manager);
  }
  // close the children array
  cbor_encoder_close_container_checked(&arrayEncoder, &childrenArrayEncoder);

  // Encode additional parameters as 4th element if present
  if (additional_parameters.size() != 0) {
    CborEncoder paramsArrayEncoder;
    uint8_t paramsArrayBuf[2048];  // allocate a large enough buffer for the additional parameters array
    cbor_encoder_init(&paramsArrayEncoder, paramsArrayBuf, sizeof(paramsArrayBuf), 0);
    
    // create array for additional parameters (array of [key, value] pairs)
    cbor_encoder_create_array(&arrayEncoder, &paramsArrayEncoder, additional_parameters.size());
    for (const auto & param : additional_parameters) {
      CborEncoder paramPairEncoder;
      uint8_t paramPairBuf[2048];
      cbor_encoder_init(&paramPairEncoder, paramPairBuf, sizeof(paramPairBuf), 0);
      
      // create [key, value] pair array
      cbor_encoder_create_array(&paramsArrayEncoder, &paramPairEncoder, 2);
      cbor_encode_text_string(&paramPairEncoder, param.first.c_str(), param.first.size());
      cbor_encode_text_string(&paramPairEncoder, param.second.c_str(), param.second.size());
      cbor_encoder_close_container_checked(&paramsArrayEncoder, &paramPairEncoder);
    }
    // close the additional parameters array
    cbor_encoder_close_container_checked(&arrayEncoder, &paramsArrayEncoder);
  }

  // close the main array
  cbor_encoder_close_container_checked(encoder, &arrayEncoder);

  return true;
}

bool Tree::serialize(
  CborEncoder * encoder, std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dictionary_manager) const
{
  CborEncoder * arrayEncoder = new CborEncoder();
  uint8_t arrayBuf[2048];
  cbor_encoder_init(arrayEncoder, arrayBuf, sizeof(arrayBuf), 0);

  // create array containing the trees basic information: name and the root node
  cbor_encoder_create_array(encoder, arrayEncoder, 2);
  cbor_encode_text_string(arrayEncoder, name.c_str(), name.size());
  root.serialize(arrayEncoder, dictionary_manager);
  // close the main array
  cbor_encoder_close_container_checked(encoder, arrayEncoder);

  delete arrayEncoder;

  return true;
}

bool PortInt::serialize(CborEncoder * encoder) const
{
  CborEncoder * portArrayEncoder = new CborEncoder();
  uint8_t portArrayBuf[2048];  // allocate a large enough buffer for the port array
  cbor_encoder_init(portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, portArrayEncoder, 2);
  cbor_encode_uint(portArrayEncoder, this->getID());
  cbor_encode_int(portArrayEncoder, this->value);
  cbor_encoder_close_container_checked(encoder, portArrayEncoder);
  delete portArrayEncoder;

  return true;
}

bool PortUInt::serialize(CborEncoder * encoder) const
{
  CborEncoder * portArrayEncoder = new CborEncoder();
  uint8_t portArrayBuf[2048];
  cbor_encoder_init(portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, portArrayEncoder, 2);
  cbor_encode_uint(portArrayEncoder, this->getID());
  cbor_encode_uint(portArrayEncoder, this->value);
  cbor_encoder_close_container_checked(encoder, portArrayEncoder);
  delete portArrayEncoder;
  return true;
}

bool PortFloat::serialize(CborEncoder * encoder) const
{
  CborEncoder * portArrayEncoder = new CborEncoder();
  uint8_t * portArrayBuf = new uint8_t[2048];  // allocate a large enough buffer for the port array
  cbor_encoder_init(portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, portArrayEncoder, 2);
  cbor_encode_uint(portArrayEncoder, this->getID());
  cbor_encode_float(portArrayEncoder, this->value);
  cbor_encoder_close_container_checked(encoder, portArrayEncoder);
  delete[] portArrayBuf;
  delete portArrayEncoder;

  return true;
}

bool PortDouble::serialize(CborEncoder * encoder) const
{
  CborEncoder * portArrayEncoder = new CborEncoder();
  uint8_t * portArrayBuf = new uint8_t[2048];  // allocate a large enough buffer for the port array
  cbor_encoder_init(portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, portArrayEncoder, 2);
  cbor_encode_uint(portArrayEncoder, this->getID());
  cbor_encode_double(portArrayEncoder, this->value);
  cbor_encoder_close_container_checked(encoder, portArrayEncoder);
  delete[] portArrayBuf;
  delete portArrayEncoder;
  return true;
}

bool PortBool::serialize(CborEncoder * encoder) const
{
  CborEncoder * portArrayEncoder = new CborEncoder();
  uint8_t * portArrayBuf = new uint8_t[2048];  // allocate a large enough buffer for the port array
  cbor_encoder_init(portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, portArrayEncoder, 2);
  cbor_encode_uint(portArrayEncoder, this->getID());
  cbor_encode_boolean(portArrayEncoder, this->value);
  cbor_encoder_close_container_checked(encoder, portArrayEncoder);
  delete[] portArrayBuf;
  delete portArrayEncoder;

  return true;
}

bool PortString::serialize(CborEncoder * encoder) const
{
  std::cout << " serializing string port" << std::endl;
  CborEncoder * portArrayEncoder = new CborEncoder();
  uint8_t * portArrayBuf = new uint8_t[2048];  // allocate a large enough buffer for the port array
  cbor_encoder_init(portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, portArrayEncoder, 2);
  cbor_encode_uint(portArrayEncoder, this->getID());
  cbor_encode_text_string(portArrayEncoder, this->value.c_str(), this->value.size());
  cbor_encoder_close_container_checked(encoder, portArrayEncoder);
  delete[] portArrayBuf;
  delete portArrayEncoder;
  return true;
}

bool PortAnyTypeAllowed::serialize(CborEncoder * encoder) const
{
  CborEncoder * portArrayEncoder = new CborEncoder();
  uint8_t * portArrayBuf = new uint8_t[2048];  // allocate a large enough buffer for the port array
  cbor_encoder_init(portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, portArrayEncoder, 2);
  cbor_encode_uint(portArrayEncoder, this->getID());
  cbor_encode_text_string(portArrayEncoder, this->value.c_str(), this->value.size());
  cbor_encoder_close_container_checked(encoder, portArrayEncoder);
  delete[] portArrayBuf;
  delete portArrayEncoder;
  // for the value of an AnyTypeAllowed port, we need to encode both the type information and the actual value, the type
  // is included as string, the data as a binary blob cbor_encode_text_string(encoder, this->value.first.c_str(),
  // this->value.first.size()); cbor_encode_byte_string(encoder, this->value.second.data(), this->value.second.size());
  return true;
}

bool PortAny::serialize(CborEncoder * encoder) const
{
  CborEncoder * portArrayEncoder = new CborEncoder();
  uint8_t * portArrayBuf = new uint8_t[2048];  // allocate a large enough buffer for the port array
  cbor_encoder_init(portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, portArrayEncoder, 2);
  cbor_encode_uint(portArrayEncoder, this->getID());
  cbor_encode_text_string(portArrayEncoder, this->value.c_str(), this->value.size());
  cbor_encoder_close_container_checked(encoder, portArrayEncoder);
  delete[] portArrayBuf;
  delete portArrayEncoder;
  // for the value of an BT::Any port, we need to encode both the type information and the actual value, the type is
  // included as string, the data as a binary blob cbor_encode_text_string(encoder, this->value.first.c_str(),
  // this->value.first.size()); cbor_encode_byte_string(encoder, this->value.second.data(), this->value.second.size());
  return true;
}

bool PortNodeStatus::serialize(CborEncoder * encoder) const
{
  CborEncoder * portArrayEncoder = new CborEncoder();
  uint8_t * portArrayBuf = new uint8_t[2048];  // allocate a large enough buffer for the port array
  cbor_encoder_init(portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, portArrayEncoder, 2);
  cbor_encode_uint(portArrayEncoder, this->getID());
  cbor_encode_uint(portArrayEncoder, (uint64_t)this->value);
  cbor_encoder_close_container_checked(encoder, portArrayEncoder);
  delete[] portArrayBuf;
  delete portArrayEncoder;
  return true;
}

bool PortSubTreeSpecial::serialize(CborEncoder * encoder) const
{
  CborEncoder * portArrayEncoder = new CborEncoder();
  uint8_t * portArrayBuf = new uint8_t[2048];  // allocate a large enough buffer for the port array
  cbor_encoder_init(portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, portArrayEncoder, 3);
  cbor_encode_uint(portArrayEncoder, this->getID());
  cbor_encode_text_string(portArrayEncoder, this->name.c_str(), this->name.size());
  cbor_encode_text_string(portArrayEncoder, this->value.c_str(), this->value.size());
  cbor_encoder_close_container_checked(encoder, portArrayEncoder);
  delete[] portArrayBuf;
  delete portArrayEncoder;
  return true;
}

bool PortInvalid::serialize(CborEncoder * encoder) const
{
  // Encode as an array: [error_string, port_index], this is deliberatly not starting with the int, to simplify
  // detection during deserialization and to avoid confusion with valid ports, which always start with the port index as
  // uint
  CborEncoder portArrayEncoder;
  uint8_t portArrayBuf[128];
  cbor_encoder_init(&portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, &portArrayEncoder, 2);
  cbor_encode_text_string(&portArrayEncoder, this->value.c_str(), this->value.size());
  cbor_encode_int(&portArrayEncoder, this->getID());
  cbor_encoder_close_container_checked(encoder, &portArrayEncoder);
  return true;
}