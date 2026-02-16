#include "auto_apms_behavior_codec/behavior_tree_representation.hpp"
#include "cbor.h"

using namespace behavior_tree_representation;

std::vector<uint8_t> Document::serialize(std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dictionary_manager) const {
  CborEncoder* encoder = new CborEncoder();
  uint8_t buf[1024];  // allocate a large enough buffer
  cbor_encoder_init(encoder, buf, sizeof(buf), 0);

  CborEncoder* arrayEncoder = new CborEncoder();
  uint8_t arrayBuf[1024];  // allocate a large enough buffer for the array
  cbor_encoder_init(arrayEncoder, arrayBuf, sizeof(arrayBuf), 0);

  //create array of trees, the main tree to execute comes first
  cbor_encoder_create_array(encoder, arrayEncoder, trees.size());
  for(Tree tree : trees){

    //the trees serialize function encodes the tree onto a given encoder
    tree.serialize(arrayEncoder, dictionary_manager);
  }
  // close the trees array
  cbor_encoder_close_container_checked(encoder, arrayEncoder);

  size_t cborSize = cbor_encoder_get_buffer_size(encoder, buf);
  std::cout << "CBOR encoded data size: " << cborSize << " bytes" << std::endl;
  std::vector<uint8_t> result(buf, buf + cborSize);
  delete encoder;
  delete arrayEncoder;

  return result;
}

bool Node::serialize(CborEncoder* encoder, std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dictionary_manager) const {
  CborEncoder arrayEncoder;
  uint8_t arrayBuf[1024]; 
  cbor_encoder_init(&arrayEncoder, arrayBuf, sizeof(arrayBuf), 0);

  //create array containing the nodes basic information: type code, an array of ports and an array of children
  cbor_encoder_create_array(encoder, &arrayEncoder, 3);
  cbor_encode_uint(&arrayEncoder, dictionary_manager->get_dictionary_info_by_name(type_name).id);

  CborEncoder portsArrayEncoder;
  uint8_t portsArrayBuf[1024];  // allocate a large enough buffer for the ports array
  cbor_encoder_init(&portsArrayEncoder, portsArrayBuf, sizeof(portsArrayBuf), 0);

  cbor_encoder_create_array(&arrayEncoder, &portsArrayEncoder, ports.size());
  for(std::shared_ptr<Port> port : ports){
    std::cout << "Serializing port: " << port->getID() << std::endl;
    port->serialize(&portsArrayEncoder);
  }
  // close the ports array
  cbor_encoder_close_container_checked(&arrayEncoder, &portsArrayEncoder);

  CborEncoder childrenArrayEncoder;
  uint8_t childrenArrayBuf[1024];  // allocate a large enough buffer for the children array
  cbor_encoder_init(&childrenArrayEncoder, childrenArrayBuf, sizeof(childrenArrayBuf), 0);
  cbor_encoder_create_array(&arrayEncoder, &childrenArrayEncoder, children.size());
  for(std::shared_ptr<Node> child : children){
    child->serialize(&childrenArrayEncoder, dictionary_manager);
  }
  // close the children array
  cbor_encoder_close_container_checked(&arrayEncoder, &childrenArrayEncoder);
  // close the main array
  cbor_encoder_close_container_checked(encoder, &arrayEncoder);


  return true;
}


bool Tree::serialize(CborEncoder* encoder, std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dictionary_manager) const {
  CborEncoder* arrayEncoder = new CborEncoder();
  uint8_t arrayBuf[1024]; 
  cbor_encoder_init(arrayEncoder, arrayBuf, sizeof(arrayBuf), 0);

  //create array containing the trees basic information: name and the root node
  cbor_encoder_create_array(encoder, arrayEncoder, 2);
  cbor_encode_text_string(arrayEncoder, name.c_str(),name.size());
  root.serialize(arrayEncoder, dictionary_manager);
  // close the main array
  cbor_encoder_close_container_checked(encoder, arrayEncoder);

  delete arrayEncoder;

  return true;
}

bool PortInt::serialize(CborEncoder* encoder) const {
  CborEncoder* portArrayEncoder = new CborEncoder();
  uint8_t portArrayBuf[1024];  // allocate a large enough buffer for the port array
  cbor_encoder_init(portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, portArrayEncoder, 2);
  cbor_encode_uint(encoder, this->getID());
  cbor_encode_int(encoder, this->value);
  cbor_encoder_close_container_checked(encoder, portArrayEncoder);
  delete portArrayEncoder;
  
  return true;
}

bool PortFloat::serialize(CborEncoder* encoder) const {
  CborEncoder* portArrayEncoder = new CborEncoder();
  uint8_t* portArrayBuf = new uint8_t[1024];  // allocate a large enough buffer for the port array
  cbor_encoder_init(portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, portArrayEncoder, 2);
  cbor_encode_uint(encoder, this->getID());
  cbor_encode_float(encoder, this->value);
  cbor_encoder_close_container_checked(encoder, portArrayEncoder);
  delete[] portArrayBuf;
  delete portArrayEncoder;
  
  return true;
}

bool PortBool::serialize(CborEncoder* encoder) const {
  CborEncoder* portArrayEncoder = new CborEncoder();
  uint8_t* portArrayBuf = new uint8_t[1024];  // allocate a large enough buffer for the port array
  cbor_encoder_init(portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, portArrayEncoder, 2);
  cbor_encode_uint(encoder, this->getID());
  cbor_encode_boolean(encoder, this->value);
  cbor_encoder_close_container_checked(encoder, portArrayEncoder);
  delete[] portArrayBuf;
  delete portArrayEncoder;

  return true;
}

bool PortString::serialize(CborEncoder* encoder) const {
  std::cout <<" serializing string port" << std::endl;
  CborEncoder* portArrayEncoder = new CborEncoder();
  uint8_t* portArrayBuf = new uint8_t[1024];  // allocate a large enough buffer for the port array
  cbor_encoder_init(portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, portArrayEncoder, 2);
  cbor_encode_uint(portArrayEncoder, this->getID());
  cbor_encode_text_string(portArrayEncoder, this->value.c_str(), this->value.size());
  cbor_encoder_close_container_checked(encoder, portArrayEncoder);
  delete[] portArrayBuf;
  delete portArrayEncoder;
  return true;
}

bool PortAnyTypeAllowed::serialize(CborEncoder* encoder) const {
  CborEncoder* portArrayEncoder = new CborEncoder();
  uint8_t* portArrayBuf = new uint8_t[1024];  // allocate a large enough buffer for the port array
  cbor_encoder_init(portArrayEncoder, portArrayBuf, sizeof(portArrayBuf), 0);
  cbor_encoder_create_array(encoder, portArrayEncoder, 2);
  cbor_encode_uint(portArrayEncoder, this->getID());
  cbor_encode_text_string(portArrayEncoder, "Placeholder",12);
  cbor_encoder_close_container_checked(encoder, portArrayEncoder);
  delete[] portArrayBuf;
  delete portArrayEncoder;
  //for the value of an AnyTypeAllowed port, we need to encode both the type information and the actual value, the type is included as string, the data as a binary blob
  //cbor_encode_text_string(encoder, this->value.first.c_str(), this->value.first.size());
  //cbor_encode_byte_string(encoder, this->value.second.data(), this->value.second.size());
  return true;
}