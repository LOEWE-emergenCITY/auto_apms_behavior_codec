#include "auto_apms_behavior_codec/behavior_tree_representation.hpp"
#include "cbor.h"

using namespace behavior_tree_representation;

std::vector<uint8_t> Document::serialize() {
  CborEncoder* encoder = new CborEncoder();
  uint8_t* buf = new uint8_t[1024];  // allocate a large enough buffer
  cbor_encoder_init(encoder, buf, sizeof(buf), 0);

  CborEncoder* arrayEncoder = new CborEncoder();
  uint8_t* arrayBuf = new uint8_t[1024];  // allocate a large enough buffer for the array
  cbor_encoder_init(arrayEncoder, arrayBuf, sizeof(arrayBuf), 0);

  CborEncoder* arrayEncoder2 = new CborEncoder();
  uint8_t* arrayBuf2 = new uint8_t[1024];  // allocate a large enough buffer for the second array
  cbor_encoder_init(arrayEncoder2, arrayBuf2, sizeof(arrayBuf2), 0);

  //create array containing the documents basic information: main tree to execute and a array of trees
  cbor_encoder_create_array(encoder, arrayEncoder, 2);
  cbor_encode_text_stringz(arrayEncoder, main_tree_to_execute.c_str());
  cbor_encoder_create_array(arrayEncoder, arrayEncoder2, trees.size());
  for(Tree tree : trees){

    //the trees serialize function encodes the tree onto a given encoder
    tree.serialize(arrayEncoder2);
  }
  // close the trees array
  cbor_encoder_close_container_checked(arrayEncoder, arrayEncoder2);

  // close the main array
  cbor_encoder_close_container_checked(encoder, arrayEncoder);

  size_t cborSize = cbor_encoder_get_buffer_size(encoder, buf);

  std::vector<uint8_t> result(buf, buf + cborSize);
  delete[] buf;
  delete encoder;
  delete[] arrayBuf;
  delete arrayEncoder;
  delete[] arrayBuf2;
  delete arrayEncoder2;

  return result;
}