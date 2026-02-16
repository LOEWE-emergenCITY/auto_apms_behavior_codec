#pragma once

#include <vector>
#include <string>
#include <memory>
#include <iostream>

#include "cbor.h"

#include "auto_apms_behavior_codec/dictionary_manager.hpp"

namespace behavior_tree_representation {

struct Port {
  virtual ~Port() = default;

  //get id of the port, this is its index in the vector of ports for the node, this is may differ between ports with same name/type, if they belong to different nodes
  virtual uint16_t getID() const = 0;
  //serializes the Port using a given CBOR encoder object, implementation depending on the type of the port
  virtual bool serialize(CborEncoder* encoder) const = 0;
};

struct PortInt : public Port {
  explicit PortInt(int32_t v, int16_t id) : value(v), id(id) {}
  uint16_t getID() const override { return this->id; }
  int32_t value;

  //serializes the Port using a given CBOR encoder object
  bool serialize(CborEncoder* encoder) const override;
  private:
  int16_t id; //id of the port, this is its index in the vector of ports for the node, this is may differ between ports with same name/type, if they belong to different nodes
};

struct PortFloat : public Port {
  explicit PortFloat(float v, int16_t id) : value(v), id(id) {}
  uint16_t getID() const override { return this->id; }
  float value;

  //serializes the Port using a given CBOR encoder object
  bool serialize(CborEncoder* encoder) const override;
  private:
  int16_t id; //id of the port, this is its index in the vector of ports for the node, this is may differ between ports with same name/type, if they belong to different nodes
};

struct PortString : public Port {
  explicit PortString(const std::string& v, int16_t id) : value(v), id(id) {}
  uint16_t getID() const override { return this->id; }
  std::string value;

  //serializes the Port using a given CBOR encoder object
  bool serialize(CborEncoder* encoder) const override;
  private:
  int16_t id; //id of the port, this is its index in the vector of ports for the node, this is may differ between ports with same name/type, if they belong to different nodes
};

struct PortBool : public Port {
  explicit PortBool(bool v, int16_t id) : value(v), id(id) {}
  uint16_t getID() const override { return this->id; }
  bool value;

  //serializes the Port using a given CBOR encoder object
  bool serialize(CborEncoder* encoder) const override;
  private:
  int16_t id; //id of the port, this is its index in the vector of ports for the node, this is may differ between ports with same name/type, if they belong to different nodes
};

struct PortAnyTypeAllowed : public Port {
  explicit PortAnyTypeAllowed(BT::AnyTypeAllowed v, int16_t id) : value(v), id(id) {}
  uint16_t getID() const override { return this->id; }
  BT::AnyTypeAllowed value;  // Binary blob data

  //serializes the Port using a given CBOR encoder object 
  bool serialize(CborEncoder* encoder) const override;
  private:
  int16_t id; //id of the port, this is its index in the vector of ports for the node, this is may differ between ports with same name/type, if they belong to different nodes
};



struct Node {
  std::string type_name;
  std::string instance_name;
  std::vector<std::shared_ptr<Port>> ports; //ports of the node, same order as in xml
  std::vector<std::shared_ptr<Node>> children; //children of the node, same order as in xml

  void print() const;
   //serializes the Node using a given CBOR encoder object using the given dictionary as reference for node and port type codes
  bool serialize(CborEncoder* encoder, std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dictionary_manager) const;
};

struct Tree {
  Node root; // root node of the tree
  std::string name;
  void print() const;

  //serializes the Tree using a given CBOR encoder object using the given dictionary as reference for node and port type codes
  bool serialize(CborEncoder* encoder, std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dictionary_manager) const;

};

struct Document {
  std::vector<Tree> trees; //trees contained in the document, same order as in xml
  std::string main_tree_to_execute; //is this required?
  void print() const;
  //serializes the document using CBOR using the given dictionary as reference for node and port type codes
  std::vector<uint8_t> serialize(std::shared_ptr<auto_apms_behavior_codec::DictionaryManager> dictionary_manager) const;
};

} // namespace behavior_tree_representation