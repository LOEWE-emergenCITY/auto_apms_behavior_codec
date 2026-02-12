#pragma once

#include <vector>
#include <string>
#include <memory>

namespace behavior_tree_representation {

struct Port {
  virtual ~Port() = default;
  virtual uint16_t getID() const = 0;
};

struct PortInt : public Port {
  explicit PortInt(int32_t v, int16_t id) : value(v), id(id) {}
  uint16_t getID() const override { return this->id; }
  int32_t value;
  private:
  int16_t id; //id of the port, this is its index in the vector of ports for the node, this is may differ between ports with same name/type, if they belong to different nodes
};

struct PortFloat : public Port {
  explicit PortFloat(float v, int16_t id) : value(v), id(id) {}
  uint16_t getID() const override { return this->id; }
  float value;
  private:
  int16_t id; //id of the port, this is its index in the vector of ports for the node, this is may differ between ports with same name/type, if they belong to different nodes
};

struct PortString : public Port {
  explicit PortString(const std::string& v, int16_t id) : value(v), id(id) {}
  uint16_t getID() const override { return this->id; }
  std::string value;
  private:
  int16_t id; //id of the port, this is its index in the vector of ports for the node, this is may differ between ports with same name/type, if they belong to different nodes
};

struct PortBool : public Port {
  explicit PortBool(bool v, int16_t id) : value(v), id(id) {}
  uint16_t getID() const override { return this->id; }
  bool value;
  private:
  int16_t id; //id of the port, this is its index in the vector of ports for the node, this is may differ between ports with same name/type, if they belong to different nodes
};

struct PortAnyTypeAllowed : public Port {
  explicit PortAnyTypeAllowed(const BT::AnyTypeAllowed& v, int16_t id) : value(v), id(id) {}
  uint16_t getID() const override { return this->id; }
  BT::AnyTypeAllowed value;
  private:
  int16_t id; //id of the port, this is its index in the vector of ports for the node, this is may differ between ports with same name/type, if they belong to different nodes
};



struct Node {
  std::string registration_name;
  std::string instance_name;
  std::vector<std::shared_ptr<Port>> ports; //ports of the node, same order as in xml
  std::vector<Node> children; //children of the node, same order as in xml
  void print() const {
    std::cout << "  Node: " << instance_name << " (Registration Name: " << registration_name << ")" << std::endl;
    std::cout << "  Ports:" << std::endl;
    for (const auto& port : ports) {
      std::cout << "    Port ID: " << port->getID() << std::endl;
    }
    if(!children.empty()){
      std::cout << "  Children:" << std::endl;
      for (const auto& child : children) {
        child.print();
      }
    }
  }
};

struct Tree {
  Node root; // root node of the tree
  std::string name;
  std::vector<uint8_t> serialize() const {
    // Minimal placeholder serialization: empty for now
    return {};
  }
  void print() const {
    std::cout << "  Tree: " << name << std::endl;
    root.print();
  }
};

struct Document {
  std::vector<Tree> trees; //trees contained in the document, same order as in xml
  std::string main_tree_to_execute; //is this required?
  void print() const {
    std::cout << "Document with " << trees.size() << " trees. Main tree to execute: " << main_tree_to_execute << std::endl;
    for (const auto& tree : trees) {
      tree.print();
    }
  }
};

} // namespace behavior_tree_representation