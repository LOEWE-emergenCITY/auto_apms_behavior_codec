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
  explicit PortInt(int32_t v) : value(v) {}
  uint16_t getID() const override { return 0; }
  int32_t value;
};

struct Node {
  std::string registration_name;
  std::string instance_name;
  std::vector<std::shared_ptr<Port>> ports;
  std::vector<Node> children;
};

struct Tree {
  std::vector<Node> nodes;

  std::vector<uint8_t> serialize() const {
    // Minimal placeholder serialization: empty for now
    return {};
  }
};

} // namespace behavior_tree_representation