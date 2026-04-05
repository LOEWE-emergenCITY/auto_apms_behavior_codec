#include "auto_apms_behavior_codec/behavior_tree_representation.hpp"

using namespace behavior_tree_representation;

void Node::print() const
{
  std::cout << "  Node: " << instance_name << " (Registration Name: " << type_name << ")" << std::endl;
  std::cout << "  Ports:" << std::endl;
  for (const auto & port : ports) {
    std::cout << "    Port ID: " << port->getID() << std::endl;
    std::cout << "    Port Value: ";
    if (auto port_int = dynamic_cast<PortInt *>(port.get())) {
      std::cout << port_int->value << " (int)" << std::endl;
    } else if (auto port_float = dynamic_cast<PortFloat *>(port.get())) {
      std::cout << port_float->value << " (float)" << std::endl;
    } else if (auto port_bool = dynamic_cast<PortBool *>(port.get())) {
      std::cout << (port_bool->value ? "true" : "false") << " (bool)" << std::endl;
    } else if (auto port_double = dynamic_cast<PortDouble *>(port.get())) {
      std::cout << port_double->value << " (double)" << std::endl;
    } else if (auto port_string = dynamic_cast<PortString *>(port.get())) {
      std::cout << port_string->value << " (string)" << std::endl;
    } else if (auto port_any = dynamic_cast<PortAnyTypeAllowed *>(port.get())) {
      std::cout << "AnyTypeAllowed: " << port_any->value << std::endl;
    } else if (auto port_subtree = dynamic_cast<PortSubTreeSpecial *>(port.get())) {
      std::cout << "SubTreeSpecial: " << port_subtree->name << " = " << port_subtree->value << std::endl;
    } else if (auto port_invalid = dynamic_cast<PortInvalid *>(port.get())) {
      std::cout << "Invalid: " << port_invalid->value << std::endl;
    } else if (auto port_node_status = dynamic_cast<PortNodeStatus *>(port.get())) {
      std::cout << "NodeStatus: " << port_node_status->string_value << " = " << port_node_status->value << std::endl;
    } else if (auto port_any_bt_any = dynamic_cast<PortAny *>(port.get())) {
      std::cout << "BT::Any: " << port_any_bt_any->value << std::endl;
    } else {
      std::cout << "Unknown port type" << std::endl;
    }
  }
  if (!children.empty()) {
    std::cout << "  Children:" << std::endl;
    for (const auto & child : children) {
      child->print();
    }
  }
}

void Tree::print() const
{
  std::cout << "  Tree: " << name << std::endl;
  root.print();
}

void Document::print() const
{
  std::cout << "Document with " << trees.size() << " trees. Main tree to execute: " << main_tree_to_execute
            << std::endl;
  for (const auto & tree : trees) {
    tree.print();
  }
}
