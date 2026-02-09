#include "behavior_tree_encoder.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace auto_apms_behavior_codec;

BehaviorTreeEncoder::BehaviorTreeEncoder()
    : rclcpp::Node("behavior_tree_encoder")
{
  // Initialize the dictionary manager
  this->dictionary_manager_ = std::make_unique<DictionaryManager>();

  this->dictionary_manager_->print_dictionary();
}

std::vector<uint8_t> BehaviorTreeEncoder::encode(const std::string& behavior_tree_yaml)
{
  // TODO: Implement behavior tree encoding logic
  return std::vector<uint8_t>();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BehaviorTreeEncoder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
