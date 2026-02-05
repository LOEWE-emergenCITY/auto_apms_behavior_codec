#pragma once

#include <vector>
#include <string>
#include "dictionary_manager.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_codec
{
  // handles encoding of behavior trees into a binary format, currently only minimal skeleton in order to be able to run a Ros node and call the dictionary manager
  class BehaviorTreeEncoder : public rclcpp::Node
  {
  public:
      BehaviorTreeEncoder();
      ~BehaviorTreeEncoder() = default;
      std::vector<uint8_t> encode(const std::string& behavior_tree_yaml);
  private:
      DictionaryManager dictionary_manager_;
  };

} 