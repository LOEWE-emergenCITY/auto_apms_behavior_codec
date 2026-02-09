#pragma once

#include <vector>
#include <string>
#include "dictionary_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

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
      std::unique_ptr<DictionaryManager> dictionary_manager_;
  };

} 