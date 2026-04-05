#include "auto_apms_behavior_codec/tree_encoder_executor_proxy.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auto_apms_behavior_codec::TreeEncoderExecutorProxy>());
  rclcpp::shutdown();
  return 0;
}
