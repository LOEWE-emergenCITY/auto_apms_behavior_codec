cmake_minimum_required(VERSION 3.8)
project(auto_apms_behavior_codec_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(auto_apms_behavior_tree REQUIRED)
find_package(auto_apms_behavior_codec REQUIRED)
find_package(auto_apms_behavior_codec_interfaces REQUIRED)

auto_apms_behavior_tree_register_nodes(goto
  NODE_MANIFEST 
  auto_apms_behavior_tree::behavior_tree_nodes
  behavior/node_manifest/goto_manifest.yaml
)

auto_apms_behavior_tree_register_trees(
  "behavior/goto_examples.xml"
  "behavior/hello_world.xml"
  NODE_MANIFEST ${PROJECT_NAME}::goto
)

add_executable(dummy_telemetry_generator
  src/dummy_telemetry_generator.cpp
)
target_link_libraries(dummy_telemetry_generator PRIVATE
  rclcpp::rclcpp
  auto_apms_behavior_codec::auto_apms_behavior_codec
  ${auto_apms_behavior_codec_interfaces_TARGETS}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS dummy_telemetry_generator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
