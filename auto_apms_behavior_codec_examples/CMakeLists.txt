cmake_minimum_required(VERSION 3.8)
project(auto_apms_behavior_codec_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(auto_apms_behavior_tree REQUIRED)

auto_apms_behavior_tree_register_trees(
  "behavior/hello_world.xml"
  NODE_MANIFEST auto_apms_behavior_tree::behavior_tree_nodes
)

ament_package()
