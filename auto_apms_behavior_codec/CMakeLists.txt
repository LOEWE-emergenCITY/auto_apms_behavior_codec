cmake_minimum_required(VERSION 3.8)
project(auto_apms_behavior_codec)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auto_apms_behavior_tree_core REQUIRED)

# Add dictionary manager source files
add_library(dictionary_manager
  src/dictionary_manager.cpp
)

# Add include directory
target_include_directories(dictionary_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dictionary manager with auto_apms_behavior_tree_core
target_link_libraries(dictionary_manager PUBLIC 
  auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
)

# Add behavior tree encoder executable
add_executable(behavior_tree_encoder_node src/behavior_tree_encoder.cpp)

# Add include directory to encoder
target_include_directories(behavior_tree_encoder_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link encoder with dependencies
target_link_libraries(behavior_tree_encoder_node PRIVATE
  dictionary_manager
  rclcpp::rclcpp
  auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
)

# Install the executable
install(
  TARGETS behavior_tree_encoder_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

ament_package()
