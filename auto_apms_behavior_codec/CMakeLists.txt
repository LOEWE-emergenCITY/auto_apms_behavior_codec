cmake_minimum_required(VERSION 3.11)
project(auto_apms_behavior_codec)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(auto_apms_behavior_tree_core REQUIRED)
find_package(auto_apms_behavior_codec_interfaces REQUIRED)
find_package(auto_apms_interfaces REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Generate parameter headers employing https://github.com/PickNikRobotics/generate_parameter_library
generate_parameter_library(encoder_params "config/encoder_params.yaml")
generate_parameter_library(decoder_base_params "config/decoder_base_params.yaml")
generate_parameter_library(decoder_publisher_params "config/decoder_publisher_params.yaml")
generate_parameter_library(decoder_executor_client_params "config/decoder_executor_client_params.yaml")

# Fetch tinycbor
include(FetchContent)
FetchContent_Declare(
  tinycbor
  GIT_REPOSITORY https://github.com/intel/tinycbor.git
  GIT_TAG v7.0
  PATCH_COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/patch_tinycbor.cmake
)
set(BUILD_TOOLS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(tinycbor)

# Add behavior tree representation source files
add_library(behavior_tree_representation
  src/behavior_tree_representation.cpp
  src/cbor_serialization.cpp
  src/cbor_deserialization.cpp
  src/telemetry_message_builder.cpp
)
target_include_directories(behavior_tree_representation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(behavior_tree_representation PUBLIC
  auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
  tinycbor
  ${auto_apms_behavior_codec_interfaces_TARGETS}
)

# Add dictionary manager source files
add_library(dictionary_manager STATIC
  src/dictionary_manager.cpp
)
target_include_directories(dictionary_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(dictionary_manager PUBLIC
  auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
  tinycbor
  behavior_tree_representation
)

# Encoder executable
add_executable(behavior_tree_encoder_node src/behavior_tree_encoder.cpp)
target_include_directories(behavior_tree_encoder_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(behavior_tree_encoder_node PRIVATE
  dictionary_manager
  rclcpp::rclcpp
  auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
  behavior_tree_representation
  ${auto_apms_behavior_codec_interfaces_TARGETS}
  encoder_params
)

# Decoder base static library
add_library(behavior_tree_decoder_base STATIC
  src/behavior_tree_decoder_base.cpp
)
target_include_directories(behavior_tree_decoder_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(behavior_tree_decoder_base PUBLIC
  dictionary_manager
  rclcpp::rclcpp
  auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
  behavior_tree_representation
  ${auto_apms_behavior_codec_interfaces_TARGETS}
  decoder_base_params
)

# Decoder publisher executable
add_executable(tree_decoder_publisher_node src/tree_decoder_publisher.cpp)
target_include_directories(tree_decoder_publisher_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(tree_decoder_publisher_node PRIVATE
  behavior_tree_decoder_base
  decoder_publisher_params
)

# Decoder executor client executable
add_executable(tree_decoder_executor_client_node src/tree_decoder_executor_client.cpp)
target_include_directories(tree_decoder_executor_client_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(tree_decoder_executor_client_node PRIVATE
  behavior_tree_decoder_base
  rclcpp_action::rclcpp_action
  ${auto_apms_interfaces_TARGETS}
  decoder_executor_client_params
)

# Install executables only
install(
  TARGETS 
  behavior_tree_encoder_node
  tree_decoder_publisher_node
  tree_decoder_executor_client_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
