cmake_minimum_required(VERSION 3.11)
project(auto_apms_behavior_codec)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auto_apms_behavior_tree_core REQUIRED)
find_package(auto_apms_behavior_codec_interfaces REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Generate parameter headers employing https://github.com/PickNikRobotics/generate_parameter_library
generate_parameter_library(encoder_params "config/encoder_params.yaml")
generate_parameter_library(decoder_params "config/decoder_params.yaml")

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

# Internal static libraries (not installed)
add_library(behavior_tree_representation STATIC
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

# Decoder executable
add_executable(behavior_tree_decoder_node src/behavior_tree_decoder.cpp)
target_include_directories(behavior_tree_decoder_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(behavior_tree_decoder_node PRIVATE
  dictionary_manager
  rclcpp::rclcpp
  auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
  behavior_tree_representation
  ${auto_apms_behavior_codec_interfaces_TARGETS}
  decoder_params
)

# Add Dummy Telemetry Generator executable
add_executable(dummy_telemetry_generator src/dummy_telemetry_generator.cpp)

# Add include directory to dummy telemetry generator
target_include_directories(dummy_telemetry_generator PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link dummy telemetry generator with dependencies
target_link_libraries(dummy_telemetry_generator
  rclcpp::rclcpp
  behavior_tree_representation
  ${auto_apms_behavior_codec_interfaces_TARGETS}
)

# Install executables only
install(
  TARGETS behavior_tree_encoder_node
          behavior_tree_decoder_node
          dummy_telemetry_generator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
