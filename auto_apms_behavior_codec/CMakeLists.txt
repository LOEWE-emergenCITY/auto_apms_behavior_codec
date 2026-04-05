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
find_package(auto_apms_behavior_tree REQUIRED)
find_package(auto_apms_behavior_codec_interfaces REQUIRED)
find_package(auto_apms_interfaces REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Generate parameter headers employing https://github.com/PickNikRobotics/generate_parameter_library
generate_parameter_library(encoder_base_params "config/encoder_base_params.yaml")
generate_parameter_library(encoder_subscriber_params "config/encoder_subscriber_params.yaml")
generate_parameter_library(encoder_executor_proxy_params "config/encoder_executor_proxy_params.yaml")
generate_parameter_library(decoder_base_params "config/decoder_base_params.yaml")
generate_parameter_library(decoder_publisher_params "config/decoder_publisher_params.yaml")
generate_parameter_library(decoder_executor_client_params "config/decoder_executor_client_params.yaml")

# Fetch tinycbor
include(FetchContent)
set(BUILD_TOOLS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  tinycbor
  GIT_REPOSITORY https://github.com/intel/tinycbor.git
  GIT_TAG v7.0
  PATCH_COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/patch_tinycbor.cmake
)
FetchContent_MakeAvailable(tinycbor)

# Add export library
add_library(${PROJECT_NAME} SHARED
  src/telemetry_message_builder.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  tinycbor
)

# Add behavior tree representation source files
add_library(behavior_tree_representation
  src/behavior_tree_representation.cpp
  src/cbor_serialization.cpp
  src/cbor_deserialization.cpp
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

#
# ENCODING
#

# Encoder base static library
add_library(behavior_tree_encoder_base STATIC
  src/behavior_tree_encoder_base.cpp
)
target_include_directories(behavior_tree_encoder_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(behavior_tree_encoder_base PUBLIC
  dictionary_manager
  rclcpp::rclcpp
  auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
  behavior_tree_representation
  ${auto_apms_behavior_codec_interfaces_TARGETS}
  encoder_base_params
)

# Encoder subscriber executable
add_executable(tree_encoder_subscriber 
  src/tree_encoder_subscriber.cpp
)
target_include_directories(tree_encoder_subscriber PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(tree_encoder_subscriber PRIVATE
  behavior_tree_encoder_base
  encoder_subscriber_params
)

# Encoder executor proxy library
add_library(tree_encoder_executor_proxy_lib STATIC
  src/tree_encoder_executor_proxy.cpp
)
target_include_directories(tree_encoder_executor_proxy_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(tree_encoder_executor_proxy_lib PUBLIC
  behavior_tree_encoder_base
  ${PROJECT_NAME}
  rclcpp_action::rclcpp_action
  auto_apms_behavior_tree::auto_apms_behavior_tree
  ${auto_apms_interfaces_TARGETS}
  ${auto_apms_behavior_codec_interfaces_TARGETS}
  encoder_executor_proxy_params
)

# Encoder executor proxy executable
add_executable(tree_encoder_executor_proxy
  src/tree_encoder_executor_proxy_main.cpp
)
target_link_libraries(tree_encoder_executor_proxy PRIVATE
  tree_encoder_executor_proxy_lib
)

#
# DECODING
#

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
add_executable(tree_decoder_publisher 
  src/tree_decoder_publisher.cpp
)
target_include_directories(tree_decoder_publisher PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(tree_decoder_publisher PRIVATE
  behavior_tree_decoder_base
  decoder_publisher_params
)

# Decoder executor client executable
add_executable(tree_decoder_executor_client 
  src/tree_decoder_executor_client.cpp
)
target_include_directories(tree_decoder_executor_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(tree_decoder_executor_client PRIVATE
  behavior_tree_decoder_base
  rclcpp_action::rclcpp_action
  ${auto_apms_interfaces_TARGETS}
  decoder_executor_client_params
)

#
# EXPORT & INSTALLATION
#

# Install headers
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)
# Install library and export targets
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)
ament_export_targets(
  export_${PROJECT_NAME}
)

# Install executables
install(
  TARGETS 
  tree_encoder_subscriber
  tree_encoder_executor_proxy
  tree_decoder_publisher
  tree_decoder_executor_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Unit tests
  ament_add_gtest(test_unit
    test/test_executor_telemetry.cpp
    test/test_executor_command.cpp
  )
  target_link_libraries(test_unit
    ${PROJECT_NAME}
    auto_apms_behavior_tree::auto_apms_behavior_tree
  )
  target_include_directories(test_unit PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  # Integration tests
  ament_add_gtest(test_integration
    test/test_tree_encoder_executor_proxy.cpp
  )
  target_link_libraries(test_integration
    tree_encoder_executor_proxy_lib
  )
endif()

ament_package()
