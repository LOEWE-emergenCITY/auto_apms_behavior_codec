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
find_package(rclcpp_components REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Generate parameter headers employing https://github.com/PickNikRobotics/generate_parameter_library
generate_parameter_library(encoder_base_params "config/encoder_base_params.yaml")
generate_parameter_library(encoder_subscriber_params "config/encoder_subscriber_params.yaml")
generate_parameter_library(encoder_executor_proxy_params "config/encoder_executor_proxy_params.yaml")
generate_parameter_library(decoder_base_params "config/decoder_base_params.yaml")
generate_parameter_library(decoder_publisher_params "config/decoder_publisher_params.yaml")
generate_parameter_library(decoder_executor_client_params "config/decoder_executor_client_params.yaml")
generate_parameter_library(decoder_executor_params "config/decoder_executor_params.yaml")

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
  src/util.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  tinycbor
)

# Ensure static libraries can be linked into shared libraries
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Add behavior tree representation source files
add_library(behavior_tree_representation STATIC
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

# Encoder subscriber component
add_library(tree_encoder_subscriber_component SHARED
  src/tree_encoder_subscriber.cpp
)
target_include_directories(tree_encoder_subscriber_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(tree_encoder_subscriber_component PUBLIC
  behavior_tree_encoder_base
  rclcpp_components::component
  encoder_subscriber_params
)
rclcpp_components_register_node(tree_encoder_subscriber_component
  PLUGIN "auto_apms_behavior_codec::TreeEncoderSubscriber"
  EXECUTABLE tree_encoder_subscriber
)

# Encoder executor proxy
add_library(tree_encoder_executor_proxy_component SHARED
  src/tree_encoder_executor_proxy.cpp
)
target_include_directories(tree_encoder_executor_proxy_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(tree_encoder_executor_proxy_component PUBLIC
  behavior_tree_encoder_base
  ${PROJECT_NAME}
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  auto_apms_behavior_tree::auto_apms_behavior_tree
  ${auto_apms_interfaces_TARGETS}
  ${auto_apms_behavior_codec_interfaces_TARGETS}
  encoder_executor_proxy_params
)
rclcpp_components_register_node(tree_encoder_executor_proxy_component
  PLUGIN "auto_apms_behavior_codec::TreeEncoderExecutorProxy"
  EXECUTABLE tree_encoder_executor_proxy
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

# Decoder publisher component
add_library(tree_decoder_publisher_component SHARED
  src/tree_decoder_publisher.cpp
)
target_include_directories(tree_decoder_publisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(tree_decoder_publisher_component PUBLIC
  behavior_tree_decoder_base
  rclcpp_components::component
  decoder_publisher_params
)
rclcpp_components_register_node(tree_decoder_publisher_component
  PLUGIN "auto_apms_behavior_codec::TreeDecoderPublisher"
  EXECUTABLE tree_decoder_publisher
)

# Decoder executor client component
add_library(tree_decoder_executor_client_component SHARED
  src/tree_decoder_executor_client.cpp
)
target_include_directories(tree_decoder_executor_client_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(tree_decoder_executor_client_component PUBLIC
  behavior_tree_decoder_base
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  ${auto_apms_interfaces_TARGETS}
  decoder_executor_client_params
)
rclcpp_components_register_node(tree_decoder_executor_client_component
  PLUGIN "auto_apms_behavior_codec::TreeDecoderExecutorClient"
  EXECUTABLE tree_decoder_executor_client
)

# Decoder executor component (remote endpoint for TreeEncoderExecutorProxy)
add_library(tree_decoder_executor_component SHARED
  src/tree_decoder_executor.cpp
)
target_include_directories(tree_decoder_executor_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(tree_decoder_executor_component PUBLIC
  behavior_tree_decoder_base
  ${PROJECT_NAME}
  rclcpp_components::component
  auto_apms_behavior_tree::auto_apms_behavior_tree
  ${auto_apms_behavior_codec_interfaces_TARGETS}
  decoder_executor_params
)
rclcpp_components_register_node(tree_decoder_executor_component
  PLUGIN "auto_apms_behavior_codec::TreeDecoderExecutor"
  EXECUTABLE tree_decoder_executor
)

#
# EXPORT & INSTALLATION
#

# Install headers
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)
# Install library and export targets
ament_export_targets(
  export_${PROJECT_NAME}
)
install(
  TARGETS 
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Install component libraries (loaded at runtime by the component container)
install(
  TARGETS
  tree_encoder_subscriber_component
  tree_encoder_executor_proxy_component
  tree_decoder_publisher_component
  tree_decoder_executor_client_component
  tree_decoder_executor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Unit tests
  ament_add_gtest(test_unit
    test/test_type_payload_string_message.cpp
    test/test_executor_feedback.cpp
    test/test_executor_command.cpp
    test/test_tree_encoding_decoding.cpp
  )
  target_link_libraries(test_unit
    ${PROJECT_NAME}
    auto_apms_behavior_tree::auto_apms_behavior_tree
    tinycbor
    behavior_tree_representation
    behavior_tree_decoder_base
    behavior_tree_encoder_base
    tree_decoder_publisher_component
  )
  target_include_directories(test_unit PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # Integration tests
  ament_add_gtest(test_integration
    test/test_tree_encoder_executor_proxy.cpp
  )
  target_link_libraries(test_integration
    tree_encoder_executor_proxy_component
  )
endif()

ament_package()
