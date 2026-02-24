cmake_minimum_required(VERSION 3.8)
project(auto_apms_behavior_codec)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auto_apms_behavior_tree_core REQUIRED)
find_package(auto_apms_behavior_codec_interfaces REQUIRED)



# Add dictionary manager source files
add_library(dictionary_manager
  src/dictionary_manager.cpp
)

# Add include directory (set after the target is created)
target_include_directories(dictionary_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#add behavior tree representation source files
add_library(behavior_tree_representation
  src/behavior_tree_representation.cpp
  src/cbor_serialization.cpp
  src/cbor_deserialization.cpp
)

# Add include directory for behavior_tree_representation target
target_include_directories(behavior_tree_representation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# add tinycbor source files
add_subdirectory(third_party/tinycbor)


# Link dictionary manager
target_link_libraries(dictionary_manager PUBLIC 
  auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
  tinycbor
  behavior_tree_representation
  ${auto_apms_codec_interfaces_TARGETS} # attempt at linking the interfaces, not working
)

target_link_libraries(behavior_tree_representation PUBLIC 
  auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
  tinycbor
  ${auto_apms_codec_interfaces_TARGETS}
)

# Add behavior tree encoder executable
add_executable(behavior_tree_encoder_node src/behavior_tree_encoder.cpp)

# Add include directory to encoder
target_include_directories(behavior_tree_encoder_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/../auto_apms_behavior_codec_interfaces/rosidl_generator_cpp> # this is not required in the example but without it the includes are not found
  $<INSTALL_INTERFACE:include>)

# Link encoder with dependencies
target_link_libraries(behavior_tree_encoder_node PRIVATE
  dictionary_manager
  rclcpp::rclcpp
  auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
  behavior_tree_representation
  ${auto_apms_codec_interfaces_TARGETS}  # attempt at linking the interfaces, not working
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
