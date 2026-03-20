cmake_minimum_required(VERSION 3.8)
project(auto_apms_behavior_codec_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/SerializedTreeMessage.msg"
  "msg/TreeXmlMessage.msg"
  "msg/ExecutorCommandMessage.msg"
  "msg/ExecutorFeedbackMessage.msg"
  "msg/SerializedTelemetryMessage.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
