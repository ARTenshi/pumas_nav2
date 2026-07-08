cmake_minimum_required(VERSION 3.5)
project(pumas_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/Joints.msg"
  "msg/StartAndEndJoints.msg"
  "msg/MotionPose.msg"
)

set(srv_files
  "srv/GetInflatedMap.srv"
  "srv/ParamReadWrite.srv"
  "srv/GetPlanWithVia.srv"
)

set(action_files
  "action/MotionSynthesis.action"
  "action/PumasNav.action"
  "action/GazeHead.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES nav_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
