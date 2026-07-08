cmake_minimum_required(VERSION 3.8)
project(motion_synth)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(builtin_interfaces REQUIRED)


# C++ section
#add_executable(hoge_node
#    src/hoge.cpp
#)

# Python section
#ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/motion_synth_server.py
  DESTINATION lib/${PROJECT_NAME}
)

## Ensure both are included
#install(TARGETS
#hoge_node  DESTINATION lib/${PROJECT_NAME}
#)

ament_package()
