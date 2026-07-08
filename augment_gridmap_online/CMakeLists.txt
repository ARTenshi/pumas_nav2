cmake_minimum_required(VERSION 3.8)
project(augment_gridmap_online)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(std_srvs REQUIRED)

find_package(rcl_interfaces REQUIRED)

find_package(yaml-cpp REQUIRED)

# C++ section
add_executable(augment_gridmap_online_node
    src/augment_gridmap_online_node.cpp
)

target_include_directories(augment_gridmap_online_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(augment_gridmap_online_node
    rclcpp

    ament_index_cpp

    geometry_msgs
    nav_msgs
    visualization_msgs

    std_srvs

    rcl_interfaces
)

target_link_libraries(augment_gridmap_online_node
    yaml-cpp
)

# Install public headers
install(DIRECTORY include/
    DESTINATION include
)

# Python section
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
#  ${PROJECT_NAME}/my_python.py
  DESTINATION lib/${PROJECT_NAME}
)

# Ensure both are included
install(TARGETS
augment_gridmap_online_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
