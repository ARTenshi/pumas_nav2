cmake_minimum_required(VERSION 3.8)
project(leg_finder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)
install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
)

# Add executable
add_executable(leg_finder src/leg_finder.cpp)

ament_target_dependencies(leg_finder
  rclcpp 
  tf2_ros
  tf2_eigen
  std_msgs
  sensor_msgs
  tf2_geometry_msgs 
  geometry_msgs
  visualization_msgs
)

target_link_libraries(leg_finder ${PCL_LIBRARIES} )

# Install Cpp executables
install(TARGETS
  leg_finder
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(CMAKE_BUILD_TYPE Debug)

ament_package()
