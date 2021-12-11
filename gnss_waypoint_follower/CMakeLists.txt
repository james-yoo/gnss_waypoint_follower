cmake_minimum_required(VERSION 3.5)
project(gnss_waypoint_follower)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(gnss_waypoint_follower_msgs REQUIRED)

include_directories(
  include
)

set(executable_name ${PROJECT_NAME}_node)
set(library_name ${executable_name}_core)

add_executable(${executable_name}
  src/main.cpp
)

add_library(${library_name} SHARED
  src/gnss_waypoint_follower.cpp
)

target_include_directories(${library_name} PRIVATE src/include)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
  gnss_waypoint_follower_msgs
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${executable_name}
  ${library_name}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()