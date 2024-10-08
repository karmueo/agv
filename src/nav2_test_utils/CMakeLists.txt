cmake_minimum_required(VERSION 3.5)
project(nav2_test_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  nav2_msgs
)

add_executable(nav2_client_util
  src/nav2_client_util.cpp
)
ament_target_dependencies(nav2_client_util
  ${dependencies}
)

add_executable(clicked_point_to_pose
  src/clicked_point_to_pose.cpp
)
ament_target_dependencies(clicked_point_to_pose
  ${dependencies}
)

add_executable(goal_listener
  src/goal_listener.cpp
)
ament_target_dependencies(goal_listener
  ${dependencies}
)

install(TARGETS nav2_client_util clicked_point_to_pose goal_listener
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
