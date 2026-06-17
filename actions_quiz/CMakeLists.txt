cmake_minimum_required(VERSION 3.8)
project(actions_quiz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(actions_quiz_msg REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(actions_quiz_server src/actions_quiz_server.cpp)
ament_target_dependencies(actions_quiz_server
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  actions_quiz_msg
  tf2
  tf2_geometry_msgs
)

add_executable(actions_quiz_client src/actions_quiz_client.cpp)
ament_target_dependencies(actions_quiz_client
  rclcpp
  rclcpp_action
  actions_quiz_msg
)

install(TARGETS
  actions_quiz_server
  actions_quiz_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
