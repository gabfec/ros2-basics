cmake_minimum_required(VERSION 3.8)
project(services_quiz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(services_quiz_srv REQUIRED)

# Server executable target mapping
add_executable(turn_s_server src/turn_s_server.cpp)
ament_target_dependencies(turn_s_server rclcpp geometry_msgs services_quiz_srv)

# Client executable target mapping
add_executable(turn_s_client src/turn_s_client.cpp)
ament_target_dependencies(turn_s_client rclcpp services_quiz_srv)

# Install steps for executables
install(TARGETS
  turn_s_server
  turn_s_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install step for launch files
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
