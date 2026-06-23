cmake_minimum_required(VERSION 3.8)
project(rover_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(std_srvs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_library(robot_status_component SHARED src/robot_status_component.cpp)
target_compile_definitions(robot_status_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(robot_status_component
  "rclcpp"
  "rclcpp_components"
  "std_srvs")
  
rclcpp_components_register_nodes(robot_status_component "rover_components::RobotStatusService")

# Add the text recognition component library
add_library(text_recognition_component SHARED src/text_recognition_component.cpp)
target_compile_definitions(text_recognition_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(text_recognition_component
  "rclcpp"
  "rclcpp_components"
  "custom_interfaces"
  "sensor_msgs"
  "cv_bridge"
  "OpenCV")

# Register the text recognition component
rclcpp_components_register_nodes(text_recognition_component "rover_components::TextRecognitionService")

install(TARGETS
  robot_status_component
  text_recognition_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install headers if you have any
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
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
