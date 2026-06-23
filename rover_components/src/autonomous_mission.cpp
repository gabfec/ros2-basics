#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rover_components/robot_status_component.hpp"
#include "rover_components/text_recognition_component.hpp"

int main(int argc, char *argv[]) {
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  // Deploy critical rover systems for autonomous operation during communication
  // blackouts. Robot status monitoring and text recognition components work
  // together for independent supply box identification and health monitoring.
  auto robot_status =
      std::make_shared<rover_components::RobotStatusService>(options);
  exec.add_node(robot_status);
  auto text_recognition =
      std::make_shared<rover_components::TextRecognitionService>(options);
  exec.add_node(text_recognition);

  // Mission executor will continue autonomous operations until manual
  // intervention or mission completion. Critical for Mars missions when Earth
  // communication is unavailable. Can only be interrupted by Ctrl-C or mission
  // abort signal.
  exec.spin();

  rclcpp::shutdown();

  return 0;
}