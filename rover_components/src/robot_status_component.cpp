#include "rover_components/robot_status_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace rover_components {
RobotStatusService::RobotStatusService(const rclcpp::NodeOptions &options)
    : Node("robot_status_service", options) {
  // Create a service that will handle status queries from mission control
  srv_ = create_service<std_srvs::srv::Trigger>(
      "get_robot_status",
      std::bind(&RobotStatusService::get_status_callback, this,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Robot Status Service Component Ready...");
}

void RobotStatusService::get_status_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request; // Suppress unused parameter warning

  // Generate simulated rover status data for Mars mission
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> battery_dist(20.0, 100.0);
  std::uniform_real_distribution<> temp_dist(25.0, 80.0);
  std::uniform_int_distribution<> bool_dist(0, 1);

  double battery_level = battery_dist(gen);
  double temperature = temp_dist(gen);
  bool camera_status = bool_dist(gen);
  bool motor_statuses[4] = {
      static_cast<bool>(bool_dist(gen)), static_cast<bool>(bool_dist(gen)),
      static_cast<bool>(bool_dist(gen)), static_cast<bool>(bool_dist(gen))};

  // Construct response message for mission control
  response->success = true;

  std::ostringstream status_report;
  status_report << "Battery Level: " << std::fixed << std::setprecision(2)
                << battery_level << "%, "
                << "Temperature: " << std::fixed << std::setprecision(2)
                << temperature << "°C, "
                << "Camera: " << (camera_status ? "Operational" : "Faulty")
                << ", "
                << "Motor 1: " << (motor_statuses[0] ? "Operational" : "Faulty")
                << ", "
                << "Motor 2: " << (motor_statuses[1] ? "Operational" : "Faulty")
                << ", "
                << "Motor 3: " << (motor_statuses[2] ? "Operational" : "Faulty")
                << ", "
                << "Motor 4: " << (motor_statuses[3] ? "Operational" : "Faulty")
                << ", ";

  response->message = status_report.str();

  RCLCPP_INFO(this->get_logger(), "Status report requested by mission control");
}

} // namespace rover_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rover_components::RobotStatusService)