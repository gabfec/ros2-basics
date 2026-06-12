#include "rclcpp/rclcpp.hpp"
#include "services_quiz_srv/srv/turn.hpp"
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("turn_s_client_node");

  auto client = node->create_client<services_quiz_srv::srv::Turn>("/turn");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Interrupted while waiting for the service.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for service /turn to appear...");
  }

  auto request = std::make_shared<services_quiz_srv::srv::Turn::Request>();
  request->direction = "right";
  request->angular_velocity = 0.2;
  request->time = 10.0;

  RCLCPP_INFO(node->get_logger(),
              "Sending service request execution command...");
  auto result = client->async_send_request(request);

  // Wait for the action loop response to finish
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    if (result.get()->success) {
      RCLCPP_INFO(node->get_logger(),
                  "Service call successful. Rover executed maneuver.");
    } else {
      RCLCPP_ERROR(node->get_logger(), "Service server reported a failure.");
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service /turn.");
  }

  rclcpp::shutdown();
  return 0;
}