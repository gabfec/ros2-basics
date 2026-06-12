#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "services_quiz_srv/srv/turn.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TurnServerNode : public rclcpp::Node {
public:
  TurnServerNode() : Node("turn_s_server_node") {
    // Service Server
    srv_server_ = this->create_service<services_quiz_srv::srv::Turn>(
        "/turn", std::bind(&TurnServerNode::handle_turn_request, this,
                           std::placeholders::_1, std::placeholders::_2));

    // Command Velocity Publisher
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Service Server /turn is now ready.");
  }

private:
  void handle_turn_request(
      const std::shared_ptr<services_quiz_srv::srv::Turn::Request> request,
      std::shared_ptr<services_quiz_srv::srv::Turn::Response> response) {
    RCLCPP_INFO(this->get_logger(),
                "Received Request: Direction=%s, Vel=%.2f, Time=%.2f",
                request->direction.c_str(), request->angular_velocity,
                request->time);

    auto msg = geometry_msgs::msg::Twist();

    // Assign proper sign for rotation direction
    if (request->direction == "left") {
      msg.angular.z = std::abs(request->angular_velocity);
    } else if (request->direction == "right") {
      msg.angular.z = -std::abs(request->angular_velocity);
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid direction input received!");
      response->success = false;
      return;
    }

    // Start Spinning
    publisher_->publish(msg);

    // Sleep precisely for the requested duration
    rclcpp::Rate loop_rate(10); // 10Hz checking
    auto start_time = this->now();
    double elapsed = 0.0;

    while (rclcpp::ok() && elapsed < request->time) {
      publisher_->publish(msg); // Maintain twist velocity output
      loop_rate.sleep();
      elapsed = (this->now() - start_time).seconds();
    }

    // Stop the robot completely
    auto stop_msg = geometry_msgs::msg::Twist();
    publisher_->publish(stop_msg);

    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Turn completed successfully.");
  }

  rclcpp::Service<services_quiz_srv::srv::Turn>::SharedPtr srv_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurnServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}