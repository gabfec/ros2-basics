#include <csignal>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class MoveRoverNode : public rclcpp::Node {
public:
  MoveRoverNode() : Node("move_rover_node") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto timer_period = std::chrono::milliseconds(500);
    timer_ = this->create_wall_timer(
        timer_period, std::bind(&MoveRoverNode::timer_callback, this));
  }

private:
  void timer_callback() {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.5;
    msg.angular.z = 0.5;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f, angular.z=%.2f",
                msg.linear.x, msg.angular.z);
  }

public:
  void stop_rover() {
    auto stop_msg =
        geometry_msgs::msg::Twist(); // All fields default to zero, which
                                     // represents stopping the rover
    publisher_->publish(stop_msg);
    RCLCPP_INFO(this->get_logger(), "Publishing stop message before shutdown");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

std::shared_ptr<MoveRoverNode> simple_publisher;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  simple_publisher = std::make_shared<MoveRoverNode>();

  // Register the signal handler for CTRL+C
  // signal(SIGINT, signal_handler);

  rclcpp::spin(simple_publisher);

  // Right here, the spin loop has ended, but the context is still alive.
  // This is the perfectly safe window to send your final stop command.
  RCLCPP_INFO(simple_publisher->get_logger(),
              "Spin stopped. Stopping rover safely...");
  simple_publisher->stop_rover();

  // Reset the pointer explicitly to trigger the node's destructor
  // BEFORE calling rclcpp::shutdown().
  simple_publisher = nullptr;

  // 5. Clean up the ROS 2 context cleanly.
  rclcpp::shutdown();

  return 0;
}