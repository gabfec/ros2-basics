#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <random>
#include <string>

class TemperatureMonitorNode : public rclcpp::Node {
public:
  TemperatureMonitorNode(const std::string &rover_name,
                         double timer_period = 0.2)
      : Node(rover_name), temperature_threshold_(70.0), rd_(), gen_(rd_()),
        temp_dist_(20.0, 100.0) {
    // create a timer sending two parameters:
    // - the duration between two callbacks (timer_period seconds)
    // - the timer function (timer_callback)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(timer_period * 1000)),
        std::bind(&TemperatureMonitorNode::timer_callback, this));

    std::random_device rd;
    gen_ = std::mt19937(rd());
  }

private:
  void timer_callback() {
    std::uniform_real_distribution<double> temp_dist(20.0, 100.0);
    double temperature = temp_dist(gen_);

    if (temperature > 70) {
      RCLCPP_WARN(this->get_logger(),
                  "Warning! High temperature detected! %.2f°C", temperature);
    } else {
      RCLCPP_INFO(this->get_logger(), "Current temperature: %0.2f°C",
                  temperature);
    }
  }

  double temperature_threshold_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> temp_dist_;
};

int main(int argc, char **argv) {
  // initialize the ROS2 communication
  rclcpp::init(argc, argv);
  // declare the node constructor
  auto node = std::make_shared<TemperatureMonitorNode>("mars_rover_1", 1.0);
  // keeps the node alive, waits for a request to kill the node (ctrl+c)
  rclcpp::spin(node);
  // shutdown the ROS2 communication
  rclcpp::shutdown();
  return 0;
}