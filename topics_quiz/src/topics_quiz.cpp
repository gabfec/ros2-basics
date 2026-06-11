#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class MarsRoverNavigationNode : public rclcpp::Node {
public:
  struct Position {
    double x;
    double y;
  };

  MarsRoverNavigationNode() : Node("topics_quiz_node") {
    // Subscriber to Laser Scan
    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser_scan", 10,
        std::bind(&MarsRoverNavigationNode::laserscan_callback, this,
                  std::placeholders::_1));

    // Subscriber to Odometry
    subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&MarsRoverNavigationNode::odom_callback, this,
                  std::placeholders::_1));

    // Subscriber to NASA Mission
    subscriber_nasa_ = this->create_subscription<std_msgs::msg::String>(
        "/nasa_mission", 10,
        std::bind(&MarsRoverNavigationNode::mission_callback, this,
                  std::placeholders::_1));

    // Publisher for movement commands
    publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Initializing positions and states matching instructions
    home_position_ = {0.0, 0.0};
    pickup_position_ = {-2.342, -2.432};
    current_position_ = {0.0, 0.0};

    current_yaw_ = 0.0;
    has_active_goal_ = false;
    mission_active_ = false;
    goal_tolerance_ = 0.1;
    odom_received_ = false; // Guard variable to guarantee odom is ready

    RCLCPP_INFO(this->get_logger(), "Mars Rover Navigation Node Ready...");
  }

  // Position Getters required for main() helper checks
  Position get_home_position() const { return home_position_; }
  Position get_pickup_position() const { return pickup_position_; }
  bool is_odom_received() const { return odom_received_; }

  bool is_at_position(const Position &target_position) {
    double distance_to_target =
        calculate_distance(current_position_, target_position);
    return distance_to_target < goal_tolerance_;
  }

  // Public method to force home target initialization from main()
  void set_initial_home_target() {
    target_position_ = home_position_;
    mission_active_ = true;
    has_active_goal_ = true;
  }

private:
  void stop_rover() {
    auto stop_msg = geometry_msgs::msg::Twist();
    publisher->publish(stop_msg);
  }

  double calculate_distance(const Position &pos1, const Position &pos2) {
    return std::sqrt(std::pow(pos1.x - pos2.x, 2) +
                     std::pow(pos1.y - pos2.y, 2));
  }

  // HINT 2: Laser Scan Processing for Obstacle Avoidance
  bool
  has_front_obstacles(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg) {
    int total_samples = laser_msg->ranges.size();

    int front_left_start = 101;
    int front_left_end = 133;
    int front_right_start = 67;
    int front_right_end = 100;

    // Adjust indices dynamically if simulation environment uses high-res array
    // sizes
    if (total_samples > 200) {
      int center = total_samples / 2;
      int span = total_samples / 12; // ~30 degree sweep window
      front_right_start = center - span;
      front_right_end = center;
      front_left_start = center;
      front_left_end = center + span;
    }

    double obstacle_threshold = 0.8; // meters

    for (int i = front_right_start; i <= front_left_end; ++i) {
      if (i >= 0 && i < total_samples) {
        if (laser_msg->ranges[i] < obstacle_threshold &&
            laser_msg->ranges[i] > laser_msg->range_min) {
          return true;
        }
      }
    }
    return false;
  }

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // If no active goal or if odom hasn't initialized yet, do not drive!
    if (!has_active_goal_ || !odom_received_) {
      stop_rover();
      return;
    }

    auto action = geometry_msgs::msg::Twist();

    if (has_front_obstacles(msg)) {
      action.linear.x = 0.0;
      action.angular.z = 0.4;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Obstacle ahead! Evading...");
    } else {
      double desired_yaw = std::atan2(target_position_.y - current_position_.y,
                                      target_position_.x - current_position_.x);

      double yaw_error = desired_yaw - current_yaw_;
      yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

      if (std::abs(yaw_error) > 0.2) {
        action.linear.x = 0.05;
        action.angular.z = (yaw_error > 0) ? 0.4 : -0.4;
      } else {
        action.linear.x = 0.4;
        action.angular.z = 0.0;
      }
    }

    publisher->publish(action);
  }

  // HINT 1: Odometry-based Position Tracking
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Update the current position and yaw from odometry
    current_position_.x = msg->pose.pose.position.x;
    current_position_.y = msg->pose.pose.position.y;

    // Extract yaw from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);

    // Mark odometry as verified and updated
    odom_received_ = true;

    // Check if the goal has been reached
    if (has_active_goal_) {
      double distance_to_goal =
          calculate_distance(current_position_, target_position_);
      if (distance_to_goal < goal_tolerance_) {
        stop_rover();
        mission_active_ = false;
        has_active_goal_ = false;
        RCLCPP_INFO(this->get_logger(), "Goal reached. Mars rover stopped.");
      }
    }
  }

  // HINT 3: Mission Command Processing
  void mission_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string command = msg->data;

    if (command == "Go-Home") {
      if (!is_at_position(home_position_)) {
        RCLCPP_INFO(this->get_logger(),
                    "NASA Command: Go-Home. Returning to base...");
        target_position_ = home_position_;
        mission_active_ = true;
        has_active_goal_ = true;
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "NASA Command: Go-Home. Already at Home position");
      }

    } else if (command == "Go-Pickup") {
      if (!is_at_position(pickup_position_)) {
        RCLCPP_INFO(this->get_logger(),
                    "NASA Command: Go-Pickup. Moving to sample site...");
        target_position_ = pickup_position_;
        mission_active_ = true;
        has_active_goal_ = true;
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "NASA Command: Go-Home. Already at Pickup position");
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_nasa_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

  Position home_position_;
  Position pickup_position_;
  Position current_position_;
  Position target_position_;

  double current_yaw_;
  double goal_tolerance_;
  bool has_active_goal_;
  bool mission_active_;
  bool odom_received_;
};

// HINT 4: Initial Position Detection Layout
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MarsRoverNavigationNode>();

  // Loop spin_some until we explicitly guarantee a real, populated /odom
  // message has been handled
  rclcpp::WallRate loop_rate(10);
  while (rclcpp::ok() && !node->is_odom_received()) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  // Check if rover needs to move to a known waypoint initially
  if (!node->is_at_position(node->get_home_position()) &&
      !node->is_at_position(node->get_pickup_position())) {
    RCLCPP_INFO(node->get_logger(),
                "Initial position unknown. Moving to home base.");
    node->set_initial_home_target();
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}