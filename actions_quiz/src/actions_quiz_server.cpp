#include "actions_quiz_msg/action/distance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Quaternion.h"

class ActionsQuizServer : public rclcpp::Node {
public:
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleDistance = rclcpp_action::ServerGoalHandle<Distance>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit ActionsQuizServer() : Node("actions_quiz_server_node") {
    using namespace std::placeholders;

    // Action server to accept goals
    this->action_server_ = rclcpp_action::create_server<Distance>(
        this, "/distance_as",
        std::bind(&ActionsQuizServer::handle_goal, this, _1, _2),
        std::bind(&ActionsQuizServer::handle_cancel, this, _1),
        std::bind(&ActionsQuizServer::handle_accepted, this, _1));

    // Publishers required by the quiz
    initial_pose_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);
    distance_left_pub_ =
        this->create_publisher<std_msgs::msg::Float64>("/distance_left", 10);
    distance_traveled_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/distance_traveled", 10);

    // Subscriber to track position
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&ActionsQuizServer::odom_callback, this, _1));

    // Action client for navigation to pose
    nav_to_pose_client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    // Variables initialization
    first_odom_ = true;
    total_distance_ = 0.0;
    current_x_ = 0.0;
    current_y_ = 0.0;

    // Wait for the localization node to be ready
    wait_for_localization();

    // Set the initial pose to (0, 0, 0)
    set_initial_pose(0.0, 0.0, 0.0);
  }

private:
  rclcpp_action::Server<Distance>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_traveled_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

  bool first_odom_;
  double total_distance_;
  double current_x_;
  double current_y_;
  double goal_x_;
  double goal_y_;

  void wait_for_localization() {
    RCLCPP_INFO(this->get_logger(), "Waiting for localization to be active...");

    while (this->count_subscribers("/initialpose") == 0 && rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for subscribers to /initialpose...");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    while (!nav_to_pose_client_->action_server_is_ready() && rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for the NavigateToPose action server...");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    RCLCPP_INFO(this->get_logger(), "Localization is active.");
  }

  void set_initial_pose(double x, double y, double yaw) {
    auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = this->get_clock()->now();

    initial_pose.pose.pose.position.x = x;
    initial_pose.pose.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    initial_pose.pose.pose.orientation.w = q.w();
    initial_pose.pose.pose.orientation.z = q.z();

    for (int i = 0; i < 10; ++i) {
      initial_pose_publisher_->publish(initial_pose);
      RCLCPP_INFO(this->get_logger(), "Publishing initial pose (%d/10)", i + 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(this->get_logger(),
                "Initial pose set to x: %.2f, y: %.2f, yaw: %.2f", x, y, yaw);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double last_x = current_x_;
    double last_y = current_y_;
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    if (first_odom_) {
      first_odom_ = false;
      return;
    }
    total_distance_ += std::sqrt(std::pow(current_x_ - last_x, 2) +
                                 std::pow(current_y_ - last_y, 2));

    // Dynamically update metrics to topics whenever odom ticks
    double dist_left = std::sqrt(std::pow(goal_x_ - current_x_, 2) +
                                 std::pow(goal_y_ - current_y_, 2));

    auto msg_left = std_msgs::msg::Float64();
    msg_left.data = dist_left;
    distance_left_pub_->publish(msg_left);

    auto msg_traveled = std_msgs::msg::Float64();
    msg_traveled.data = total_distance_;
    distance_traveled_pub_->publish(msg_traveled);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Distance::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with x: %.2f, y: %.2f, yaw: %.2f",
                goal->x, goal->y, goal->yaw);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleDistance> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDistance> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&ActionsQuizServer::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleDistance> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Distance::Result>();

    // Reset tracking configurations
    total_distance_ = 0.0;
    goal_x_ = goal->x;
    goal_y_ = goal->y;

    // Send navigation goal
    bool navigation_result =
        send_navigation_goal(goal->x, goal->y, goal->yaw, goal_handle);

    if (navigation_result) {
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
      result->success = true;
      result->distance_traveled = total_distance_;
      goal_handle->succeed(result);
    } else {
      RCLCPP_INFO(this->get_logger(), "Failed to reach goal :(");
      result->success = false;
      result->distance_traveled = total_distance_;
      goal_handle->abort(result);
    }
  }

  bool send_navigation_goal(
      double x, double y, double yaw,
      const std::shared_ptr<GoalHandleDistance> main_goal_handle) {
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_msg.pose.pose.orientation.w = q.w();
    goal_msg.pose.pose.orientation.z = q.z();

    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(),
                "Sending navigation goal to: x=%.2f, y=%.2f, yaw=%.2f", x, y,
                yaw);
    auto send_goal_future = nav_to_pose_client_->async_send_goal(goal_msg);

    if (send_goal_future.wait_for(std::chrono::seconds(10)) !=
        std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
      return false;
    }

    auto nav_goal_handle = send_goal_future.get();
    if (!nav_goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Navigation goal rejected");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Navigation goal accepted");

    auto get_result_future =
        nav_to_pose_client_->async_get_result(nav_goal_handle);

    // Loop until Nav2 result future resolves, providing live feedback updates
    while (get_result_future.wait_for(std::chrono::milliseconds(200)) !=
           std::future_status::ready) {

      // Manage action cancel requests directly inside the future tracker loop
      if (main_goal_handle->is_canceling()) {
        nav_to_pose_client_->async_cancel_goal(nav_goal_handle);
        return false;
      }

      auto feedback = std::make_shared<Distance::Feedback>();

      // Publish Action Feedback object tracking parameters
      double dist_left = std::sqrt(std::pow(goal_x_ - current_x_, 2) +
                                   std::pow(goal_y_ - current_y_, 2));
      feedback->distance_left = dist_left;
      main_goal_handle->publish_feedback(feedback);
    }

    auto nav_result = get_result_future.get();
    if (nav_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Navigation failed with status: %d",
                  static_cast<int>(nav_result.code));
      return false;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionsQuizServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}