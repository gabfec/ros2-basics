#include "actions_quiz_msg/action/distance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <memory>

class ActionsQuizClient : public rclcpp::Node {
public:
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleDistance = rclcpp_action::ClientGoalHandle<Distance>;

  explicit ActionsQuizClient() : Node("actions_quiz_client_node") {
    this->client_ptr_ =
        rclcpp_action::create_client<Distance>(this, "/distance_as");
    // One-shot timer to trigger the goal sequence without blocking the thread
    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&ActionsQuizClient::send_goal, this));
  }

private:
  void send_goal() {
    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server /distance_as not available.");
      return;
    }

    auto goal_msg = Distance::Goal();
    goal_msg.x = 8.3;
    goal_msg.y = -2.2;
    goal_msg.yaw = -0.2;

    RCLCPP_INFO(this->get_logger(), "Sending goal: x=8.3, y=-2.2, yaw=-0.2");

    auto send_goal_options = rclcpp_action::Client<Distance>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&ActionsQuizClient::goal_response_callback, this,
                  std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&ActionsQuizClient::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(
        &ActionsQuizClient::result_callback, this, std::placeholders::_1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void
  goal_response_callback(const GoalHandleDistance::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server.");
    }
  }

  void
  feedback_callback(GoalHandleDistance::SharedPtr,
                    const std::shared_ptr<const Distance::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback: Distance left = %.2f meters",
                feedback->distance_left);
  }

  void result_callback(const GoalHandleDistance::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Action completed with success: %s",
                  result.result->success ? "True" : "False");
      RCLCPP_INFO(this->get_logger(), "Total distance traveled: %.2f meters",
                  result.result->distance_traveled);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
    }
    rclcpp::shutdown();
  }

  rclcpp_action::Client<Distance>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionsQuizClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}