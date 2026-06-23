#ifndef COMPOSITION__ROBOT_STATUS_COMPONENT_HPP_
#define COMPOSITION__ROBOT_STATUS_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rover_components/visibility_control.h"
#include "std_srvs/srv/trigger.hpp"

namespace rover_components {

class RobotStatusService : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit RobotStatusService(const rclcpp::NodeOptions &options);

private:
  void get_status_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};

} // namespace rover_components

#endif // COMPOSITION__ROBOT_STATUS_COMPONENT_HPP_