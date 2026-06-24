#ifndef COMPOSITION__TEXT_RECOGNITION_COMPONENT_HPP_
#define COMPOSITION__TEXT_RECOGNITION_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rover_components/visibility_control.h"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <opencv2/opencv.hpp>

namespace rover_components {

class TextRecognitionService : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit TextRecognitionService(const rclcpp::NodeOptions &options);

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void handle_text_recognition_request(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::string detect_text(const cv::Mat &image);
  std::string simulate_text_recognition(const cv::Mat &roi);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

  std::string last_detected_text_;
  float confidence_threshold_;
  float nms_threshold_;
};

} // namespace rover_components

#endif // COMPOSITION__TEXT_RECOGNITION_COMPONENT_HPP_