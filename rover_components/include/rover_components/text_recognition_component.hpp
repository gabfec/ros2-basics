#ifndef COMPOSITION__TEXT_RECOGNITION_COMPONENT_HPP_
#define COMPOSITION__TEXT_RECOGNITION_COMPONENT_HPP_

#include "custom_interfaces/srv/text_recognition.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_components/visibility_control.h"
#include "sensor_msgs/msg/image.hpp"

namespace rover_components {

class TextRecognitionService : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit TextRecognitionService(const rclcpp::NodeOptions &options);

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void handle_text_recognition_request(
      const std::shared_ptr<custom_interfaces::srv::TextRecognition::Request>
          request,
      std::shared_ptr<custom_interfaces::srv::TextRecognition::Response>
          response);
  std::string detect_text(const cv::Mat &image);
  std::pair<std::string, std::array<int32_t, 4>>
  detect_text_with_bounding_box(const cv::Mat &image);
  std::string simulate_text_recognition(const cv::Mat &roi);
  void initialize_text_detection();
  std::string to_upper(const std::string &str);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Service<custom_interfaces::srv::TextRecognition>::SharedPtr srv_;
  std::string last_detected_text_;
  std::array<int32_t, 4> last_bounding_box_;
  float confidence_threshold_;
  float nms_threshold_;
};

} // namespace rover_components

#endif // COMPOSITION__TEXT_RECOGNITION_COMPONENT_HPP_