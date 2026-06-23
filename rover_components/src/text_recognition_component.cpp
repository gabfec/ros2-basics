#include "rover_components/text_recognition_component.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "custom_interfaces/srv/text_recognition.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace rover_components {
TextRecognitionService::TextRecognitionService(
    const rclcpp::NodeOptions &options)
    : Node("text_recognition_service", options) {
  // Initialize text detection
  initialize_text_detection();

  // Subscribe to the image topic
  image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/leo/camera/image_raw", 1,
      std::bind(&TextRecognitionService::image_callback, this,
                std::placeholders::_1));

  // Create a service that will handle text recognition requests
  srv_ = create_service<custom_interfaces::srv::TextRecognition>(
      "text_recognition_service",
      std::bind(&TextRecognitionService::handle_text_recognition_request, this,
                std::placeholders::_1, std::placeholders::_2));

  // Variables to store the last detected text and bounding box
  last_detected_text_ = "";
  last_bounding_box_ = {0, 0, 0, 0}; // start_x, start_y, end_x, end_y

  RCLCPP_INFO(this->get_logger(),
              "Text Recognition Service Component Ready...");
}

void TextRecognitionService::initialize_text_detection() {
  // For a complete implementation, load the EAST model here
  confidence_threshold_ = 0.5;
  nms_threshold_ = 0.4;
}

void TextRecognitionService::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  // Convert ROS image to OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Perform text detection and extract bounding box
  auto result = detect_text_with_bounding_box(cv_ptr->image);

  // Store the last detected text and bounding box
  last_detected_text_ = result.first;
  last_bounding_box_ = result.second;

  if (!last_detected_text_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Detected: %s",
                last_detected_text_.c_str());
  }
}

std::pair<std::string, std::array<int32_t, 4>>
TextRecognitionService::detect_text_with_bounding_box(const cv::Mat &image) {
  // Simplified text detection with bounding box extraction
  cv::Mat gray, thresh;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  // Find contours that might contain text
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(thresh, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  for (const auto &contour : contours) {
    cv::Rect bounding_rect = cv::boundingRect(contour);

    // Filter by size (assuming text boxes have reasonable dimensions)
    if (bounding_rect.width > 50 && bounding_rect.height > 20 &&
        bounding_rect.width < 200 && bounding_rect.height < 100) {

      // Extract ROI
      cv::Mat roi = image(bounding_rect);

      // Simulate text recognition
      std::string detected_text = simulate_text_recognition(roi);

      if (!detected_text.empty()) {
        std::array<int32_t, 4> bbox = {
            static_cast<int32_t>(bounding_rect.x),
            static_cast<int32_t>(bounding_rect.y),
            static_cast<int32_t>(bounding_rect.x + bounding_rect.width),
            static_cast<int32_t>(bounding_rect.y + bounding_rect.height)};
        return {detected_text, bbox};
      }
    }
  }

  return {"", {0, 0, 0, 0}}; // No text detected
}

std::string
TextRecognitionService::simulate_text_recognition(const cv::Mat &roi) {
  // Simple heuristic based on color patterns
  cv::Scalar mean_color = cv::mean(roi);

  // Simple heuristic based on the expected label colors
  if (mean_color[1] > mean_color[0] && mean_color[1] > mean_color[2]) {
    return "FOOD"; // Greenish tint might indicate FOOD label
  } else if (mean_color[0] > mean_color[1] && mean_color[2] < mean_color[0]) {
    return "WASTE"; // Reddish tint might indicate WASTE label
  }

  return ""; // No text detected
}

std::string TextRecognitionService::to_upper(const std::string &str) {
  std::string result = str;
  std::transform(result.begin(), result.end(), result.begin(), ::toupper);
  return result;
}

void TextRecognitionService::handle_text_recognition_request(
    const std::shared_ptr<custom_interfaces::srv::TextRecognition::Request>
        request,
    std::shared_ptr<custom_interfaces::srv::TextRecognition::Response>
        response) {
  std::string detected_text = to_upper(last_detected_text_);
  std::string requested_label = to_upper(request->label);

  if (detected_text == requested_label) {
    response->success = true;
    response->start_x = last_bounding_box_[0];
    response->start_y = last_bounding_box_[1];
    response->end_x = last_bounding_box_[2];
    response->end_y = last_bounding_box_[3];
  } else {
    response->success = false;
    response->start_x = response->start_y = response->end_x = response->end_y =
        0;
  }

  RCLCPP_INFO(
      this->get_logger(),
      "Service called. Requested label: %s, Detected text: %s, Success: %s",
      request->label.c_str(), detected_text.c_str(),
      response->success ? "true" : "false");
}

} // namespace rover_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rover_components::TextRecognitionService)
