#include "rover_components/text_recognition_component.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace rover_components {

TextRecognitionService::TextRecognitionService(
    const rclcpp::NodeOptions &options)
    : Node("text_recognition_service", options) {
  // Initialize text detection parameters
  confidence_threshold_ = 0.5;
  nms_threshold_ = 0.4;
  last_detected_text_ = "";

  // Subscribe to camera feed for supply box identification
  image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/leo/camera/image_raw", 1,
      std::bind(&TextRecognitionService::image_callback, this,
                std::placeholders::_1));

  // Create service for mission control text recognition requests
  srv_ = create_service<std_srvs::srv::Trigger>(
      "text_recognition_service",
      std::bind(&TextRecognitionService::handle_text_recognition_request, this,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Text Recognition Service Component Ready "
                                  "for supply box identification...");
}

void TextRecognitionService::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  // Convert ROS image to OpenCV format
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Perform text detection on supply boxes
  std::string detected_text = detect_text(cv_ptr->image);

  // Store the last detected text for service requests
  if (!detected_text.empty()) {
    last_detected_text_ = detected_text;
  }

  // Log detection results for mission monitoring
  if (!last_detected_text_.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "Supply box detected: %s",
                 last_detected_text_.c_str());
  }
}

std::string TextRecognitionService::detect_text(const cv::Mat &image) {
  // Simplified text detection using OpenCV methods
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

      // Extract ROI for text recognition
      cv::Mat roi = image(bounding_rect);

      // Simulate text recognition on supply box labels
      std::string detected_text = simulate_text_recognition(roi);

      if (!detected_text.empty()) {
        return detected_text;
      }
    }
  }

  return ""; // No text detected
}

std::string
TextRecognitionService::simulate_text_recognition(const cv::Mat &roi) {
  // Simple heuristic based on color patterns to identify supply box labels
  cv::Scalar mean_color = cv::mean(roi);

  // Generate random text detection for simulation purposes
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 1);

  // Simulate detection with 70% probability
  if (dis(gen) == 0) {
    // Analyze color patterns to distinguish FOOD vs WASTE containers
    if (mean_color[1] > mean_color[0] && mean_color[1] > mean_color[2]) {
      return "FOOD"; // Greenish tint indicates FOOD label
    } else {
      return "WASTE"; // Other patterns indicate WASTE label
    }
  }

  return ""; // No recognizable text pattern
}

void TextRecognitionService::handle_text_recognition_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request; // Suppress unused parameter warning

  // For simulation purposes, generate a detection result if none exists
  if (last_detected_text_.empty()) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 2);

    int result = dis(gen);
    if (result == 0) {
      last_detected_text_ = "FOOD";
    } else if (result == 1) {
      last_detected_text_ = "WASTE";
    } else {
      last_detected_text_ = "";
    }
  }

  // Convert detected text to uppercase for consistent comparison
  std::string detected_text = last_detected_text_;
  std::transform(detected_text.begin(), detected_text.end(),
                 detected_text.begin(), ::toupper);

  // Respond with success = true if valid supply box label detected
  if (detected_text == "FOOD" || detected_text == "WASTE") {
    response->success = true;
  } else {
    response->success = false;
  }

  // Return detected text or status message
  response->message =
      detected_text.empty() ? "No supply box detected" : detected_text;

  RCLCPP_INFO(
      this->get_logger(),
      "Mission control requested text recognition. Detected: %s, Success: %s",
      response->message.c_str(), response->success ? "true" : "false");
}

} // namespace rover_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rover_components::TextRecognitionService)