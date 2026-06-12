#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

class TextRecognitionService : public rclcpp::Node {
public:
  TextRecognitionService() : Node("text_recognition_service") {
    // Initialize text detection
    initializeTextDetection();

    // Subscribe to the image topic
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/leo/camera/image_raw", 1,
        std::bind(&TextRecognitionService::image_callback, this,
                  std::placeholders::_1));

    // Create a service to handle text recognition requests
    std::string name_service = "/text_recognition_service";
    service_ = this->create_service<std_srvs::srv::Trigger>(
        name_service,
        std::bind(&TextRecognitionService::handle_text_recognition_request,
                  this, std::placeholders::_1, std::placeholders::_2));

    // Variable to store the last detected text
    last_detected_text_ = "";

    RCLCPP_INFO(this->get_logger(), "%s Service Server Ready...",
                name_service.c_str());
  }

private:
  void initializeTextDetection() {
    // For a complete implementation, load the EAST model here
    confidence_threshold_ = 0.5;
    nms_threshold_ = 0.4;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert ROS image to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Perform text detection
    std::string detected_text = detectText(cv_ptr->image);

    // Store the last detected text
    if (!detected_text.empty()) {
      last_detected_text_ = detected_text;
    } else {
      last_detected_text_ = "";
    }

    RCLCPP_INFO(this->get_logger(), "Result: %s", last_detected_text_.c_str());
  }

  std::string detectText(const cv::Mat &image) {
    // Simplified text detection using basic OpenCV methods
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

        // Simulate text recognition (replace with actual OCR)
        std::string detected_text = simulateTextRecognition(roi);

        if (!detected_text.empty()) {
          return detected_text;
        }
      }
    }

    return ""; // No text detected
  }

  std::string simulateTextRecognition(const cv::Mat &roi) {
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

  void handle_text_recognition_request(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request; // Suppress unused parameter warning

    // Convert to uppercase for comparison
    std::string detected_text = last_detected_text_;
    std::transform(detected_text.begin(), detected_text.end(),
                   detected_text.begin(), ::toupper);

    // Respond with success = true if the detected text is "FOOD" or "WASTE"
    if (detected_text == "FOOD" || detected_text == "WASTE") {
      response->success = true;
    } else {
      response->success = false;
    }

    // Always return the detected text in the message element
    response->message =
        detected_text.empty() ? "No text detected" : detected_text;

    RCLCPP_INFO(
        this->get_logger(), "Service called. Detected text: %s, Success: %s",
        response->message.c_str(), response->success ? "true" : "false");
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

  std::string last_detected_text_;
  float confidence_threshold_;
  float nms_threshold_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TextRecognitionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}