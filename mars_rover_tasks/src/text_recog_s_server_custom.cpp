#include <algorithm>
#include <cctype>
#include <custom_interfaces/srv/text_recognition.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

class TextRecognitionServiceCustom : public rclcpp::Node {
public:
  TextRecognitionServiceCustom() : Node("text_recognition_service_custom") {
    // Initialize text detection
    initializeTextDetection();

    // Subscribe to the image topic
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/leo/camera/image_raw", 1,
        std::bind(&TextRecognitionServiceCustom::image_callback, this,
                  std::placeholders::_1));

    // Create a service to handle text recognition requests
    std::string name_service = "/text_recognition_service_custom";
    service_ = this->create_service<custom_interfaces::srv::TextRecognition>(
        name_service,
        std::bind(
            &TextRecognitionServiceCustom::handle_text_recognition_request,
            this, std::placeholders::_1, std::placeholders::_2));

    // Variables to store the last detected text and bounding box
    last_detected_text_ = "";
    last_bounding_box_ = {0, 0, 0, 0}; // start_x, start_y, end_x, end_y

    RCLCPP_INFO(this->get_logger(),
                "%s CUSTOM Interface Service Server Ready...",
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

    // Perform text detection and extract bounding box
    auto result = detectTextWithBoundingBox(cv_ptr->image);

    // Store the last detected text and bounding box
    last_detected_text_ = result.first;
    last_bounding_box_ = result.second;

    RCLCPP_INFO(this->get_logger(), "Result: %s", last_detected_text_.c_str());
  }

  std::pair<std::string, std::array<int32_t, 4>>
  detectTextWithBoundingBox(const cv::Mat &image) {
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
        std::string detected_text = simulateTextRecognition(roi);

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

  std::string toUpper(const std::string &str) {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::toupper);
    return result;
  }

  void handle_text_recognition_request(
      const std::shared_ptr<custom_interfaces::srv::TextRecognition::Request>
          request,
      std::shared_ptr<custom_interfaces::srv::TextRecognition::Response>
          response) {
    std::string detected_text = toUpper(last_detected_text_);
    std::string requested_label = toUpper(request->label);

    if (detected_text == requested_label) {
      response->success = true;
      response->start_x = last_bounding_box_[0];
      response->start_y = last_bounding_box_[1];
      response->end_x = last_bounding_box_[2];
      response->end_y = last_bounding_box_[3];
    } else {
      response->success = false;
      response->start_x = response->start_y = response->end_x =
          response->end_y = 0;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Service called. Requested label: %s, Detected text: %s, Success: %s",
        request->label.c_str(), detected_text.c_str(),
        response->success ? "true" : "false");
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Service<custom_interfaces::srv::TextRecognition>::SharedPtr service_;

  std::string last_detected_text_;
  std::array<int32_t, 4> last_bounding_box_;
  float confidence_threshold_;
  float nms_threshold_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TextRecognitionServiceCustom>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}