#include <chrono>
#include <custom_interfaces/srv/text_recognition.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace std::chrono_literals;

class TextRecognitionClientCustom : public rclcpp::Node {
public:
  TextRecognitionClientCustom() : Node("text_recognition_client_custom") {
    // Create the Service Client object
    // This defines the name ('/text_recognition_service_custom') and type
    // (TextRecognition) of the Service Server to connect to.
    std::string name_service = "/text_recognition_service_custom";
    client_ = this->create_client<custom_interfaces::srv::TextRecognition>(
        name_service);

    // Wait for the service to be available (checks every second)
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service %s not available, waiting again...",
                  name_service.c_str());
    }
  }

  void send_request(const std::string &label) {
    // Create a TextRecognition request
    auto request =
        std::make_shared<custom_interfaces::srv::TextRecognition::Request>();
    request->label = label;

    // Send the request asynchronously
    auto result_future = client_->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = result_future.get();
      // Log the service response
      RCLCPP_INFO(this->get_logger(),
                  "Success: %s, Bounding Box: %d, %d, %d, %d",
                  response->success ? "true" : "false", response->start_x,
                  response->start_y, response->end_x, response->end_y);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    }
  }

private:
  rclcpp::Client<custom_interfaces::srv::TextRecognition>::SharedPtr client_;
};

int main(int argc, char **argv) {
  // Initialize the ROS communication
  rclcpp::init(argc, argv);

  // Declare the node constructor
  auto client = std::make_shared<TextRecognitionClientCustom>();

  // Label to search for (can be changed based on requirements)
  std::string label_to_find = "FOOD";

  // Run the send_request() method
  client->send_request(label_to_find);

  // Shutdown the ROS communication
  rclcpp::shutdown();
  return 0;
}