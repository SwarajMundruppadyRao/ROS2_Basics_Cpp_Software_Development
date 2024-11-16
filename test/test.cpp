#include <chrono>
#include <catch_ros2/catch_ros2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "beginner_tutorials/srv/change_string.hpp"

class IntegrationTestFixture {
 public:
  IntegrationTestFixture()
      : node_(rclcpp::Node::make_shared("catch2_integration_test_node")),
        logger_(node_->get_logger()) {
    // Declare and fetch a test duration parameter
    node_->declare_parameter<double>("test_duration", 10.0);
    node_->get_parameter("test_duration", test_duration_);
    RCLCPP_INFO(logger_,
      "Integration test node initialized with test_duration = %.2f seconds.",
      test_duration_);
  }

  ~IntegrationTestFixture() {
    RCLCPP_INFO(logger_, "Integration test node shutting down.");
  }

 protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  double test_duration_;
};

////////////////////////////////////////////////
// Test Case 1: Service Interaction Test
////////////////////////////////////////////////

/**
 * @brief Test case to check if the service node is able to respond to requests
 */

TEST_CASE_METHOD(IntegrationTestFixture,
  "test service interaction", "/service_node") {
  auto client = node_->create_client
    <beginner_tutorials::srv::ChangeString>("service_node");
  RCLCPP_INFO(logger_, "Service client created for 'service_node'.");
  rclcpp::Time start_time = rclcpp::Clock().now();
  bool service_available = false;
  rclcpp::Duration duration = rclcpp::Duration::from_seconds(0);
  RCLCPP_INFO(logger_, "Performing Test...");

  auto timeout = std::chrono::milliseconds(
                  static_cast<int>(test_duration_ * 1000));
  if (client->
              wait_for_service(timeout)) {
    service_available = true;
    duration = rclcpp::Clock().now() - start_time;

    auto request = std::make_shared
          <beginner_tutorials::srv::ChangeString::Request>();
    request->input = "Test Message";

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto result = future.get();
      REQUIRE(result->output == "Test Message Edited by the service node");
      RCLCPP_INFO(logger_,
        "Service interaction test passed with response: '%s'",
        result->output.c_str());
    } else {
      RCLCPP_ERROR(logger_, "Failed to call service 'service_node'.");
      FAIL("Service call failed.");
    }
  }

  RCLCPP_INFO(logger_,
    "Service availability: %d after %.2f seconds.",
    service_available, duration.seconds());
  CHECK(service_available);
}

////////////////////////////////////////////////
// Test Case 2: Topic Publisher Test
////////////////////////////////////////////////
/**
 * @brief Test case to check if the topic publisher is able to publish messages
 */

TEST_CASE_METHOD(IntegrationTestFixture, "test topic chatter", "topic") {
  bool message_received = false;

  // Callback definition
  auto callback = [&](const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(logger_, "I heard: '%s'", msg->data.c_str());
    message_received = true;
  };

  // Create a subscriber
  auto subscription = node_->
    create_subscription<std_msgs::msg::String>("chatter", 10, callback);

  // Wait for the message
  rclcpp::Rate rate(10.0);
  auto start_time = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - start_time;
  auto timeout = rclcpp::Duration::from_seconds(test_duration_);
  RCLCPP_INFO(logger_, "Waiting for message on 'chatter'...");

  while (!message_received && duration < timeout) {
    rclcpp::spin_some(node_);
    rate.sleep();
    duration = rclcpp::Clock().now() - start_time;
  }

  RCLCPP_INFO(logger_,
    "Test finished after %.2f seconds. Message received: %d.",
    duration.seconds(), message_received);
  CHECK(message_received);
}
