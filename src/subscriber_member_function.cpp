/**
 * BSD 3-Clause License
 * @file subscriber_member_function.cpp
 * @brief Subscriber class that creates a subscriber for the service node
 * @version 1.0
 * @date 2024-11-04
 * @author Swaraj Mundruppady Rao (swarajmr@umd.edu)
 */

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @class MinimalSubscriber
 * @brief A simple ROS2 subscriber node that listens to a topic and logs the received messages.
 */
class MinimalSubscriber : public rclcpp::Node{
 public:
   /**
   * @brief Constructor for MinimalSubscriber.
   * Initializes the node and sets up the subscription.
   */
  MinimalSubscriber(): Node("minimal_subscriber") {
    try {
    // Create a subscription to the "topic" topic with a queue size of 10.
    // The TopicCallback method will be called
    // whenever a new message is received.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::TopicCallback, this, _1));
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Subscriber has been started.");
  } catch (...) {
    // Log an error and a fatal message
    // if an exception occurs during initialization.
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Error occurred in the constructor.");
    RCLCPP_FATAL_STREAM(this->get_logger(),
      "Fatal error occurred in the constructor.");
    }
  }

 private:
   /**
   * @brief Callback function that is called whenever a new message is received on the subscribed topic.
   * @param msg The received message.
   */
  void TopicCallback(const std_msgs::msg::String &msg) const {
    // Log the received message.
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  // A shared pointer to the subscription object.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  // Initialize the ROS2 system.
  rclcpp::init(argc, argv);

  // Create a shared pointer to the MinimalSubscriber node.
  auto node = std::make_shared<MinimalSubscriber>();

  // Spin the node to start the subscriber.
  rclcpp::spin(node);

  // Shutdown the ROS2 system.
  rclcpp::shutdown();

  // Log a warning message indicating that the node is shutting down.
  RCLCPP_WARN_STREAM(node->get_logger(), "Shutting Down!! " << 4);
  return 0;
}
