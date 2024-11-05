
/**
 * BSD 3-Clause License
 * @file publisher_member_function.cpp
 * @brief ROS2 Node with Publisher and a service demonstrating the use of member functions as callbacks
 * @version 1.0 
 * @date 2024-11-04
 * @author Swaraj Mundruppady Rao
 * @copyright Copyright (c) 2024
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "beginner_tutorials/srv/change_string.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Parameter Types
using PARAMETER_EVENT = std::shared_ptr<rclcpp::ParameterEventHandler>;
using PARAMETER_HANDLE = std::shared_ptr<rclcpp::ParameterCallbackHandle>;


/**
 * @class MinimalPublisher
 * @brief A ROS2 node that publishes messages to a topic and provides a service to change the message content.
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for MinimalPublisher.
   * Initializes the node and sets up the publisher and service.
   */
  MinimalPublisher() : Node("publisher"), count_(0) {
    try {
      // Create a publisher to the "topic" topic with a queue size of 10.
      publisher_ = this-> create_publisher<std_msgs::msg::String>("topic", 10);

      // Declare a parameter with a default value of 2.
      auto param_desc = rcl_interfaces::msg::ParameterDescriptor();

      // Declare a parameter "frequency" with a default value of 2.
      param_desc.description =" This parameter is updated by the given input "
                              "argument in the launch file and is used by"
                              " the publisher and subscriber"
                              " to display the message.";
      this->declare_parameter("frequency", 2, param_desc);

      // Get the parameter value.
      auto frequency = this->get_parameter("frequency")
                        .get_parameter_value().get<int>();

      RCLCPP_INFO_STREAM(this->get_logger(), " Parameter Value: " << frequency);

      // Create a timer that triggers the
      // timer_callback function at the specified frequency.
      timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/frequency)
                , std::bind(&MinimalPublisher::timer_callback, this));

      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialize the Publisher");

      // Create a service that allows changing the base output string.
      service_ = this->create_service<beginner_tutorials::srv::ChangeString>
                ("service_node",
                 std::bind(&MinimalPublisher::changeString, this,
                  std::placeholders::_1, std::placeholders::_2));
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialize the Service");
    } catch (...) {
      // Log an error and a fatal message if an
      // exception occurs during initialization.
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Error during Initialization");
      RCLCPP_FATAL_STREAM(this->get_logger(),
                            "The publisher may not work as expected");
    }
  }

  /**
   * @brief Callback function that handles incoming service requests to change the base output string.
   * @param request The service request containing the new base output string.
   * @param response The service response containing the modified output string.
   */
  void changeString(const std::shared_ptr<beginner_tutorials
                    ::srv::ChangeString::Request> request,
                    std::shared_ptr<beginner_tutorials
                    ::srv::ChangeString::Response> response) {
    response->output = request->input + " Edited by the service node";
    service_response_message = response->output;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
      "Incoming request\ninput: '%s'",
      request->input.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
      "sending back response: '%s'",
      response->output.c_str());
  }

 private:
  /**
   * @brief Timer callback function that publishes messages to the topic at regular intervals.
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = service_response_message;

    // Log the message data.
    RCLCPP_DEBUG_STREAM(this->get_logger(), " Inserting the message data");

    // Publish the message.
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  ///< Timer object to trigger the timer callback.
  rclcpp::TimerBase::SharedPtr timer_;

  ///< Publisher object to publish messages.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  ///< Service object to handle service requests.
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service_;

  ///< Message to be published, updated by the service.
  std::string service_response_message{" Hi, This is Swaraj"};

  ///< Counter for the number of messages published.
  size_t count_;
};

int main(int argc, char* argv[]) {
  // Initialize the ROS2 system.
  rclcpp::init(argc, argv);

  // Create a shared pointer to the MinimalPublisher node.
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);

  // Shutdown the ROS2 system.
  rclcpp::shutdown();

  RCLCPP_WARN_STREAM(node->get_logger(), "Shutting Down!!");
  return 0;
}

