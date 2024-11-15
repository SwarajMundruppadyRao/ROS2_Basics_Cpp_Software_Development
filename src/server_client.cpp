// Copyright (c) 2024 Swaraj Mundruppady Rao. All rights reserved.
// Licensed under the BSD 3-Clause License.
/**
 * BSD 3-Clause License
 * @file server_client.cpp
 * @brief ServiceClient class that creates a client for the service node
 * @author Swaraj Mundruppady Rao (swarajmr@umd.edu)
 * @version 1.0
 * @date 2024-11-04
 * @copyright Copyright (c) 2024 Swaraj Mundruppady Rao
 */

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "beginner_tutorials/srv/change_string.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief ServiceClient class that creates a client for the service node
 */
class ServiceClient : public rclcpp::Node {
 public:
  ServiceClient() : Node("server_client") {
    client = this->create_client<beginner_tutorials::srv::ChangeString>(
        "service_node");
  }

  /**
   * @brief Get the Request object
   * @param argv input string
   * @return std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
   */
  auto getRequest(char** argv) {
    auto request =
        std::make_shared<beginner_tutorials::srv::ChangeString::Request>();
    request->input = argv[1];
    return request;
  }

  /**
   * @brief Get the Response object
   * @return std::shared_ptr<beginner_tutorials::srv::ChangeString::Response>
   */
  rclcpp::Client<beginner_tutorials::srv::ChangeString>::SharedPtr client;
};

/**
 * @brief Main function that creates a client for the service node
 * @param argc number of arguments
 * @param argv input string
 * @return int
 */
int main(int argc, char** argv) {
  // Initialize the ROS2 client
  rclcpp::init(argc, argv);
  // Create a shared pointer to the ServiceClient object
  std::shared_ptr<ServiceClient> service_client =
      std::make_shared<ServiceClient>();
  // Wait for the service to be available
  while (!service_client->client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(service_client->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    // Log a message if the service is not available
    RCLCPP_INFO(service_client->get_logger(),
                "Service not available. Waiting again...");
  }

  // Create a request object
  auto request = service_client->getRequest(argv);
  // Send the request to the service node
  auto result = service_client->client->async_send_request(request);
  // Wait for the response
  if (rclcpp::spin_until_future_complete(service_client, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " change string '%s' ",
                result.get()->output.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service change_string");
  }
  // Shutdown the ROS2 client
  rclcpp::shutdown();
  return 0;
}
