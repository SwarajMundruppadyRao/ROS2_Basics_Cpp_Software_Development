/**
 * @file test.cpp
 * @brief ROS2 Integration tests for MinimalPublisher and related nodes
 * @version 1.0
 * @date 2024-15-11
 * @copyright Swaraj Mundruppady Rao (swarajmr@umd.edu)
 */


/**
 * @file test.cpp
 * @brief ROS2 Integration tests for MinimalPublisher and related nodes
 * @version 1.0
 * @date 2024-15-11
 * @copyright Swaraj Mundruppady Rao
 */

#include <gtest/gtest.h>
#include <cstdio>
#include <string>
#include <thread>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <beginner_tutorials/srv/change_string.hpp>

class TaskPlanningFixture : public ::testing::Test {
 public:
  TaskPlanningFixture()
  : node_(std::make_shared<rclcpp::Node>("integration_test_node")),
    Logger(node_->get_logger()) {
    RCLCPP_INFO(Logger, "The test node has been initialized.");
  }

  void SetUp() override {
    // Start the MinimalPublisher node
    bool retVal = StartROSExec("beginner_tutorials", "publisher", "talker");
    ASSERT_TRUE(retVal);

    // Allow time for the publisher to start
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(Logger, "The publisher has been started.");
  }

  void TearDown() override {
    // Stop the publisher node
    bool retVal = StopROSExec();
    ASSERT_TRUE(retVal);
    RCLCPP_INFO(Logger, "The publisher has been stopped.");
  }

 protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger Logger;

  // Indicates if data was received
  bool has_data_ = false;
  // Stores the received message
  std::string received_message_ = "";

  // Expected message
  const std::string expected_message_ = " Hi, This is Swaraj";

  std::stringstream cmd_ss, cmdInfo_ss, killCmd_ss;

  bool StartROSExec(const char * pkg_name, const char * node_name,
    const char * exec_name) {
    cmd_ss << "ros2 run " << pkg_name << " "
            << exec_name << " > /dev/null 2> /dev/null &";
    cmdInfo_ss << "ros2 node info " << "/" <<
            node_name << " > /dev/null 2> /dev/null";

    // Construct kill command
    char execName[16];

    // Copy the executable name
    snprintf(execName, sizeof(execName), "%s", exec_name);
    killCmd_ss << "pkill --signal SIGINT " <<
              execName << " > /dev/null 2> /dev/null";

    // Stop any existing instance
    StopROSExec();

    // Start the ROS 2 node and wait for it to be ready
    int retVal = system(cmd_ss.str().c_str());
    if (retVal != 0) {
      return false;
      }

    retVal = -1;
    while (retVal != 0) {
      retVal = system(cmdInfo_ss.str().c_str());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return true;
  }

  bool StopROSExec() {
    if (killCmd_ss.str().empty()) {
      return true;
      }
    int retVal = system(killCmd_ss.str().c_str());
    return retVal == 0;
  }
};

TEST_F(TaskPlanningFixture, PublisherSubscriberTest) {
  // Create a subscriber to listen to the "chatter" topic
  auto subscription = node_->create_subscription<std_msgs::msg::String>(
    "chatter", 10,
    [&](const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO(Logger, "I heard: '%s'", msg->data.c_str());
      received_message_ = msg->data;
      has_data_ = true;
    });

  // Wait up to 5 seconds for a message
  rclcpp::Rate rate(10);
  auto start_time = std::chrono::steady_clock::now();
  while (!has_data_ &&
    std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::steady_clock::now() - start_time)
    .count() < 5) {
    rclcpp::spin_some(node_);
    rate.sleep();
  }

  // Verify the message matches the expected output
  ASSERT_TRUE(has_data_);
  ASSERT_EQ(received_message_, expected_message_);
}

TEST_F(TaskPlanningFixture, ServiceInteractionTest) {
  // Create a client to interact with the "service_node" service
  auto client = node_->
    create_client<beginner_tutorials::srv::ChangeString>("service_node");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  // Create a request
  auto request =
    std::make_shared<beginner_tutorials::srv::ChangeString::Request>();
  request->input = "Test Message";

  // Call the service and wait for the result
  auto result = client->async_send_request(request);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_, result),
            rclcpp::FutureReturnCode::SUCCESS);

  // Verify the service response
  ASSERT_EQ(result.get()->output, "Test Message Edited by the service node");
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
