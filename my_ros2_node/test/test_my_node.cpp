
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include "my_ros2_node/my_node.hpp"


using ::testing::_;
using ::testing::Invoke;
using ::testing::Return;

// Mock class for MyNode
class MockMyNode : public MyNode {
public:
  // MOCK_METHOD(void, topic_callback, (const std_msgs::msg::String::SharedPtr msg), (override));
  // MOCK_METHOD(void, handle_service, (const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
  //                                    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response), (override));
  MOCK_METHOD(void, topic_callback, (const std_msgs::msg::String::SharedPtr msg));
  MOCK_METHOD(void, handle_service, (const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                                     std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response));

};

// Test case for topic callback
TEST(MyNodeTest, TopicCallbackTest) {
  auto mock_node = std::make_shared<MockMyNode>();  // Create mock object

  std_msgs::msg::String::SharedPtr msg = std::make_shared<std_msgs::msg::String>();
  msg->data = "Hello, World!";

  // Set expectations
  EXPECT_CALL(*mock_node, topic_callback(msg)).Times(1);

  // Call the method under test
  mock_node->topic_callback(msg);
}

// Test case for service callback
TEST(MyNodeTest, ServiceCallbackTest) {
  auto mock_node = std::make_shared<MockMyNode>();  // Create mock object

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 3;
  request->b = 4;
  auto response = std::make_shared<example_interfaces::srv::AddTwoInts::Response>();

  // Set expectations
  EXPECT_CALL(*mock_node, handle_service(request, response)).WillOnce(Invoke([&](auto req, auto res) {
    res->sum = req->a + req->b;
  }));

  // Call the method under test
  mock_node->handle_service(request, response);

  // Check the result
  EXPECT_EQ(response->sum, 7);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);  // Initialize ROS 2 node

  int ret = RUN_ALL_TESTS();

  rclcpp::shutdown();  // Shutdown ROS 2 node
  return ret;
}
