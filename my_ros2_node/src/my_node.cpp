#include "my_node.hpp"

MyNode::MyNode()
  : Node("my_node")
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic",
    10,
    std::bind(&MyNode::topic_callback, this, std::placeholders::_1)
  );

  service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
    "add_two_ints",
    std::bind(&MyNode::handle_service, this, std::placeholders::_1, std::placeholders::_2)
  );

  client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
}

void MyNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
}

void MyNode::handle_service(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                            std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
  response->sum = request->a + request->b;
  RCLCPP_INFO(this->get_logger(), "Incoming request: a=%ld, b=%ld", request->a, request->b);
  RCLCPP_INFO(this->get_logger(), "Sending back response: [%ld]", response->sum);
}
