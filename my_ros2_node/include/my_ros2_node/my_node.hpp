#ifndef MY_NODE_HPP_
#define MY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <functional>

class MyNode : public rclcpp::Node {
public:
  MyNode();

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg);
  void handle_service(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                      std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

#endif  // MY_NODE_HPP_
