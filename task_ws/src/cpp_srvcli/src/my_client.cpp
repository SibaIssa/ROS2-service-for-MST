#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
 
#include <chrono>
#include <cstdlib>
#include <memory>
 
using namespace std::chrono_literals;
 
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
 
  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
      return 1;
  }
   
  // Create the node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
   
  // Create the client for the node
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
 
  // Make the request
  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request-> a = atoll(argv[1]);
 
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      // Show an error if the user types CTRL + C
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    // Search for service nodes in the network
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
 
  // Send a request
  auto result = client->async_send_request(request);
   
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }
 
  rclcpp::shutdown();
  return 0;
}
