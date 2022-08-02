#include "rclcpp/rclcpp.hpp"
#include "mst_msgs/msg/mst_input.hpp"
#include "mst_msgs/srv/mst_output.hpp"
 
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
 
using namespace std::chrono_literals;
using namespace  std; 


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\nusage: Enter Your Graph as a sequence of edges in form «X-Y», where X and Y are nodes of the graph. Elements are separated with whitespace.\n\nEnter your Graph:\n ");
   
  // Create the node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("mst_client");
   
  // Create the client for the node
  rclcpp::Client<mst_msgs::srv::MstOutput>::SharedPtr client =
    node->create_client<mst_msgs::srv::MstOutput>("mst_service");
 
  // Make the request
  auto request = std::make_shared<mst_msgs::srv::MstOutput::Request>();
 // request->input = argv[1];
 getline(cin, request->input);
 
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
    RCLCPP_INFO(rclcpp::get_logger("Response: The MST for the given graph is: "), result.get()->output);
   
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("Response: "), "Failed to call mst_service");
  }
 
  rclcpp::shutdown();
  return 0;
}
