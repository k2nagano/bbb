#include <iostream>

// int
// main(int argc, char* argv[])
// {
//     return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <memory>

void
on_toggle_play(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    response->success = true;
    response->message = "aaa";
    // request->a + request->b;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
    //             "Incoming request\na: %ld"
    //             " b: %ld",
    //             request->a, request->b);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]",
    //             (long int)response->sum);
}

int
main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("camera_streaming_node");

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service =
        node->create_service<std_srvs::srv::SetBool>("/toggle_play", &on_toggle_play);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to receive request.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}