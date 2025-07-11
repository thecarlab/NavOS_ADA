#include <rclcpp/rclcpp.hpp>
#include "pure_pursuit_controller/pure_pursuit_controller.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<pure_pursuit_controller::PurePursuitController>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Pure Pursuit Controller...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
} 