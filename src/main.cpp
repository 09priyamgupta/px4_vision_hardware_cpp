#include <rclcpp/rclcpp.hpp>
#include "px4_vision_hardware_cpp/vision_landing_mode.hpp"
#include "px4_vision_hardware_cpp/mission_executor.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    // Create the shared ROS 2 node
    auto node = std::make_shared<rclcpp::Node>("vision_landing_control_node");

    // 1. Instantiate the "Muscle" (The custom flight mode)
    VisionLandingMode vision_mode(*node);

    // 2. Instantiate the "Brain" (The state machine, passing it the mode)
    MissionExecutor executor(vision_mode);

    // 3. Register the executor with the PX4 flight controller
    if (!executor.doRegister()) {
        RCLCPP_FATAL(node->get_logger(), "Failed to register Vision Landing Mode with PX4!");
        return -1;
    }

    // Keep the node alive
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}