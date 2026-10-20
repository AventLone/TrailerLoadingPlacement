#include "free_space_detection/nodes/FreeSpaceDetect.h"

int main(const int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const auto options = rclcpp::NodeOptions();
    const auto free_space_detect_node = std::make_shared<FreeSpaceDetect>("free_space_detection", options);
    rclcpp::spin(free_space_detect_node);
    rclcpp::shutdown();
    return 0;
}