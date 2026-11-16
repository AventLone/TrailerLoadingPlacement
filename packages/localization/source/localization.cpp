#include "localization/nodes/Localization.h"


int main(const int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Localization>("dynamic_localization"));
    rclcpp::shutdown();
    return 0;
}