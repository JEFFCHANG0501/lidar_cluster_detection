#include "pcd_publisher/pcd_publisher.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCDPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}