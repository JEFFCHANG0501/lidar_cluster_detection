#include "lidar_euclidean_clustering/euclidean_clustering.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EuclideanClustering>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}