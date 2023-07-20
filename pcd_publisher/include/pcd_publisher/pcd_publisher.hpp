#include <boost/filesystem.hpp>
#include <csignal>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"


class PCDPublisher : public rclcpp::Node
{
public:
    explicit PCDPublisher();

private:
    std::string folder_path;
    std::vector<std::string> files;

    void getFileList(const std::string& dir_path, std::vector<std::string>& files);
    void timer_callback();

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub;
    rclcpp::TimerBase::SharedPtr timer;
    size_t count_;
};