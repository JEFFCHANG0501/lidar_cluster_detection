#include "pcd_publisher/pcd_publisher.hpp"

namespace fs = boost::filesystem;
using namespace std::chrono_literals;

PCDPublisher::PCDPublisher()
: Node("pcd_publisher"), count_(0)
{
    RCLCPP_INFO(this->get_logger(), "Start publish point cloud.");

    folder_path = "/home/jeff/lidar_cluster_detection/pcd_publisher/data/";
    getFileList(folder_path, files);
    
    pcd_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/vls128_0/velodyne_points", 10);
    timer = create_wall_timer(100ms, std::bind(&PCDPublisher::timer_callback, this));
}

void PCDPublisher::getFileList(const std::string& dir_path, std::vector<std::string>& files)
{
    if (!fs::is_directory(dir_path))
    {
        std::cout << "Error: " << dir_path << " is not a valid directory." << std::endl;
        return;
    }

    for (const auto& entry : fs::directory_iterator(dir_path))
    {
        const auto& file_path = entry.path();
        if (fs::is_regular_file(file_path))
        {
            if (file_path.extension() == ".pcd")
                files.push_back(file_path.stem().string());
        }
    }

    std::sort(files.begin(), files.end());
}

void PCDPublisher::timer_callback()
{
    if (count_ < files.size())
    {
        std::string full_file_path = folder_path + files[count_] + ".pcd";
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(full_file_path, *cloud) == -1)
            RCLCPP_ERROR(this->get_logger(), "Error loading PCD file: %s", full_file_path.c_str());
        else
        {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*cloud, msg);
            msg.header.stamp = get_clock()->now();
            msg.header.frame_id = "vls_128";
            pcd_pub->publish(msg);
        }
        count_++;

    }
    else
        count_=0;
}