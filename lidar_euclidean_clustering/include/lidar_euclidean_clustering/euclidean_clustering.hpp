#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>


class EuclideanClustering : public rclcpp::Node 
{
public:
    EuclideanClustering();

private:
    static float leaf_size;
    Eigen::Vector4f ROI_MIN_POINT;
    Eigen::Vector4f ROI_MAX_POINT;
    Eigen::Vector4f EagleCar_MIN_POINT; 
    Eigen::Vector4f EagleCar_MAX_POINT;

    void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud, const float leaf_size);

    void publishMarkerArray(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr > > clusters);

    void publishEagleCar(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& publisher);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filter_cloud_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerArray_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr eaglecar_pub_;
};