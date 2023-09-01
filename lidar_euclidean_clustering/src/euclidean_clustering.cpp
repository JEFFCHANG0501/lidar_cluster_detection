#include "lidar_euclidean_clustering/euclidean_clustering.hpp"

float EuclideanClustering::leaf_size = 0.2;

EuclideanClustering::EuclideanClustering()
: Node("euclidean_clustering")
{
    RCLCPP_INFO(this->get_logger(), "start Euclidean Clustering.");

    ROI_MIN_POINT = Eigen::Vector4f(-10.0, -5.0, -2.0, 1.0);
    ROI_MAX_POINT = Eigen::Vector4f(10.0, 70.0, 2.0, 1.0);
    EagleCar_MIN_POINT = Eigen::Vector4f(-0.8, -1.0, -1.95, 1.0);
    EagleCar_MAX_POINT = Eigen::Vector4f(0.8, 4.0, 0, 1.0);

    pcd_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/vls128_0/velodyne_points", 10, std::bind(&EuclideanClustering::points_callback, this, std::placeholders::_1));

    filter_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/vls128_0/filter_cloud", 10);
    markerArray_pub = create_publisher<visualization_msgs::msg::MarkerArray>("/vls128_0/object/markers", 10);
    eaglecar_pub_ = create_publisher<visualization_msgs::msg::Marker>("/eaglecar", 10);

}

void EuclideanClustering::points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_pc);

    /*** PointCloud Downsampling ***/ 
    downsampleCloud(pcl_pc, cloud_filtered, leaf_size);


    /*** Cropping the ROI ***/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> region(true);
    region.setMin(ROI_MIN_POINT);
    region.setMax(ROI_MAX_POINT);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_roi);


    /*** Remove EagleCar Region ***/ 
    std::vector<int> indices;
    pcl::CropBox<pcl::PointXYZ> roof(true);
    roof.setMin(EagleCar_MIN_POINT);
    roof.setMax(EagleCar_MAX_POINT);
    roof.setInputCloud(cloud_roi);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (auto& point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_roi);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_roi);


    /*** Ground Remove ***/
    pcl::IndicesPtr pc_indices(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZ> pt;
    pt.setInputCloud(cloud_roi);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(-1.85, 0.5);
    pt.filter(*pc_indices);


    /*** Euclidean Clustering ***/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_roi, pc_indices);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.4);
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_roi);
    ec.setIndices(pc_indices);
    ec.extract(cluster_indices);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr > > clusters;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cluster->points.push_back(cloud_roi->points[*pit]);
        
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }


    /*** Publish Filtered point cloud ***/
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::msg::PointCloud2 ros_pc2_out;
    pcl::copyPointCloud(*cloud_roi, *pc_indices, *pcl_pc_out);
    pcl::toROSMsg(*pcl_pc_out, ros_pc2_out);
    filter_cloud_pub->publish(ros_pc2_out);

    /*** Publish Marker for EagleCar ***/
    publishEagleCar(eaglecar_pub_);

    /*** Publish MarkerArray for 3D BBox ***/
    publishMarkerArray(msg, clusters);

}

void EuclideanClustering::publishMarkerArray(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr > > clusters)
{
    visualization_msgs::msg::MarkerArray marker_array_msg;
    marker_array_msg.markers.clear();
    
    for (size_t i=0; i<clusters.size(); i++)
    {
        Eigen::Vector4f min, max;
        pcl::getMinMax3D(*clusters[i], min, max);
        
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "euclidean_clustering";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point p[24];
        p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
        p[1].x = min[0];  p[1].y = max[1];  p[1].z = max[2];
        p[2].x = max[0];  p[2].y = max[1];  p[2].z = max[2];
        p[3].x = max[0];  p[3].y = min[1];  p[3].z = max[2];
        p[4].x = max[0];  p[4].y = max[1];  p[4].z = max[2];
        p[5].x = max[0];  p[5].y = max[1];  p[5].z = min[2];
        p[6].x = min[0];  p[6].y = min[1];  p[6].z = min[2];
        p[7].x = max[0];  p[7].y = min[1];  p[7].z = min[2];
        p[8].x = min[0];  p[8].y = min[1];  p[8].z = min[2];
        p[9].x = min[0];  p[9].y = max[1];  p[9].z = min[2];
        p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
        p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
        p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
        p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
        p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
        p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
        p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
        p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
        p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
        p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
        p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
        p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
        p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
        p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
        for(int i = 0; i < 24; i++) {
            marker.points.push_back(p[i]);
        }
        
        marker.scale.x = 0.02;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.5;
        marker.lifetime = rclcpp::Duration(0.1);

        visualization_msgs::msg::Marker distance_marker;
        distance_marker.header = msg->header;
        distance_marker.ns = "distance";
        distance_marker.id = i;
        distance_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        distance_marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point distance_marker_pose;
        distance_marker_pose = marker.points.at(6);
        distance_marker_pose.x = (marker.points.at(6).x + marker.points.at(7).x)/2;
        distance_marker_pose.y -= 0.5;

        distance_marker.pose.position = distance_marker_pose;
        distance_marker.scale.z = 0.7;
        distance_marker.color.a = 1.0;
        distance_marker.color.r = 1.0;
        distance_marker.color.g = 1.0;
        distance_marker.color.b = 1.0;

        float eu_distance;
        if(marker.points.at(6).x > 0)
            eu_distance = sqrt(
                pow(marker.points.at(6).x, 2) + 
                pow(marker.points.at(6).y, 2) + 
                pow(marker.points.at(6).z, 2));
        else if(marker.points.at(6).x < 0 && marker.points.at(7).x > 0)
            eu_distance = sqrt(
                pow((marker.points.at(6).x + marker.points.at(7).x)/2, 2) + 
                pow(marker.points.at(6).y, 2) + 
                pow(marker.points.at(6).z, 2));
        else
            eu_distance = sqrt(
                pow(marker.points.at(7).x, 2) + 
                pow(marker.points.at(7).y, 2) + 
                pow(marker.points.at(7).z, 2));
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << eu_distance;
        std::string text_distance = ss.str();
        distance_marker.text = text_distance;

        marker_array_msg.markers.push_back(marker);
        marker_array_msg.markers.push_back(distance_marker);

    }
    markerArray_pub->publish(marker_array_msg);
}

void EuclideanClustering::publishEagleCar(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& publisher)
{
    auto marker_msg = visualization_msgs::msg::Marker();
    marker_msg.header.frame_id = "vls_128";
    marker_msg.type = visualization_msgs::msg::Marker::CUBE;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.pose.position.x = 0.0;
    marker_msg.pose.position.y = 1.0;
    marker_msg.pose.position.z = -0.975;
    marker_msg.scale.x = 1.783;
    marker_msg.scale.y = 4.395;
    marker_msg.scale.z = 1.95;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 0.2;
    marker_msg.color.b = 0.9;
    marker_msg.color.a = 1.0;
    marker_msg.color.a = 0.5;
    marker_msg.lifetime.sec = 0;

    publisher->publish(marker_msg);
}

void EuclideanClustering::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud, const float leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(in_cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*out_cloud);
}
