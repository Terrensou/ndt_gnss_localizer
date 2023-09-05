#ifndef _MAP_LOADER_H_
#define _MAP_LOADER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>
#include <pcl_ros/transforms.h>
#include "gnssTools.hpp"

class MapLoader{
public:
    GNSSTools gnssTools;

    ros::Publisher pc_map_pub_;
    ros::Publisher lla_map_pub_;
    std::vector<std::string> file_list_;
    std::string map_lla_file;
    Eigen::Vector3d map_lla_vector;

    sensor_msgs::NavSatFix map_lla_msg;

    MapLoader(ros::NodeHandle &nh);

private:

    float tf_x_, tf_y_, tf_z_, tf_roll_, tf_pitch_, tf_yaw_; 

    void init_tf_params(ros::NodeHandle &nh);
    sensor_msgs::PointCloud2 CreatePcd();
    sensor_msgs::PointCloud2 TransformMap(sensor_msgs::PointCloud2 & in);
    void SaveMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_ptr);
}; //MapLoader

#endif