#include "map_loader.h"


static void splitString(const std::string &s,
                        std::vector<std::string> &sv,
                        const char delim = ' ') {
    sv.clear();
    std::istringstream iss(s);
    std::string temp;

    while (std::getline(iss, temp, delim)) {
        sv.emplace_back(std::move(temp));
    }

    return;
}


MapLoader::MapLoader(ros::NodeHandle &nh){
    std::string pcd_file_path, map_topic, map_lla_topic;
    nh.param<std::string>("pcd_path", pcd_file_path, "");
    nh.param<std::string>("pcd_lla_path", map_lla_file, "");
    nh.param<std::string>("map_topic", map_topic, "/ndt_gnss_localizer/point_map");
    nh.param<std::string>("map_lla_topic", map_lla_topic, "/ndt_gnss_localizer/map_lla");

    init_tf_params(nh);

    pc_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 10, true);
    lla_map_pub_ = nh.advertise<sensor_msgs::NavSatFix>(map_lla_topic, 10, true);

    file_list_.push_back(pcd_file_path);

    auto pc_msg = CreatePcd();
    
    auto out_msg = TransformMap(pc_msg);

    if (out_msg.width != 0) {
		out_msg.header.frame_id = "map";
		pc_map_pub_.publish(out_msg);

        map_lla_msg.header.stamp = ros::Time::now();
        map_lla_msg.header.frame_id = "map";
        lla_map_pub_.publish(map_lla_msg);
	}

}

void MapLoader::init_tf_params(ros::NodeHandle &nh){
    nh.param<float>("x", tf_x_, 0.0);
    nh.param<float>("y", tf_y_, 0.0);
    nh.param<float>("z", tf_z_, 0.0);
    nh.param<float>("roll", tf_roll_, 0.0);
    nh.param<float>("pitch", tf_pitch_, 0.0);
    nh.param<float>("yaw", tf_yaw_, 0.0);
    ROS_INFO_STREAM("x" << tf_x_ <<"y: "<<tf_y_<<"z: "<<tf_z_<<"roll: "
                        <<tf_roll_<<" pitch: "<< tf_pitch_<<"yaw: "<<tf_yaw_);
}

sensor_msgs::PointCloud2 MapLoader::TransformMap(sensor_msgs::PointCloud2 & in){
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(in, *in_pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Translation3f tl_m2w(tf_x_, tf_y_, tf_z_);                 // tl: translation
    Eigen::AngleAxisf rot_x_m2w(tf_roll_, Eigen::Vector3f::UnitX());  // rot: rotation
    Eigen::AngleAxisf rot_y_m2w(tf_pitch_, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_m2w(tf_yaw_, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f tf_m2w = (tl_m2w * rot_z_m2w * rot_y_m2w * rot_x_m2w).matrix();

    pcl::transformPointCloud(*in_pc, *transformed_pc_ptr, tf_m2w);

    SaveMap(transformed_pc_ptr);

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*transformed_pc_ptr, output_msg);
    return output_msg;
}

void MapLoader::SaveMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_ptr){
    pcl::io::savePCDFile("/tmp/transformed_map.pcd", *map_pc_ptr);
}



sensor_msgs::PointCloud2 MapLoader::CreatePcd()
{
	sensor_msgs::PointCloud2 pcd, part;
	for (const std::string& path : file_list_) {
		// Following outputs are used for progress bar of Runtime Manager.
		if (pcd.width == 0) {
			if (pcl::io::loadPCDFile(path.c_str(), pcd) == -1) {
				std::cerr << "load failed " << path << std::endl;
			}
		} else {
			if (pcl::io::loadPCDFile(path.c_str(), part) == -1) {
				std::cerr << "load failed " << path << std::endl;
			}
			pcd.width += part.width;
			pcd.row_step += part.row_step;
			pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
		}
		std::cerr << "load map" << path << std::endl;
		if (!ros::ok()) break;
	}

    // get map LLA
    std::fstream originStream(map_lla_file, std::fstream::in);
    std::cerr << "load LLA" << map_lla_file << std::endl;
    if (originStream.is_open()) { //checking whether the file is open
        double longitude, latitude, altitude;
        char *ptr_t;
        std::string st;
        std::vector<std::string> sv;

        getline(originStream, st);
        splitString(st, sv);

        latitude = boost::numeric_cast<double>(strtof64(sv[0].c_str(), &ptr_t));
        longitude = boost::numeric_cast<double>(strtof64(sv[1].c_str(), &ptr_t));
        altitude = boost::numeric_cast<double>(strtof64(sv[2].c_str(), &ptr_t));

        map_lla_msg.longitude = longitude;
        map_lla_msg.latitude = latitude;
        map_lla_msg.altitude = altitude;

        geographic_msgs::GeoPoint pLLA;
        pLLA.latitude = latitude;
        pLLA.longitude = longitude;
        pLLA.altitude = altitude;
        geodesy::UTMPoint pUTM;
        geodesy::fromMsg(pLLA, pUTM);

        // publish
        geometry_msgs::PoseStamped map_pose_stamped_msg;
        map_pose_stamped_msg.header.stamp = ros::Time::now();
        map_pose_stamped_msg.header.frame_id = map_frame_;
        map_pose_stamped_msg.pose.position.x = pUTM.easting;
        map_pose_stamped_msg.pose.position.y = pUTM.northing;
        map_pose_stamped_msg.pose.position.z = pUTM.altitude;

        map_pose_stamped_msg.pose.orientation.x = 0;
        map_pose_stamped_msg.pose.orientation.y = 0;
        map_pose_stamped_msg.pose.orientation.z = 0;
        map_pose_stamped_msg.pose.orientation.w = 1;

//        std::cout << "lat: " << latitude << "  ,lon: " << longitude << "   ; x: " << pUTM.easting << " , y: " << pUTM.northing << std::endl;


        map_frame_ = "map";
        world_frame_ = "world";
        publish_map_tf(world_frame_, map_frame_, map_pose_stamped_msg);
//        publish_tf(map_frame_, world_frame_, map_pose_stamped_msg);

    } else {
        ROS_ERROR("Can not get global map LLA.");
    }

	return pcd;
}

void MapLoader::publish_map_tf(
        const std::string &frame_id, const std::string &child_frame_id,
        const geometry_msgs::PoseStamped &pose_msg) {
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;
    transform_stamped.header.stamp = pose_msg.header.stamp;

    transform_stamped.transform.translation.x = pose_msg.pose.position.x;
    transform_stamped.transform.translation.y = pose_msg.pose.position.y;
    transform_stamped.transform.translation.z = pose_msg.pose.position.z;

    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
    transform_stamped.transform.rotation.x = tf_quaternion.x();
    transform_stamped.transform.rotation.y = tf_quaternion.y();
    transform_stamped.transform.rotation.z = tf_quaternion.z();
    transform_stamped.transform.rotation.w = tf_quaternion.w();

    tf2_broadcaster_.sendTransform(transform_stamped);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_loader");

    ROS_INFO("\033[1;32m---->\033[0m Map Loader Started.");

    ros::NodeHandle nh("~");

    MapLoader map_loader(nh);

    ros::spin();

    return 0;
}
