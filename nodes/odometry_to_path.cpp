#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geodesy/utm.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xmlmemory.h>
#include <libxml/xmlstring.h>
#include <libxml/xpath.h>

#include <chrono>
#include <boost/filesystem.hpp>

#include "gnssTools.hpp"

class Path_Generator {

public:

    Path_Generator(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : nh_(nh), private_nh_(private_nh),
                                                                       tf2_listener_(tf2_buffer_) {

        private_nh_.getParam("save_path_dir", save_path_dir_);
        private_nh_.getParam("kml_config_file", kml_config_file_);
        private_nh_.getParam("map_frame", map_frame_);
        private_nh_.getParam("world_frame", world_frame_);
        map_lla_sub_ = nh_.subscribe("/ndt_localizer/map_lla", 1, &Path_Generator::callback_maplla, this,
                                     ros::TransportHints().tcpNoDelay());
        odom_efk_sub_ = nh_.subscribe("/ndt/ekf/global_odom", 1, &Path_Generator::generatePathFromEKFOdometry, this,
                                      ros::TransportHints().tcpNoDelay());
        odom_ndt_sub_ = nh_.subscribe("/ndt/Odom", 1, &Path_Generator::generatePathFromNDTOdometry, this,
                                      ros::TransportHints().tcpNoDelay());
        odom_gnss_sub_ = nh_.subscribe("/integrated_nav/Odom", 1, &Path_Generator::generatePathFromGNSSOdometry, this,
                                       ros::TransportHints().tcpNoDelay());
        odom2_gnss_sub_ = nh_.subscribe("/integrated_nav/Odom_2", 1, &Path_Generator::generatePathFromGNSSOdometry2, this,
                                       ros::TransportHints().tcpNoDelay());

        path_ekf_pub_ = nh_.advertise<nav_msgs::Path>("/ndt/ekf/path", 10, true);
        path_ndt_pub_ = nh_.advertise<nav_msgs::Path>("/ndt/path", 10, true);
        path_gnss_pub_ = nh_.advertise<nav_msgs::Path>("/gnss/path", 10, true);
        path2_gnss_pub_ = nh_.advertise<nav_msgs::Path>("/gnss/path2", 10, true);

        save_path_srv_ = nh.advertiseService("ndt_localizer/save_path",
                                             &Path_Generator::save_path, this);

        odom_ekf_path.header.frame_id = world_frame_;
        odom_ndt_path.header.frame_id = world_frame_;
        odom_gnss_path.header.frame_id = world_frame_;
        odom2_gnss_path.header.frame_id = world_frame_;

        readKMLParameter();
    }

    void callback_maplla(const sensor_msgs::NavSatFix::ConstPtr &navsat_msg_ptr) {
        Eigen::Vector3d map_lla_vector = Eigen::Vector3d(navsat_msg_ptr->latitude, navsat_msg_ptr->longitude,
                                                         navsat_msg_ptr->altitude);

        gnssTools.lla_origin_ = map_lla_vector;

        geographic_msgs::GeoPoint pLLA;
        pLLA.latitude = map_lla_vector(0);
        pLLA.longitude = map_lla_vector(1);
        pLLA.altitude = map_lla_vector(2);
        geodesy::UTMPoint pUTM;
        geodesy::fromMsg(pLLA, pUTM);
        band = pUTM.band;
        zone = pUTM.zone;

//        std::cout << "map :lat: " << pLLA.latitude << "  ,long: " << pLLA.longitude << "  ,alt: " << pLLA.altitude << std::endl;

        init_flag = true;
    }


    void generatePathFromEKFOdometry(const nav_msgs::Odometry::ConstPtr &msg_in) {
        if (!init_flag) {
            return;
        }

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = msg_in->header;
        pose_stamped.header.frame_id = msg_in->child_frame_id;

        pose_stamped.pose.orientation = msg_in->pose.pose.orientation;

        geodesy::UTMPoint pUTM;
        pUTM.easting = msg_in->pose.pose.position.x;
        pUTM.northing = msg_in->pose.pose.position.y;
        pUTM.altitude = msg_in->pose.pose.position.z;
        pUTM.zone = zone;
        pUTM.band = band;
        auto pLLA = geodesy::toMsg(pUTM);

        Eigen::Vector3d lla;
        lla.setIdentity();
        lla = Eigen::Vector3d(pLLA.latitude,
                              pLLA.longitude,
                              pLLA.altitude);
        Eigen::Vector3d ecef = gnssTools.LLA2ECEF(lla);
        Eigen::Vector3d enu = gnssTools.ECEF2ENU(ecef);
        pose_stamped.pose.position.x = enu(0);
        pose_stamped.pose.position.y = enu(1);
        pose_stamped.pose.position.z = enu(2);

        ekf_lla_vec.push_back(lla);

        odom_ekf_path.poses.push_back(pose_stamped);

        path_ekf_pub_.publish(odom_ekf_path);

    }

    void generatePathFromNDTOdometry(const nav_msgs::Odometry::ConstPtr &msg_in) {
        if (!init_flag) {
            return;
        }

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = msg_in->header;
        pose_stamped.header.frame_id = msg_in->child_frame_id;

        pose_stamped.pose.orientation = msg_in->pose.pose.orientation;

        geodesy::UTMPoint pUTM;
        pUTM.easting = msg_in->pose.pose.position.x;
        pUTM.northing = msg_in->pose.pose.position.y;
        pUTM.altitude = msg_in->pose.pose.position.z;
        pUTM.zone = zone;
        pUTM.band = band;
        auto pLLA = geodesy::toMsg(pUTM);

        Eigen::Vector3d lla;
        lla.setIdentity();
        lla = Eigen::Vector3d(pLLA.latitude,
                              pLLA.longitude,
                              pLLA.altitude);
        Eigen::Vector3d ecef = gnssTools.LLA2ECEF(lla);
        Eigen::Vector3d enu = gnssTools.ECEF2ENU(ecef);
        pose_stamped.pose.position.x = enu(0);
        pose_stamped.pose.position.y = enu(1);
        pose_stamped.pose.position.z = enu(2);

        ndt_lla_vec.push_back(lla);

        odom_ndt_path.poses.push_back(pose_stamped);

        path_ndt_pub_.publish(odom_ndt_path);

    }

    void generatePathFromGNSSOdometry(const nav_msgs::Odometry::ConstPtr &msg_in) {
        if (!init_flag) {
            return;
        }

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = msg_in->header;
        pose_stamped.header.frame_id = msg_in->child_frame_id;

        pose_stamped.pose.orientation = msg_in->pose.pose.orientation;

        geodesy::UTMPoint pUTM;
        pUTM.easting = msg_in->pose.pose.position.x;
        pUTM.northing = msg_in->pose.pose.position.y;
        pUTM.altitude = msg_in->pose.pose.position.z;
        pUTM.zone = zone;
        pUTM.band = band;
        auto pLLA = geodesy::toMsg(pUTM);
//        std::cout << pUTM << std::endl;
        std::cout << "lat: " << pLLA.latitude << "  ,long: " << pLLA.longitude << "  ,alt: " << pLLA.altitude << std::endl;
        Eigen::Vector3d lla;
        lla.setIdentity();
        lla = Eigen::Vector3d(pLLA.latitude,
                              pLLA.longitude,
                              pLLA.altitude);
        Eigen::Vector3d ecef = gnssTools.LLA2ECEF(lla);
        Eigen::Vector3d enu = gnssTools.ECEF2ENU(ecef);
        pose_stamped.pose.position.x = enu(0);
        pose_stamped.pose.position.y = enu(1);
        pose_stamped.pose.position.z = enu(2);
        std::cout << "east: " << enu(0) << "  ,north: " << enu(1) << "  ,alt: " << enu(2) << std::endl;
        gnss_lla_vec.push_back(lla);

        odom_gnss_path.poses.push_back(pose_stamped);

        path_gnss_pub_.publish(odom_gnss_path);

    }

    void generatePathFromGNSSOdometry2(const nav_msgs::Odometry::ConstPtr &msg_in) {
        if (!init_flag) {
            return;
        }

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = msg_in->header;
        pose_stamped.header.frame_id =  msg_in->child_frame_id;

        pose_stamped.pose.orientation = msg_in->pose.pose.orientation;

        geodesy::UTMPoint pUTM;
        pUTM.easting = msg_in->pose.pose.position.x;
        pUTM.northing = msg_in->pose.pose.position.y;
        pUTM.altitude = msg_in->pose.pose.position.z;
        pUTM.zone = zone;
        pUTM.band = band;
        auto pLLA = geodesy::toMsg(pUTM);

        Eigen::Vector3d lla;
        lla.setIdentity();
        lla = Eigen::Vector3d(pLLA.latitude,
                              pLLA.longitude,
                              pLLA.altitude);
        Eigen::Vector3d ecef = gnssTools.LLA2ECEF(lla);
        Eigen::Vector3d enu = gnssTools.ECEF2ENU(ecef);
        pose_stamped.pose.position.x = enu(0);
        pose_stamped.pose.position.y = enu(1);
        pose_stamped.pose.position.z = enu(2);

        gnss2_lla_vec.push_back(lla);

        odom2_gnss_path.poses.push_back(pose_stamped);

        path2_gnss_pub_.publish(odom2_gnss_path);

    }


    bool get_transform(
            const std::string &target_frame, const std::string &source_frame,
            const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr) {
        if (target_frame == source_frame) {
            transform_stamped_ptr->header.stamp = ros::Time::now();
            transform_stamped_ptr->header.frame_id = target_frame;
            transform_stamped_ptr->child_frame_id = source_frame;
            transform_stamped_ptr->transform.translation.x = 0.0;
            transform_stamped_ptr->transform.translation.y = 0.0;
            transform_stamped_ptr->transform.translation.z = 0.0;
            transform_stamped_ptr->transform.rotation.x = 0.0;
            transform_stamped_ptr->transform.rotation.y = 0.0;
            transform_stamped_ptr->transform.rotation.z = 0.0;
            transform_stamped_ptr->transform.rotation.w = 1.0;
            return true;
        }

        try {
            *transform_stamped_ptr =
                    tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

            transform_stamped_ptr->header.stamp = ros::Time::now();
            transform_stamped_ptr->header.frame_id = target_frame;
            transform_stamped_ptr->child_frame_id = source_frame;
            transform_stamped_ptr->transform.translation.x = 0.0;
            transform_stamped_ptr->transform.translation.y = 0.0;
            transform_stamped_ptr->transform.translation.z = 0.0;
            transform_stamped_ptr->transform.rotation.x = 0.0;
            transform_stamped_ptr->transform.rotation.y = 0.0;
            transform_stamped_ptr->transform.rotation.z = 0.0;
            transform_stamped_ptr->transform.rotation.w = 1.0;
            return false;
        }
        return true;
    }

    bool save_path(std_srvs::Empty::Request &req,
                   std_srvs::Empty::Response &res) {

        auto save_dir = boost::filesystem::path(save_path_dir_);
        if (!boost::filesystem::exists(save_dir)) {
            boost::filesystem::create_directories(save_dir);
        }

        auto time_now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        struct tm *ptm = localtime(&time_now);

//    auto time_now = std::to_string(ros::Time::now().toSec());
        char date[60] = {0};
        sprintf(date, "%d-%02d-%02d-%02d.%02d.%02d", (int) ptm->tm_year + 1900, (int) ptm->tm_mon + 1,
                (int) ptm->tm_mday, (int) ptm->tm_hour, (int) ptm->tm_min, (int) ptm->tm_sec);
        auto time_now_str = std::string(date);

        auto file_dir = save_dir.c_str() + time_now_str + "/";
        boost::filesystem::create_directory(file_dir);

        save_path2tum(file_dir);
        save_path2kml(file_dir);
    }

    bool save_path2geojson(const std::string save_dir_str) {
        auto save_dir = boost::filesystem::path(save_dir_str);
        if (!boost::filesystem::exists(save_dir)) {
            boost::filesystem::create_directories(save_dir);
        }

        std::ofstream fout;

        fout.open(save_dir_str + "ndt_tum.txt");
        if (odom_ndt_path.poses.size() > 0 && fout.is_open()) {
            fout.precision(15);
            for (auto i = odom_ndt_path.poses.cbegin(); i != odom_ndt_path.poses.cend(); ++i) {
                fout << i->header.stamp.toSec() << " ";
                auto x = i->pose.position.x;
                auto y = i->pose.position.y;
                auto z = i->pose.position.z;
                auto qx = i->pose.orientation.x;
                auto qy = i->pose.orientation.y;
                auto qz = i->pose.orientation.z;
                auto qw = i->pose.orientation.w;
                fout << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
            }
            fout.close();
        } else {
            ROS_WARN("Lidar Path is Empty!");
            return false;
        }

        fout.open(save_dir_str + "gnss_enu_tum.txt");
        if (odom_gnss_path.poses.size() > 0 && fout.is_open()) {
            fout.precision(15);
            for (auto i = odom_gnss_path.poses.cbegin(); i != odom_gnss_path.poses.cend(); ++i) {
                fout << i->header.stamp.toSec() << " ";
                auto x = i->pose.position.x;
                auto y = i->pose.position.y;
                auto z = i->pose.position.z;
                auto qx = i->pose.orientation.x;
                auto qy = i->pose.orientation.y;
                auto qz = i->pose.orientation.z;
                auto qw = i->pose.orientation.w;
                fout << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
            }
            fout.close();
        } else {
            ROS_WARN("GNSS Path is Empty!");
        }

        ROS_INFO("Save Lidar Localization Path Successful.");
        return true;
    }

    bool save_path2tum(const std::string save_dir_str) {
        auto save_dir = boost::filesystem::path(save_dir_str);
        if (!boost::filesystem::exists(save_dir)) {
            boost::filesystem::create_directories(save_dir);
        }

        std::ofstream fout;

        fout.open(save_dir_str + "ndt_tum.txt");
        if (odom_ndt_path.poses.size() > 0 && fout.is_open()) {
            fout.precision(15);
            for (auto i = odom_ndt_path.poses.cbegin(); i != odom_ndt_path.poses.cend(); ++i) {
                fout << i->header.stamp.toSec() << " ";
                auto x = i->pose.position.x;
                auto y = i->pose.position.y;
                auto z = i->pose.position.z;
                auto qx = i->pose.orientation.x;
                auto qy = i->pose.orientation.y;
                auto qz = i->pose.orientation.z;
                auto qw = i->pose.orientation.w;
                fout << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
            }
            fout.close();
        } else {
            ROS_WARN("Lidar Path is Empty!");
        }

        fout.open(save_dir_str + "gnss_enu_tum.txt");
        if (odom_gnss_path.poses.size() > 0 && fout.is_open()) {
            fout.precision(15);
            for (auto i = odom_gnss_path.poses.cbegin(); i != odom_gnss_path.poses.cend(); ++i) {
                fout << i->header.stamp.toSec() << " ";
                auto x = i->pose.position.x;
                auto y = i->pose.position.y;
                auto z = i->pose.position.z;
                auto qx = i->pose.orientation.x;
                auto qy = i->pose.orientation.y;
                auto qz = i->pose.orientation.z;
                auto qw = i->pose.orientation.w;
                fout << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
            }
            fout.close();
        } else {
            ROS_WARN("GNSS Path is Empty!");
        }

        fout.open(save_dir_str + "gnss2_enu_tum.txt");
        if (odom2_gnss_path.poses.size() > 0 && fout.is_open()) {
            fout.precision(15);
            for (auto i = odom2_gnss_path.poses.cbegin(); i != odom2_gnss_path.poses.cend(); ++i) {
                fout << i->header.stamp.toSec() << " ";
                auto x = i->pose.position.x;
                auto y = i->pose.position.y;
                auto z = i->pose.position.z;
                auto qx = i->pose.orientation.x;
                auto qy = i->pose.orientation.y;
                auto qz = i->pose.orientation.z;
                auto qw = i->pose.orientation.w;
                fout << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
            }
            fout.close();
        } else {
            ROS_WARN("GNSS2 Path is Empty!");
        }

        fout.open(save_dir_str + "ekf_enu_tum.txt");
        if (odom_ekf_path.poses.size() > 0 && fout.is_open()) {
            fout.precision(15);
            for (auto i = odom_ekf_path.poses.cbegin(); i != odom_ekf_path.poses.cend(); ++i) {
                fout << i->header.stamp.toSec() << " ";
                auto x = i->pose.position.x;
                auto y = i->pose.position.y;
                auto z = i->pose.position.z;
                auto qx = i->pose.orientation.x;
                auto qy = i->pose.orientation.y;
                auto qz = i->pose.orientation.z;
                auto qw = i->pose.orientation.w;
                fout << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
            }
            fout.close();
        } else {
            ROS_WARN("EKF Filter Path is Empty!");
        }


//        ROS_INFO("Save Localization Path Successful.");
        return true;
    }

    bool save_path2kml(const std::string save_dir_str) {

        auto save_dir = boost::filesystem::path(save_dir_str);
        if (!boost::filesystem::exists(save_dir)) {
            boost::filesystem::create_directories(save_dir);
        }

        std::ofstream fout;
        fout.open(save_dir_str + "ndt_trajectry.kml");
        if (ndt_lla_vec.size() > 0 && fout.is_open()) {

            fout.precision(15);
            int index = 0;
            fout << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl;
            fout << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">" << std::endl;
            fout << "<Document>" << std::endl;
            fout << "<name>"
                 << "Lidar Trajectory"
                 << "</name>" << std::endl;
            fout << "<description>"
                 << "Lidar NDT Trajectory in base_link"
                 << "</description>" << std::endl;

            fout << "<Style id=\"" << kml_config_parameter[index++] << "\">" << std::endl;
            fout << "<LineStyle>" << std::endl;
            fout << "<color>"
                 << "7fFF1900"
                 << "</color>" << std::endl;
            fout << "<width>" << kml_config_parameter[index++] << "</width>" << std::endl;
            fout << "</LineStyle>" << std::endl;
            fout << "<PolyStyle>" << std::endl;
            fout << "<color>"
                 << "7fFF1900"
                 << "</color>" << std::endl;
            fout << "</PolyStyle>" << std::endl;
            fout << "</Style>" << std::endl;
            fout << "<Placemark>" << std::endl;
            fout << "<styleUrl>" << kml_config_parameter[index++] << "</styleUrl>" << std::endl;
            fout << "<LineString>" << std::endl;
            fout << "<extrude>" << kml_config_parameter[index++] << "</extrude>" << std::endl;
            fout << "<tessellate>" << kml_config_parameter[index++] << "</tessellate>"
                 << std::endl;
            fout << "<altitudeMode>" << kml_config_parameter[index++] << "</altitudeMode>"
                 << std::endl;
            fout << "<coordinates>" << std::endl;

            for (int i = 0; i < ndt_lla_vec.size(); i++) {
                fout << ndt_lla_vec.at(i)[1] << ',' << ndt_lla_vec.at(i)[0] << ','
                     << ndt_lla_vec.at(i)[2] << std::endl;
            }

            fout << "</coordinates>" << std::endl;
            fout << "</LineString></Placemark>" << std::endl;
            fout << "</Document></kml>" << std::endl;
        }
        fout.close();

        fout.open(save_dir_str + "gnss_trajectry.kml");
        if (gnss_lla_vec.size() > 0 && fout.is_open()) {

            fout.precision(15);
            int index = 0;
            fout << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl;
            fout << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">" << std::endl;
            fout << "<Document>" << std::endl;
            fout << "<name>"
                 << "GNSS Trajectory"
                 << "</name>" << std::endl;
            fout << "<description>"
                 << "GNSS Trajectory in base_link"
                 << "</description>" << std::endl;

            fout << "<Style id=\"" << kml_config_parameter[index++] << "\">" << std::endl;
            fout << "<LineStyle>" << std::endl;
            fout << "<color>"
                 << "7f19FF00"
                 << "</color>" << std::endl;
            fout << "<width>" << kml_config_parameter[index++] << "</width>" << std::endl;
            fout << "</LineStyle>" << std::endl;
            fout << "<PolyStyle>" << std::endl;
            fout << "<color>"
                 << "7f19FF00"
                 << "</color>" << std::endl;
            fout << "</PolyStyle>" << std::endl;
            fout << "</Style>" << std::endl;
            fout << "<Placemark>" << std::endl;
            fout << "<styleUrl>" << kml_config_parameter[index++] << "</styleUrl>" << std::endl;
            fout << "<LineString>" << std::endl;
            fout << "<extrude>" << kml_config_parameter[index++] << "</extrude>" << std::endl;
            fout << "<tessellate>" << kml_config_parameter[index++] << "</tessellate>"
                 << std::endl;
            fout << "<altitudeMode>" << kml_config_parameter[index++] << "</altitudeMode>"
                 << std::endl;
            fout << "<coordinates>" << std::endl;

            for (int i = 0; i < gnss_lla_vec.size(); i++) {
                fout << gnss_lla_vec.at(i)[1] << ',' << gnss_lla_vec.at(i)[0] << ','
                     << gnss_lla_vec.at(i)[2] << std::endl;
            }

            fout << "</coordinates>" << std::endl;
            fout << "</LineString></Placemark>" << std::endl;
            fout << "</Document></kml>" << std::endl;
        }
        fout.close();

        fout.open(save_dir_str + "gnss2_trajectry.kml");
        if (gnss2_lla_vec.size() > 0 && fout.is_open()) {

            fout.precision(15);
            int index = 0;
            fout << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl;
            fout << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">" << std::endl;
            fout << "<Document>" << std::endl;
            fout << "<name>"
                 << "GNSS Trajectory"
                 << "</name>" << std::endl;
            fout << "<description>"
                 << "GNSS Trajectory in base_link"
                 << "</description>" << std::endl;

            fout << "<Style id=\"" << kml_config_parameter[index++] << "\">" << std::endl;
            fout << "<LineStyle>" << std::endl;
            fout << "<color>"
                 << "7f19FF00"
                 << "</color>" << std::endl;
            fout << "<width>" << kml_config_parameter[index++] << "</width>" << std::endl;
            fout << "</LineStyle>" << std::endl;
            fout << "<PolyStyle>" << std::endl;
            fout << "<color>"
                 << "7f19FF00"
                 << "</color>" << std::endl;
            fout << "</PolyStyle>" << std::endl;
            fout << "</Style>" << std::endl;
            fout << "<Placemark>" << std::endl;
            fout << "<styleUrl>" << kml_config_parameter[index++] << "</styleUrl>" << std::endl;
            fout << "<LineString>" << std::endl;
            fout << "<extrude>" << kml_config_parameter[index++] << "</extrude>" << std::endl;
            fout << "<tessellate>" << kml_config_parameter[index++] << "</tessellate>"
                 << std::endl;
            fout << "<altitudeMode>" << kml_config_parameter[index++] << "</altitudeMode>"
                 << std::endl;
            fout << "<coordinates>" << std::endl;

            for (int i = 0; i < gnss_lla_vec.size(); i++) {
                fout << gnss_lla_vec.at(i)[1] << ',' << gnss_lla_vec.at(i)[0] << ','
                     << gnss_lla_vec.at(i)[2] << std::endl;
            }

            fout << "</coordinates>" << std::endl;
            fout << "</LineString></Placemark>" << std::endl;
            fout << "</Document></kml>" << std::endl;
        }
        fout.close();

        fout.open(save_dir_str + "ekf_trajectry.kml");
        if (ekf_lla_vec.size() > 0 && fout.is_open()) {

            fout.precision(15);
            int index = 0;
            fout << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl;
            fout << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">" << std::endl;
            fout << "<Document>" << std::endl;
            fout << "<name>"
                 << "ekf filter Trajectory"
                 << "</name>" << std::endl;
            fout << "<description>"
                 << "ekf filter Trajectory in base_link"
                 << "</description>" << std::endl;

            fout << "<Style id=\"" << kml_config_parameter[index++] << "\">" << std::endl;
            fout << "<LineStyle>" << std::endl;
            fout << "<color>"
                 << "7f1900FF"
                 << "</color>" << std::endl;
            fout << "<width>" << kml_config_parameter[index++] << "</width>" << std::endl;
            fout << "</LineStyle>" << std::endl;
            fout << "<PolyStyle>" << std::endl;
            fout << "<color>"
                 << "7f19FF00"
                 << "</color>" << std::endl;
            fout << "</PolyStyle>" << std::endl;
            fout << "</Style>" << std::endl;
            fout << "<Placemark>" << std::endl;
            fout << "<styleUrl>" << kml_config_parameter[index++] << "</styleUrl>" << std::endl;
            fout << "<LineString>" << std::endl;
            fout << "<extrude>" << kml_config_parameter[index++] << "</extrude>" << std::endl;
            fout << "<tessellate>" << kml_config_parameter[index++] << "</tessellate>"
                 << std::endl;
            fout << "<altitudeMode>" << kml_config_parameter[index++] << "</altitudeMode>"
                 << std::endl;
            fout << "<coordinates>" << std::endl;

            for (int i = 0; i < ekf_lla_vec.size(); i++) {
                fout << ekf_lla_vec.at(i)[1] << ',' << ekf_lla_vec.at(i)[0] << ','
                     << ekf_lla_vec.at(i)[2] << std::endl;
            }

            fout << "</coordinates>" << std::endl;
            fout << "</LineString></Placemark>" << std::endl;
            fout << "</Document></kml>" << std::endl;
        }
        fout.close();
        return 0;
    }

    int readKMLParameter() {
        xmlDocPtr pDoc = xmlReadFile((kml_config_file_).c_str(),
                                     "UTF-8", XML_PARSE_RECOVER);
        if (NULL == pDoc) {
            std::cout << "open config.xml error\n" << std::endl;
            return 1;
        }

        xmlNodePtr pRoot = xmlDocGetRootElement(pDoc);
        if (NULL == pRoot) {
            std::cout << "get config.xml root error\n" << std::endl;
            return 1;
        }

        xmlNodePtr pFirst = pRoot->children;

        while (NULL != pFirst) {
            xmlChar *value = NULL;
            if (!xmlStrcmp(pFirst->name, (const xmlChar *) ("style"))) {
                xmlNodePtr pStyle = pFirst->children;
                while (NULL != pStyle) {
                    value = xmlNodeGetContent(pStyle);
                    if (xmlStrcmp(pStyle->name, (const xmlChar *) ("text"))) {
                        kml_config_parameter.push_back((char *) value);
                    }
                    pStyle = pStyle->next;
                }
            } else if (!xmlStrcmp(pFirst->name, (const xmlChar *) ("Placemark"))) {
                xmlNodePtr pPlacemark = pFirst->children;
                while (NULL != pPlacemark) {
                    value = xmlNodeGetContent(pPlacemark);
                    if (xmlStrcmp(pPlacemark->name, (const xmlChar *) ("text"))) {
                        kml_config_parameter.push_back((char *) value);
                    }
                    pPlacemark = pPlacemark->next;
                }
            } else {
                value = xmlNodeGetContent(pFirst);
                if (xmlStrcmp(pFirst->name, (const xmlChar *) ("text"))) {
                    kml_config_parameter.push_back((char *) value);
                }
            }
            pFirst = pFirst->next;
        }
//    ROS_WARN("KML_CONFIG: %d",kml_config_parameter.size());
        return 0;
    }

private:

    GNSSTools gnssTools;
    bool init_flag = false;
    char band;
    uint8_t zone;

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;

    std::string map_frame_ = "map";
    std::string world_frame_ = "world";
    std::string save_path_dir_;
    std::string kml_config_file_;

    ros::Publisher path_ekf_pub_;
    ros::Publisher path_ndt_pub_;
    ros::Publisher path_gnss_pub_;
    ros::Publisher path2_gnss_pub_;

    ros::Subscriber map_lla_sub_;
    ros::Subscriber odom_efk_sub_;
    ros::Subscriber odom_ndt_sub_;
    ros::Subscriber odom_gnss_sub_;
    ros::Subscriber odom2_gnss_sub_;

    ros::ServiceServer save_path_srv_;

    nav_msgs::Path odom_ekf_path;
    nav_msgs::Path odom_ndt_path;
    nav_msgs::Path odom_gnss_path;
    nav_msgs::Path odom2_gnss_path;

    std::vector<Eigen::Vector3d> gnss_lla_vec;
    std::vector<Eigen::Vector3d> gnss2_lla_vec;
    std::vector<Eigen::Vector3d> ndt_lla_vec;
    std::vector<Eigen::Vector3d> ekf_lla_vec;
    std::vector<std::string> kml_config_parameter;

    ros::NodeHandle &nh_, &private_nh_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_generator_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    Path_Generator generator(nh, private_nh);
    ros::spin();
    return 0;
}