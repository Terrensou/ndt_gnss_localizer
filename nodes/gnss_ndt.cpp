#include "ndt.h"
#include <boost/bind.hpp>


static std::string POINTS_TOPIC;
static std::string GNSS_TOPIC;
static std::string IMU_TOPIC;
bool delete_no_fixed = false;
bool publish_tf_flag = true;

NdtLocalizer::NdtLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : nh_(nh), private_nh_(private_nh),
                                                                               tf2_listener_(tf2_buffer_) {

    key_value_stdmap_["state"] = "Initializing";
    init_params();

    // Publishers
    sensor_aligned_pose_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points_aligned", 10);
    ndt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 10);
    exe_time_pub_ = nh_.advertise<std_msgs::Float32>("exe_time_ms", 10);
    transform_probability_pub_ = nh_.advertise<std_msgs::Float32>("transform_probability", 10);
    iteration_num_pub_ = nh_.advertise<std_msgs::Float32>("iteration_num", 10);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/ndt_gnss_localizer/Odom", 10);
    navsatfix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/ndt_gnss_localizer/Navsatfix", 10, true);
    gnss_initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10, true);

    // Subscribers
    initial_pose_sub_ = nh_.subscribe("initialpose", 100, &NdtLocalizer::callback_init_pose, this,
                                      ros::TransportHints().tcpNoDelay());
    map_points_sub_ = nh_.subscribe("/ndt_localizer/points_map", 1, &NdtLocalizer::callback_pointsmap, this,
                                    ros::TransportHints().tcpNoDelay());
    map_lla_sub_ = nh_.subscribe("/ndt_localizer/map_lla", 1, &NdtLocalizer::callback_maplla, this,
                                 ros::TransportHints().tcpNoDelay());
    sensor_points_sub_ = nh_.subscribe("filtered_points", 1, &NdtLocalizer::callback_pointcloud, this,
                                       ros::TransportHints().tcpNoDelay());

    // Services
    reset_gnss_init_pose_srv_ = nh.advertiseService("ndt_localizer/reset_gnss_init",
                                                    &NdtLocalizer::reset_gnss_init_pose, this);

    imu_sub_.subscribe(nh, IMU_TOPIC, 100, ros::TransportHints().tcpNoDelay());
    gnss_sub_.subscribe(nh, GNSS_TOPIC, 100, ros::TransportHints().tcpNoDelay());
    cloud_sub_.subscribe(nh, POINTS_TOPIC, 100, ros::TransportHints().tcpNoDelay());
    navsatfix_imu_cloud_sync.reset(
            new NAVSATFIX_IMU_Cloud_Sync(NAVSATFIX_IMU_Cloud_Policy(100), gnss_sub_, imu_sub_, cloud_sub_));
    navsatfix_imu_cloud_sync->registerCallback(boost::bind(&NdtLocalizer::callback_gnss_init_pose, this, _1, _2, _3));

    diagnostic_thread_ = std::thread(&NdtLocalizer::timer_diagnostic, this);
    diagnostic_thread_.detach();

}

NdtLocalizer::~NdtLocalizer() {}

void NdtLocalizer::timer_diagnostic() {
    ros::Rate rate(100);
    while (ros::ok()) {
        diagnostic_msgs::DiagnosticStatus diag_status_msg;
        diag_status_msg.name = "ndt_scan_matcher";
        diag_status_msg.hardware_id = "";

        for (const auto &key_value: key_value_stdmap_) {
            diagnostic_msgs::KeyValue key_value_msg;
            key_value_msg.key = key_value.first;
            key_value_msg.value = key_value.second;
            diag_status_msg.values.push_back(key_value_msg);
        }

        diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
        diag_status_msg.message = "";
        if (key_value_stdmap_.count("state") && key_value_stdmap_["state"] == "Initializing") {
            diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
            diag_status_msg.message += "Initializing State. ";
        }
        if (
                key_value_stdmap_.count("skipping_publish_num") &&
                std::stoi(key_value_stdmap_["skipping_publish_num"]) > 1) {
            diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
            diag_status_msg.message += "skipping_publish_num > 1. ";
        }
        if (
                key_value_stdmap_.count("skipping_publish_num") &&
                std::stoi(key_value_stdmap_["skipping_publish_num"]) >= 5) {
            diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
            diag_status_msg.message += "skipping_publish_num exceed limit. ";
        }

        diagnostic_msgs::DiagnosticArray diag_msg;
        diag_msg.header.stamp = ros::Time::now();
        diag_msg.status.push_back(diag_status_msg);

        diagnostics_pub_.publish(diag_msg);

        rate.sleep();
    }
}

bool NdtLocalizer::reset_gnss_init_pose(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res) {
    gnss_pose = false;
    ROS_INFO("Reset GNSS init pose, wait for Navsatfix message.");
    return true;
}

void NdtLocalizer::callback_gnss_init_pose(
        const sensor_msgs::NavSatFix::ConstPtr &msgNavsatFix_in, const sensor_msgs::Imu::ConstPtr &msgIMU_in,
        const sensor_msgs::PointCloud2::ConstPtr &msgPoints_in) {
    if (!gnss_pose) {

        geometry_msgs::PoseWithCovarianceStamped::Ptr mapTF_imu_pose_msg_ptr(
                new geometry_msgs::PoseWithCovarianceStamped);
        mapTF_imu_pose_msg_ptr->header.stamp = msgIMU_in->header.stamp;
        mapTF_imu_pose_msg_ptr->header.frame_id = msgIMU_in->header.frame_id;

        Eigen::Vector3d lla;
        lla.setIdentity();
        lla = Eigen::Vector3d(msgNavsatFix_in->latitude,
                              msgNavsatFix_in->longitude,
                              msgNavsatFix_in->altitude);
        Eigen::Vector3d ecef = gnssTools.LLA2ECEF(lla);
        Eigen::Vector3d enu = gnssTools.ECEF2ENU(ecef);

        mapTF_imu_pose_msg_ptr->pose.pose.position.x = enu(0);
        mapTF_imu_pose_msg_ptr->pose.pose.position.y = enu(1);
        mapTF_imu_pose_msg_ptr->pose.pose.position.z = 0;
        ROS_WARN("Lidar Odometer GNSS init Point: lat:%f, lon:%f, alt:%f", lla(0), lla(1), lla(2));

        mapTF_imu_pose_msg_ptr->pose.pose.orientation = msgIMU_in->orientation;
        initial_pose_cov_msg_.pose.pose.position = mapTF_imu_pose_msg_ptr->pose.pose.position;

        double roll, pitch, yaw;
        tf::Quaternion origin_orientation;
        tf::quaternionMsgToTF(mapTF_imu_pose_msg_ptr->pose.pose.orientation, origin_orientation);
        tf::Matrix3x3(origin_orientation).getRPY(roll, pitch, yaw);

        auto filted_orientation = tf::createQuaternionFromRPY(0, 0, yaw);
//        auto filted_orientation = tf::createQuaternionFromRPY(0, 0, yaw);
        tf::quaternionTFToMsg(filted_orientation, initial_pose_cov_msg_.pose.pose.orientation);
        tf::Matrix3x3(filted_orientation).getRPY(roll, pitch, yaw);
        std::cout << "GNSS init east:" << initial_pose_cov_msg_.pose.pose.position.x << ", north:"
                  << initial_pose_cov_msg_.pose.pose.position.y << ", up:" << initial_pose_cov_msg_.pose.pose.position.z
                  << ", roll:" << roll << ", pitch:" << pitch << ", yaw:" << yaw << std::endl;


        gnss_pose = true;

        geometry_msgs::PoseWithCovarianceStamped gnss_initial_pose_msg;
        gnss_initial_pose_msg.header.frame_id = map_frame_;
        gnss_initial_pose_msg.header.stamp = msgNavsatFix_in->header.stamp;
        gnss_initial_pose_msg.pose = initial_pose_cov_msg_.pose;

        gnss_initial_pose_pub_.publish(gnss_initial_pose_msg);

    }
}

void NdtLocalizer::callback_init_pose(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial_pose_msg_ptr) {
    if (initial_pose_msg_ptr->header.frame_id == map_frame_) {
        initial_pose_cov_msg_ = *initial_pose_msg_ptr;
    } else {
        // get TF from pose_frame to map_frame
        geometry_msgs::TransformStamped::Ptr TF_pose_to_map_ptr(new geometry_msgs::TransformStamped);
        get_transform(map_frame_, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

        // transform pose_frame to map_frame
        geometry_msgs::PoseWithCovarianceStamped::Ptr mapTF_initial_pose_msg_ptr(
                new geometry_msgs::PoseWithCovarianceStamped);
        tf2::doTransform(*initial_pose_msg_ptr, *mapTF_initial_pose_msg_ptr, *TF_pose_to_map_ptr);
        // mapTF_initial_pose_msg_ptr->header.stamp = initial_pose_msg_ptr->header.stamp;
        initial_pose_cov_msg_ = *mapTF_initial_pose_msg_ptr;
    }
    // if click the initpose again, re init！
    init_pose = false;

    double roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(initial_pose_cov_msg_.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    std::cout << "Manual init x:" << initial_pose_cov_msg_.pose.pose.position.x << ", y:"
              << initial_pose_cov_msg_.pose.pose.position.y << ", z:" << initial_pose_cov_msg_.pose.pose.position.z
              << ", roll:" << roll << ", pitch:" << pitch << ", yaw:" << yaw << std::endl;
}

void NdtLocalizer::callback_pointsmap(
        const sensor_msgs::PointCloud2::ConstPtr &map_points_msg_ptr) {
    const auto trans_epsilon = ndt_.getTransformationEpsilon();
    const auto step_size = ndt_.getStepSize();
    const auto resolution = ndt_.getResolution();
    const auto max_iterations = ndt_.getMaximumIterations();

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_new;

    ndt_new.setTransformationEpsilon(trans_epsilon);
    ndt_new.setStepSize(step_size);
    ndt_new.setResolution(resolution);
    ndt_new.setMaximumIterations(max_iterations);

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
    ndt_new.setInputTarget(map_points_ptr);
    // create Thread
    // detach
    pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt_new.align(*ndt_output_cloud, Eigen::Matrix4f::Identity());

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_new;

    icp_new.setMaxCorrespondenceDistance(100);
    icp_new.setMaximumIterations(100);
    icp_new.setTransformationEpsilon(1e-6);
    icp_new.setEuclideanFitnessEpsilon(1e-6);
    icp_new.setRANSACIterations(0);

    icp_new.setInputTarget(map_points_ptr);
    // create Thread
    // detach
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    icp_new.align(*icp_output_cloud, Eigen::Matrix4f::Identity());

    // swap
    ndt_map_mtx_.lock();
    ndt_ = ndt_new;
    icp_ = icp_new;
    ndt_map_mtx_.unlock();
}

void NdtLocalizer::callback_maplla(const sensor_msgs::NavSatFix::ConstPtr &navsat_msg_ptr) {
    Eigen::Vector3d map_lla_vector = Eigen::Vector3d(navsat_msg_ptr->latitude, navsat_msg_ptr->longitude,
                                                     navsat_msg_ptr->altitude);

    gnssTools.lla_origin_ = map_lla_vector;

    ROS_WARN("Map ENU original LLA is: lat:%f lon:%f alt:%f", map_lla_vector(0), map_lla_vector(1), map_lla_vector(2));
}


void NdtLocalizer::callback_pointcloud(
        const sensor_msgs::PointCloud2::ConstPtr &sensor_points_sensorTF_msg_ptr) {
    const auto exe_start_time = std::chrono::system_clock::now();
    // mutex Map
    std::lock_guard<std::mutex> lock(ndt_map_mtx_);

    const std::string sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
    const auto sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_sensorTF_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);
    // get TF base to sensor
    geometry_msgs::TransformStamped::Ptr TF_base_to_sensor_ptr(new geometry_msgs::TransformStamped);
    get_transform(base_frame_, sensor_frame, TF_base_to_sensor_ptr);

    const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_base_to_sensor_ptr);
    const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_baselinkTF_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(
            *sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix);

    // set input point cloud
    ndt_.setInputSource(sensor_points_baselinkTF_ptr);

    if (ndt_.getInputTarget() == nullptr) {
        ROS_WARN_STREAM_THROTTLE(1, "No MAP!");
        return;
    }
    // align
    Eigen::Matrix4f initial_pose_matrix;
    if (!init_pose) {
        Eigen::Affine3d initial_pose_affine;
        tf2::fromMsg(initial_pose_cov_msg_.pose.pose, initial_pose_affine);
        initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

        double roll, pitch, yaw;
        tf::Quaternion init_orientation;
        tf::quaternionMsgToTF(initial_pose_cov_msg_.pose.pose.orientation, init_orientation);
        tf::Matrix3x3(init_orientation).getRPY(roll, pitch, yaw);
        const geometry_msgs::Pose init_pose_msg = tf2::toMsg(initial_pose_affine);
        ROS_WARN("Lidar Odom init position: east: %f, north: %f, up: %f, roll:%f, pitch:%f, yaw:%f.",
                 init_pose_msg.position.x, init_pose_msg.position.y, init_pose_msg.position.z, roll, pitch, yaw);


        //use the outcome of ndt as the initial guess for ICP
//        icp_.setInputSource(sensor_points_baselinkTF_ptr);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr icp_align_result(new pcl::PointCloud<pcl::PointXYZ>());
//        icp_.align(*icp_align_result, initial_pose_matrix);
//
//        const Eigen::Matrix4f icp_result_pose_matrix = icp_.getFinalTransformation();
//        initial_pose_matrix = icp_result_pose_matrix;


//        pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//        ndt_.align(*ndt_output_cloud, icp_result_pose_matrix);
//
//        initial_pose_matrix = ndt_.getFinalTransformation();
        // for the first time, we don't know the pre_trans, so just use the init_trans,
        // which means, the delta trans for the second time is 0
        pre_trans = initial_pose_matrix;
        delta_trans.setIdentity();

        init_pose = true;

    } else {
        // use predicted pose as init guess (currently we only impl linear model)
        initial_pose_matrix = pre_trans * delta_trans;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const auto align_start_time = std::chrono::system_clock::now();
    key_value_stdmap_["state"] = "Aligning";
    ndt_.align(*output_cloud, initial_pose_matrix);
    key_value_stdmap_["state"] = "Sleeping";
    const auto align_end_time = std::chrono::system_clock::now();
    const double align_time =
            std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() / 1000.0;

    const Eigen::Matrix4f result_pose_matrix = ndt_.getFinalTransformation();
    Eigen::Affine3d result_pose_affine;
    result_pose_affine.matrix() = result_pose_matrix.cast<double>();
    const geometry_msgs::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

    const auto exe_end_time = std::chrono::system_clock::now();
    const double exe_time =
            std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;

    const float transform_probability = ndt_.getTransformationProbability();
    const int iteration_num = ndt_.getFinalNumIteration();

    bool is_converged = true;
    static size_t skipping_publish_num = 0;
    if (
            iteration_num >= ndt_.getMaximumIterations() + 2 ||
            transform_probability < converged_param_transform_probability_) {
        is_converged = false;
        ++skipping_publish_num;
        std::cout << "Not Converged" << std::endl;
    } else {
        skipping_publish_num = 0;
    }
    // calculate the delta tf from pre_trans to current_trans
    delta_trans = pre_trans.inverse() * result_pose_matrix;

    Eigen::Vector3f delta_translation = delta_trans.block<3, 1>(0, 3);
    std::cout << "delta east: " << delta_translation(0) << " north: " << delta_translation(1) <<
              " up: " << delta_translation(2) << std::endl;

    Eigen::Matrix3f delta_rotation_matrix = delta_trans.block<3, 3>(0, 0);
    Eigen::Vector3f delta_euler = delta_rotation_matrix.eulerAngles(2, 1, 0);
    std::cout << "delta yaw: " << delta_euler(0) << " pitch: " << delta_euler(1) <<
              " roll: " << delta_euler(2) << std::endl;

//    pre_trans = result_pose_matrix;
    // refer to: https://github.com/FAIRSpace-AdMall/ndt_localizer/blob/master/nodes/ndt.cpp
    double_t deviation_t, deviation_r;

    if (!is_converged) {
        deviation_t = deviation_r = 1000.;
        delta_trans.setIdentity();
    } else {
        pre_trans = result_pose_matrix;
        deviation_t = delta_translation.norm() * 10;
        deviation_r = std::min(std::abs(float(M_PI) - delta_euler(0)), std::abs(delta_euler(0))) * 10;
    }

    // publish
    geometry_msgs::PoseStamped result_pose_stamped_msg;
    geometry_msgs::PoseWithCovarianceStamped result_pose_stamped_with_cov_msg;
    result_pose_stamped_msg.header.stamp = sensor_ros_time;
    result_pose_stamped_msg.header.frame_id = map_frame_;
    result_pose_stamped_with_cov_msg.header = result_pose_stamped_msg.header;
    result_pose_stamped_msg.pose = result_pose_msg;
    result_pose_stamped_with_cov_msg.pose.pose = result_pose_msg;

    result_pose_stamped_with_cov_msg.pose.covariance[0] = deviation_t;
    result_pose_stamped_with_cov_msg.pose.covariance[7] = deviation_t;
    result_pose_stamped_with_cov_msg.pose.covariance[14] = deviation_t;
    result_pose_stamped_with_cov_msg.pose.covariance[21] = deviation_r;
    result_pose_stamped_with_cov_msg.pose.covariance[28] = deviation_r;
    result_pose_stamped_with_cov_msg.pose.covariance[35] = deviation_r;


    if (is_converged) {
        ndt_pose_pub_.publish(result_pose_stamped_msg);
    }
    if (!is_converged && init_pose && gnss_pose && skipping_publish_num > 10) {
        init_pose = false;
        gnss_pose = false;
    }

    if (publish_tf_flag) {
        publish_tf(map_frame_, base_frame_, result_pose_stamped_msg);
    }

    // publish aligned point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(
            *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
    sensor_msgs::PointCloud2 sensor_points_mapTF_msg;
    pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
    sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
    sensor_points_mapTF_msg.header.frame_id = map_frame_;
    sensor_aligned_pose_pub_.publish(sensor_points_mapTF_msg);


    std_msgs::Float32 exe_time_msg;
    exe_time_msg.data = exe_time;
    exe_time_pub_.publish(exe_time_msg);

    std_msgs::Float32 transform_probability_msg;
    transform_probability_msg.data = transform_probability;
    transform_probability_pub_.publish(transform_probability_msg);

    std_msgs::Float32 iteration_num_msg;
    iteration_num_msg.data = iteration_num;
    iteration_num_pub_.publish(iteration_num_msg);

    key_value_stdmap_["seq"] = std::to_string(sensor_points_sensorTF_msg_ptr->header.seq);
    key_value_stdmap_["transform_probability"] = std::to_string(transform_probability);
    key_value_stdmap_["iteration_num"] = std::to_string(iteration_num);
    key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);

    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "align_time: " << align_time << "ms" << std::endl;
    std::cout << "exe_time: " << exe_time << "ms" << std::endl;
    std::cout << "trans_prob: " << transform_probability << std::endl;
    std::cout << "iter_num: " << iteration_num << std::endl;
    std::cout << "skipping_publish_num: " << skipping_publish_num << std::endl;

    double roll, pitch, yaw;
    tf::Quaternion align_orientation;
    tf::quaternionMsgToTF(result_pose_stamped_with_cov_msg.pose.pose.orientation, align_orientation);
    tf::Matrix3x3(align_orientation).getRPY(roll, pitch, yaw);
    ROS_INFO("Lidar Odom current position: east: %f, north: %f, up: %f, roll:%f, pitch:%f, yaw:%f",
             result_pose_stamped_with_cov_msg.pose.pose.position.x,
             result_pose_stamped_with_cov_msg.pose.pose.position.y,
             result_pose_stamped_with_cov_msg.pose.pose.position.z, roll, pitch, yaw);

    if (!is_converged && delete_no_fixed) {
        return;
    }
    publish_navsatfix(result_pose_stamped_with_cov_msg, is_converged);
    publish_odom(result_pose_stamped_with_cov_msg);
}

void NdtLocalizer::init_params() {

    private_nh_.getParam("base_frame", base_frame_);
    ROS_INFO("base_frame_id: %s", base_frame_.c_str());

    double trans_epsilon = ndt_.getTransformationEpsilon();
    double step_size = ndt_.getStepSize();
    double resolution = ndt_.getResolution();
    int max_iterations = ndt_.getMaximumIterations();

    private_nh_.getParam("trans_epsilon", trans_epsilon);
    private_nh_.getParam("step_size", step_size);
    private_nh_.getParam("resolution", resolution);
    private_nh_.getParam("max_iterations", max_iterations);

    map_frame_ = "map";

    ndt_.setTransformationEpsilon(trans_epsilon);
    ndt_.setStepSize(step_size);
    ndt_.setResolution(resolution);
    ndt_.setMaximumIterations(max_iterations);

    ROS_INFO(
            "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", trans_epsilon,
            step_size, resolution, max_iterations);

    private_nh_.getParam(
            "converged_param_transform_probability", converged_param_transform_probability_);
}


bool NdtLocalizer::get_transform(
        const std::string &target_frame, const std::string &source_frame,
        const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr, const ros::Time &time_stamp) {
    if (target_frame == source_frame) {
        transform_stamped_ptr->header.stamp = time_stamp;
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
                tf2_buffer_.lookupTransform(target_frame, source_frame, time_stamp);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

        transform_stamped_ptr->header.stamp = time_stamp;
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

bool NdtLocalizer::get_transform(
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

void NdtLocalizer::publish_tf(
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

void NdtLocalizer::publish_navsatfix(
        const geometry_msgs::PoseWithCovarianceStamped pose, const bool is_fixed) {
    Eigen::Vector3d enu;
    enu.setIdentity();
    enu = Eigen::Vector3d(pose.pose.pose.position.x,
                          pose.pose.pose.position.y,
                          pose.pose.pose.position.z);
    Eigen::Vector3d ecef = gnssTools.ENU2ECEF(enu);
    Eigen::Vector3d lla = gnssTools.ECEF2LLA(ecef);

    ROS_INFO("Lidar Odom current LLA position: lat: %f, lon: %f, alt: %f.", lla(0), lla(1), lla(2));

    sensor_msgs::NavSatFix msg_out;
//    msg_out.header.frame_id = "lidar_link";
    msg_out.header.frame_id = pose.header.frame_id;
    msg_out.header.stamp = pose.header.stamp;

    msg_out.latitude = lla(0);
    msg_out.longitude = lla(1);
    msg_out.altitude = lla(2);

    msg_out.position_covariance[0] = pose.pose.covariance[0];
    msg_out.position_covariance[4] = pose.pose.covariance[7];
    msg_out.position_covariance[8] = pose.pose.covariance[14];
    msg_out.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    if (is_fixed) {
        msg_out.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    } else {
        msg_out.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    }

    navsatfix_pub_.publish(msg_out);
}

void NdtLocalizer::publish_odom(
        const geometry_msgs::PoseWithCovarianceStamped pose) {
    Eigen::Vector3d enu;
    enu.setIdentity();
    enu = Eigen::Vector3d(pose.pose.pose.position.x,
                          pose.pose.pose.position.y,
                          pose.pose.pose.position.z);
    Eigen::Vector3d ecef = gnssTools.ENU2ECEF(enu);
    Eigen::Vector3d lla = gnssTools.ECEF2LLA(ecef);

    nav_msgs::Odometry msg_out;
//    msg_out.header.frame_id = "lidar_link";
    msg_out.header.frame_id = pose.header.frame_id;
    msg_out.child_frame_id = base_frame_;
    msg_out.header.stamp = pose.header.stamp;

    geographic_msgs::GeoPoint pLLA;
    pLLA.latitude = lla(0);
    pLLA.longitude = lla(1);
    pLLA.altitude = lla(2);
    geodesy::UTMPoint pUTM;
    geodesy::fromMsg(pLLA, pUTM);

    msg_out.pose.pose.position.x = pUTM.easting;
    msg_out.pose.pose.position.y = pUTM.northing;
    msg_out.pose.pose.position.z = pUTM.altitude;

    msg_out.pose.pose.orientation = pose.pose.pose.orientation;
    msg_out.pose.covariance = pose.pose.covariance;

    odom_pub_.publish(msg_out);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "ndt_localizer");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.getParam("points_topic", POINTS_TOPIC);
    private_nh.getParam("imu_topic", IMU_TOPIC);
    private_nh.getParam("gnss_topic", GNSS_TOPIC);
    private_nh.getParam("delete_no_fixed", delete_no_fixed);
    private_nh.getParam("publish_tf", publish_tf_flag);
//    std::cout << IMU_TOPIC << "   " << GNSS_TOPIC << std::endl;

    NdtLocalizer ndt_localizer(nh, private_nh);

    ros::spin();

    return 0;
}