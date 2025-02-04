#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <signal.h> 
#include <geometry_msgs/TransformStamped.h>

bool g_shutdown_requested = false;

void sigintHandler(int sig) {
    ROS_INFO("Ctrl-C detected, shutting down...");
    g_shutdown_requested = true;
}

void loadPoseToMatrix(const std::string& filename, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& poses) {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::exit((std::cout << "Failed to open:" << filename << "\n", EXIT_FAILURE));
    }

    std::vector<std::vector<double>> values;
    std::string line;
    while (std::getline(ifs, line)) {
        std::istringstream ss(line);
        values.emplace_back(std::istream_iterator<double>{ss}, std::istream_iterator<double>{}); // C++ >=17
    }

    if (values.empty() || values[0].empty()) {
        std::exit((std::cout << "Values data is empty or malformed.", EXIT_FAILURE));
    }

    for (int i = 0; i < (int)values.size(); ++i) {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        for (int j = 0; j < (int)values[0].size(); ++j) {
            int row = j / 4;
            int col = j % 4;
            pose(row, col) = values[i][j];
        }
        poses.push_back(pose);
    } 
    ifs.close();
}

void loadXYZIToCloudXYZ(const std::string& bin_path, int index, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::string filename = bin_path + std::to_string(index) + ".pcd"; 
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
    
    if (pcl::io::loadPCDFile(filename, *cloud_xyzi) == -1) {
        PCL_ERROR("Couldn't read the file %s\n", filename.c_str());
        return;
    }

    cloud->resize(cloud_xyzi->size());
    for (size_t i = 0; i < cloud_xyzi->size(); ++i) {
        cloud->points[i].x = cloud_xyzi->points[i].x;
        cloud->points[i].y = cloud_xyzi->points[i].y;
        cloud->points[i].z = cloud_xyzi->points[i].z;
    }
}

// 生成时间戳
std::vector<double> generateTimestamp(int num_poses, double interval_seconds) {
    std::vector<double> timestamps(num_poses);
    double current_time = 0.0;  
    for (int i = 0; i < num_poses; ++i) {
        timestamps[i] = current_time;
        current_time += interval_seconds; 
    }
    return timestamps;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "play_mulran");
    ros::NodeHandle nh("~");

    std::string DIR_PCD;
    std::string PATH_POSE;
    int SKIP_NUM;
    nh.getParam("DIR_PCD", DIR_PCD);
    nh.getParam("PATH_POSE", PATH_POSE);
    nh.getParam("SKIP_NUM", SKIP_NUM);

    std::cout << "======== Playing Mulran Dataset ========\n";
    std::cout << "PCD Dir: " << DIR_PCD << "\n";
    std::cout << "POSE Path: " << PATH_POSE << "\n";

    // pose file 
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> poses;
    loadPoseToMatrix(PATH_POSE, poses);
    std::cout << "Pose total: " << poses.size() << "\n";
    
    // generate timestamp
    std::vector<double> times = generateTimestamp(poses.size(), 0.1);

    // publisher
    ros::Publisher pub_laser_cloud = nh.advertise<sensor_msgs::PointCloud2>("/lidar_points", 1000);
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/lidar_pose", 1000);
    geometry_msgs::PoseStamped pose_msg;

    // add a delay to wait subscriber
    ros::Duration(1.0).sleep(); 

    // Ctrl+C signal
    signal(SIGINT, sigintHandler);

    ros::Rate r(10.0);
    int line_count = 0;

    while (ros::ok() && !g_shutdown_requested) {
        if (line_count < poses.size()) {
            std::cout << "playing: " << line_count << "th pcd" << "\n";
            
            // times && cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            loadXYZIToCloudXYZ(DIR_PCD, line_count, laser_cloud);

            // transform
            Eigen::Matrix4f pose_matrix = poses[line_count];
            Eigen::Matrix3f rotation = pose_matrix.block<3, 3>(0, 0);
            Eigen::Quaternionf quaternion(rotation);
            
            pose_msg.pose.position.x = pose_matrix(0, 3);
            pose_msg.pose.position.y = pose_matrix(1, 3);
            pose_msg.pose.position.z = pose_matrix(2, 3);
            pose_msg.pose.orientation.x = quaternion.x();
            pose_msg.pose.orientation.y = quaternion.y();
            pose_msg.pose.orientation.z = quaternion.z();
            pose_msg.pose.orientation.w = quaternion.w();
            pose_msg.header.stamp = ros::Time().fromSec(times[line_count]);
            pose_msg.header.frame_id = "map";  
            pub_pose.publish(pose_msg);

            sensor_msgs::PointCloud2 laser_cloud_msg;
            pcl::toROSMsg(*laser_cloud, laser_cloud_msg);
            laser_cloud_msg.header.stamp = ros::Time().fromSec(times[line_count]);
            laser_cloud_msg.header.frame_id = "/camera_init";
            pub_laser_cloud.publish(laser_cloud_msg);

            line_count += SKIP_NUM;
            r.sleep();
        } else {
            break; // exit
        }
    }
    return 0;
}