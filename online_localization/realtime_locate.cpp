#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include "for_time.hpp"
#include "for_cloud.hpp"
#include "for_desc.hpp"
#include "for_io.hpp"
#include "pcl/io/pcd_io.h"
#include <pcl/common/transforms.h>
#include <algorithm> // for std::max_element
#include <omp.h>
#include <pcl/search/kdtree.h> 
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

PointCloudPtr _candidate_pts(new PointCloud);
pcl::PointXYZ _cnt_pt(0.0f, 0.0f, 0.0f);
float _theta_reso = M_PI * 2.0 / (float) PARA_COLS;
int  _desc_total_num = 0;

Eigen::Matrix4f _pose_scan = Eigen::Matrix4f::Identity();
pcl::PointXYZ _xy_result = pcl::PointXYZ(0.0f, 0.0f, 0.0f);  // discard
float _theta_result = 0; // discard
int _theta_index = 0;
int _pt_index = 0;
float _max_score = 0.0f;

std::mutex mBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> _cloud_buf;
pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_scan(new pcl::PointCloud<pcl::PointXYZ>);

// re-lcoate
std::unordered_map<size_t, std::vector<size_t>>  _key_locations_relocate;

// local search
std::vector<std::vector<Bitset>> _descs_map_local;
pcl::search::Search<pcl::PointXYZ>::Ptr _tree_candidates(new pcl::search::KdTree<pcl::PointXYZ>());
int _local_num = 50;

// init
bool _has_initialized = false;
int _init_scan_num = 10;
std::vector<Bitset> _descs_init;
std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> _poses_init;
std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> _poses_opt_init;
std::vector<float> _scores_init;
std::queue<geometry_msgs::PoseStamped> _pose_msgs_init;

// real-time locate
float _var_1 = 0.0f;
float _var_2 = 0.0f;
float _var_scan = 0.0f;
float _var_motion = PARA_LENGTH * PARA_LENGTH * 0.01;
Eigen::Matrix4f _pose_last_1 = Eigen::Matrix4f::Identity();
Eigen::Matrix4f _pose_last_2 = Eigen::Matrix4f::Identity();

void recieveCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
	mBuf.lock();
	_cloud_buf.push(cloud_msg);
	mBuf.unlock();
}

void findMaxVotes(const Bitset& query) {
    _max_score = 0.0;
    std::vector<int> votes(_desc_total_num, 0); 
    omp_set_num_threads(4);
    #pragma omp parallel for
    for (size_t key = 0; key < PARA_CELL_NUM; ++key) {
        if (query[key] && _key_locations_relocate.count(key)) { 
            for (size_t index : _key_locations_relocate.at(key)) {
                #pragma omp atomic
                ++votes[index];
            }
        }
    } 
    auto max_iter = std::max_element(votes.begin(), votes.end());
    const int max_index = std::distance(votes.begin(),  max_iter);

    _pt_index = max_index / PARA_COLS;
    _theta_index = max_index % PARA_COLS;
    _max_score = *max_iter / (float)query.count();
}

// relocate
Eigen::Matrix4f relocateBySingleDesc(const Bitset& desc_curr){
    findMaxVotes(desc_curr);
    Eigen::Matrix4f pose = get2DPoseFromPointsandTheta(_candidate_pts, _pt_index, _theta_index, _theta_reso);
    return pose;
}

// local search
Eigen::Matrix4f localSearch(const Bitset& desc_curr, const Eigen::Matrix4f& pose_last){
    pcl::PointXYZ pt_last;
    pt_last.x = pose_last(0, 3);
    pt_last.y = pose_last(1, 3);
    pt_last.z = 0.0;

    std::vector<int> indices;
    std::vector<float> distances_sq;
    _tree_candidates->nearestKSearch(pt_last, _local_num, indices, distances_sq);

    float max_score = -1000.0;
    int best_point_index = 0;
    int best_theta_index = 0;
    float occupy_num = desc_curr.count();
    for (int i = 0; i < indices.size(); ++i) {
        for (int j = 0; j < PARA_COLS; ++j) {
            int id_pt = indices[i];
            float score = scoreBtnDescBit(desc_curr, _descs_map_local[id_pt][j]) / occupy_num;
            if (score > max_score) {
                best_point_index = id_pt;
                best_theta_index = j;
                max_score = score;
            }
        }
    }

    _pt_index = best_point_index;
    _theta_index = best_theta_index;
    _max_score = max_score;

    Eigen::Matrix4f pose = get2DPoseFromPointsandTheta(_candidate_pts, best_point_index, 
        best_theta_index, _theta_reso);
    return pose;
}

float reComputeSeqPoses(const int& index, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& poses_out) {
    float total_score = 0.0f;
    poses_out.resize(_init_scan_num);
    Eigen::Matrix4f pose_last = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f pose_curr = Eigen::Matrix4f::Identity();
    
    poses_out[index] = _poses_init[index];
    pose_last = _poses_init[index];
    if (index == 0) { // 0 -> N-1
        for (int i = index + 1; i < poses_out.size(); ++i) {
            pose_curr = localSearch(_descs_init[i], pose_last);
            pose_last = pose_curr;
            poses_out[i] = pose_curr;
            total_score += _max_score;
        }
        return total_score;
    } else if (index == (_init_scan_num - 1)) { // N-1 -> 0
        for (int i = index - 1; i >= 0; --i) {
            pose_curr = localSearch(_descs_init[i], pose_last);
            pose_last = pose_curr;
            poses_out[i] = pose_curr;
            total_score += _max_score;
        }
        return total_score;
    } else {
        for (int i = index + 1; i < poses_out.size(); ++i) {
            pose_curr = localSearch(_descs_init[i], pose_last);
            pose_last = pose_curr;
            poses_out[i] = pose_curr;
            total_score += _max_score;
        }

        pose_last = _poses_init[index];
        for (int i = index - 1; i >= 0; --i) {
            pose_curr = localSearch(_descs_init[i], pose_last);
            pose_last = pose_curr;
            poses_out[i] = pose_curr;
            total_score += _max_score;
        }
        return total_score;
    }
} 

void trajectoryInit() {
    if (_descs_init.size() != _init_scan_num || _poses_init.size() != _init_scan_num) {
        std::cout << "****** Warning: Trajectory init involves unequal sizes! ******\n";
        return;
    }

    float max_score = -1000.0;
    for (int i = 0; i < _descs_init.size(); ++i) {
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> poses_seq;
        float seq_score = reComputeSeqPoses(i, poses_seq); // 9 scores
        if (seq_score > max_score) {
            _poses_opt_init = poses_seq; // update seq pose
            max_score = seq_score; 
        }
    }
    
    max_score = 1.0 - max_score / (_init_scan_num - 1);
    _var_1 = std::pow(max_score * PARA_LENGTH, 2);
    _var_2 = std::pow(max_score * PARA_LENGTH, 2);
    _pose_last_1 = _poses_opt_init[_init_scan_num - 1];
    _pose_last_2 = _poses_opt_init[_init_scan_num - 2];
}

void onlineKalmanLocate(const Bitset& desc_scan) {
    _pose_scan = localSearch(desc_scan, _pose_last_1);
    float var_measure = std::pow((1.0 - _max_score)* PARA_LENGTH, 2) ;

    // pri: priori, pos: posterior 
    Eigen::Matrix<float, 1, 2> A_k; A_k << 2, -1;
    Eigen::Matrix<float, 2, 3> x_k_1; 
    x_k_1 << _pose_last_1(0, 3), _pose_last_1(1, 3), _pose_last_1(2, 3),
             _pose_last_2(0, 3), _pose_last_2(1, 3), _pose_last_2(2, 3);
    
    Eigen::Matrix<float, 1, 3> x_k_pri = A_k * x_k_1;
    Eigen::Matrix<float, 2, 2> P_k_1;
    P_k_1 << _var_1, 0, 0, _var_2;
    float P_k_pri = A_k * P_k_1 * A_k.transpose() + _var_motion;

    float C_k = 1.0;
    float K = P_k_pri * C_k * 1.0 / (C_k * P_k_pri * C_k + var_measure);

    Eigen::Matrix<float, 1, 3> x_k_pos = x_k_pri + K * 
        (_pose_scan.block<3, 1>(0, 3).transpose() - C_k * x_k_pri);
    
    _pose_scan(0, 3) = x_k_pos(0, 0);
    _pose_scan(1, 3) = x_k_pos(0, 1);
    _pose_scan(2, 3) = x_k_pos(0, 2);
    _var_scan =  (1 - K * C_k)* P_k_pri;
    
    // std::cout << "Var: " << var_measure << ", " << _var_motion << ", " << _var_scan << "\n";
    ///////////////////////////////////
    _var_2 = _var_1;
    _var_1 = _var_scan;
    _pose_last_2 = _pose_last_1;
    _pose_last_1 = _pose_scan;

}
int main(int argc, char** argv) {
    ros::init(argc, argv, "realtime_locate");
    ros::NodeHandle nh("~");;

    nh.getParam("PARA_DIR_MAP", PARA_DIR_MAP);
    nh.getParam("PARA_MIN_Z_LOCATE", PARA_MIN_Z_LOCATE);
    nh.getParam("PARA_MAX_Z_LOCATE", PARA_MAX_Z_LOCATE);
    nh.getParam("PARA_LENGTH", PARA_LENGTH);


    ros::Subscriber subscriber_scan = nh.subscribe("/lidar_points", 1000, recieveCloud);
    ros::Publisher publiser_aligned_scan = nh.advertise<sensor_msgs::PointCloud2>("/aligned_scan", 1000);
    ros::Publisher publisher_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose_3dof", 1000);
    ros::Publisher publisher_path = nh.advertise<nav_msgs::Path> ("/path", 10);

    sensor_msgs::PointCloud2 msg_aligned_scan;
    geometry_msgs::PoseStamped pose_msg;
    nav_msgs::Path msg_path;
    pose_msg.header.frame_id = "map"; 
    msg_path.header.frame_id = "map";

    std::cout << "==================================================\n";
    std::cout << "||                                              ||\n";
    std::cout << "||           Loading Map Database..             ||\n";
    std::cout << "||     Please do not turn off the terminal.     ||\n";
    std::cout << "||                                              ||\n";
    std::cout << "==================================================\n";
   
    // candidate pts
    if (pcl::io::loadPCDFile(PARA_DIR_MAP + "candidate_pts.pcd", *_candidate_pts) == 0) {
        std::cout << "Loaded candidate pts contains:" << _candidate_pts->size() << " points\n";
    } else {
        ROS_ERROR("Failed to load [[ Candidate Pts ]] from %s", (PARA_DIR_MAP + "candidate_pts.pcd").c_str());
    }
    _desc_total_num = _candidate_pts->size() * PARA_COLS;
    
    _descs_map_local.resize(_candidate_pts->size());
    for (int i = 0; i < _descs_map_local.size(); ++i) {
        _descs_map_local[i] = std::vector<Bitset>(PARA_COLS);
    }

    // build tree in xoy plane
    PointCloudPtr candidate_pts_2d(new PointCloud);
    pcl::copyPointCloud(*_candidate_pts, *candidate_pts_2d);
    for (auto &pt : candidate_pts_2d->points) {
        pt.z = 0.0;
    }
    _tree_candidates->setInputCloud(candidate_pts_2d);

    TicToc timer_database;
    for (size_t key = 0; key < PARA_CELL_NUM; ++key) {
        if (key % 400 == 0) {
            int progress = std::ceil(((float)key / PARA_CELL_NUM) * 100);
            std::cout << "### Progress: " << progress << "% (" << key << "/" << PARA_CELL_NUM << ") ###\n";
        }

        std::vector<size_t> numbers;
        readIntegersFromBinary(PARA_DIR_MAP + "database/" + std::to_string(key) + ".bin", numbers);
        
        // fill in relocate data
        _key_locations_relocate[key].assign(numbers.begin(), numbers.end());
        
        // fill in local search data
        for (const size_t& desc_index: numbers) {
            int id_pt = desc_index / PARA_COLS;
            int id_theta = desc_index % PARA_COLS;
            _descs_map_local[id_pt][id_theta][key] = 1;
        }
    }
    std::cout << "Loading database takes: " << timer_database.toc() << "ms\n\n";
    
    std::cout << "==================================================\n";
    std::cout << "||         Real-time Locating Wihtin Map        ||\n";
    std::cout << "||       Waiting for LiDRA scans to arrive      ||\n";
    std::cout << "==================================================\n\n";
    
    // float rate = 5.0;
    // ros::Rate loop_rate(rate);
    while (ros::ok()) {
        ros::spinOnce(); 
        if (_cloud_buf.empty()) {
            continue;
        }

        _cloud_scan->clear();
        pcl::fromROSMsg(*_cloud_buf.front(), *_cloud_scan);
        _cloud_buf.pop();
        TicToc timer_locate;

        voxelSampleCloud(_cloud_scan, _cloud_scan, 0.2, 0.2, 0.2);

        // TicToc timer_desc;
        Bitset desc_scan = makeOPDescBit(_cloud_scan, _cnt_pt, PARA_ROWS, PARA_COLS, PARA_LENGTH, PARA_MIN_Z_LOCATE, PARA_MAX_Z_LOCATE); 
        
        // case 1: init and local search
        if (!_has_initialized) { 
            // store scans for waiting init
            if (_poses_init.size() < _init_scan_num) { // size < 10
                Eigen::Matrix4f pose_scan = relocateBySingleDesc(desc_scan);
                std::cout << "Max score: " << _max_score << std::endl;
                _poses_init.push_back(pose_scan);
                _descs_init.push_back(desc_scan);
                
                // trigger traj init
                if (_poses_init.size() >= _init_scan_num) {
                    std::cout << "=============================================\n";
                    std::cout << "===== trigger trajectory initialization =====\n";
                    std::cout << "=============================================\n";

                    TicToc timer_init;
                    _scores_init.resize(_init_scan_num);
                    trajectoryInit();
                    std::cout << "Init takes " << timer_init.toc() << "ms\n";
                    _has_initialized = true;

                    for (int i = 0; i < _poses_opt_init.size(); ++i) {
                        pose_msg.header.stamp = ros::Time::now(); 
                        pose_msg.pose.position.x = _poses_opt_init[i](0, 3);
                        pose_msg.pose.position.y = _poses_opt_init[i](1, 3);
                        pose_msg.pose.position.z = _poses_opt_init[i](2, 3);
                        Eigen::Quaternionf quat(_poses_opt_init[i].block<3, 3>(0, 0));
                        pose_msg.pose.orientation.x = quat.x();
                        pose_msg.pose.orientation.y = quat.y();
                        pose_msg.pose.orientation.z = quat.z();
                        pose_msg.pose.orientation.w = quat.w();

                        msg_path.header.stamp = pose_msg.header.stamp;
                        msg_path.poses.push_back(pose_msg);
                        // std::cout << "x, y, z: " << _poses_opt_init[i](0, 3) << ", " << _poses_opt_init[i](1, 3) << ", " << _poses_opt_init[i](2, 3) << "\n";
                    }
                } else {
                    continue;
                }
            }
        } else {
            publisher_path.publish(msg_path);
            
            onlineKalmanLocate(desc_scan);
            // std::cout << "Online Location takes: " << timer_locate.toc() << "ms\n";

            pcl::transformPointCloud(*_cloud_scan, *_cloud_scan, _pose_scan);

            pcl::toROSMsg(*_cloud_scan, msg_aligned_scan);
            msg_aligned_scan.header.stamp = ros::Time::now();
            msg_aligned_scan.header.frame_id = "map"; 
            publiser_aligned_scan.publish(msg_aligned_scan);

            pose_msg.header.stamp = ros::Time::now(); 
            pose_msg.pose.position.x = _pose_scan(0, 3);
            pose_msg.pose.position.y = _pose_scan(1, 3);
            pose_msg.pose.position.z = _pose_scan(2, 3);
            Eigen::Quaternionf quat(_pose_scan.block<3, 3>(0, 0));
            pose_msg.pose.orientation.x = quat.x();
            pose_msg.pose.orientation.y = quat.y();
            pose_msg.pose.orientation.z = quat.z();
            pose_msg.pose.orientation.w = quat.w();
            publisher_pose.publish(pose_msg);

            // std::cout << "x, y, z: " << _pose_scan(0, 3) << ", " << _pose_scan(1, 3) << ", " << _pose_scan(2, 3) << "\n";            
        }

        // loop_rate.sleep();
    }

    return 0;
}
