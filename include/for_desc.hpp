#ifndef _LIDAR_200_UTILS_OSC_H_
#define _LIDAR_200_UTILS_OSC_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <bitset>
#include "for_cloud.hpp"

// mapping
float PARA_GRID_SIZE_MAP = 0.8;
int PARA_MAX_PTS_PER_MAP_GRID = 20;
float PARA_MIN_NOVE = 1.0;
float PARA_GRID_SIZE_GROUND = 1.0;
float PARA_RASTER_SIZE = 1.0;

float PARA_MIN_Z_MAP = -0.5;
float PARA_MAX_Z_MAP = 50.0;
int PARA_MIN_PTS_PER_GROUND_GRID = 40;
float PARA_VOXEL_LEAF = 0.2;
float PARA_SENSOR_HEIGHT = 1.75;
std::string PARA_DIR_MAP = "./";

// database && relocate
int PARA_ROWS = 40;
int PARA_COLS = 60;
float PARA_LENGTH = 1.0;
float PARA_MIN_Z_LOCATE = -0.5;
float PARA_MAX_Z_LOCATE = 50.0;
const int PARA_CELL_NUM = PARA_ROWS * PARA_COLS;

typedef std::bitset<2400> Bitset; // PARA_ROWS * PARA_COLS
const double PI_2 = M_PI * 2.0;

// Occupied place description 
Bitset makeOPDescBit(const PointCloudPtr& cloud_in, const pcl::PointXYZ& pt_cnt, 
    const int& rows, const int& cols, const float& length, const float& min_z, const float& max_z) {

    const float dxy_max = rows * length;
    const float theta_reso = PI_2 / cols;
    Bitset desc;
    for (const auto& pt : cloud_in->points) {
        if (pt.z < min_z || pt.z > max_z) {
            continue;
        }

        float dx = pt.x - pt_cnt.x;
        float dy = pt.y - pt_cnt.y;
        float dxy = std::hypot(dx, dy);
        if (dxy > dxy_max) {
            continue;
        }

        float theta = std::atan2(dy, dx);
        if (theta < 0) {
            theta += PI_2;
        }

        // Check if r, c are within bounds
        const int r = std::floor((dxy_max - dxy) / length);
        const int c = std::floor(theta / theta_reso);
        if (r >= 0 && r < rows && c >= 0 && c < cols) {
            desc[r * cols + c] = 1;
        }
    }
    return desc;
}

int scoreBtnDescBit(const Bitset& desc_src, const Bitset& desc_dst) {
    return (desc_src & desc_dst).count();
}


Eigen::MatrixXi makeOPDescMat(const PointCloudPtr& cloud_in, const pcl::PointXYZ& pt_cnt, 
    const int& rows, const int& cols, const float& length, const float& min_z, const float& max_z) {

    const float dxy_max = rows * length;
    const float theta_reso = PI_2 / cols;

    Eigen::MatrixXi desc = Eigen::MatrixXi::Zero(rows, cols);
    for (const auto& pt : cloud_in->points) {
        if (pt.z < min_z || pt.z > max_z) {
            continue;
        } 

        float dx = pt.x - pt_cnt.x;
        float dy = pt.y - pt_cnt.y;
        float dxy = std::hypot(dx, dy);

        // Skip if point is too far
        if (dxy > dxy_max) {
            continue;
        } 

        // Calculate theta
        float theta = std::atan2(dy, dx);
        if (theta < 0) {
            theta += PI_2;
        }

        // Check if r, c are within bounds
        const int r = std::floor((dxy_max - dxy) / length);
        const int c = std::floor(theta / theta_reso);
        if (r >= 0 && r < rows && c >= 0 && c < cols) {
            desc(r, c) = 1;
        }
    }
    return desc;
}

Eigen::Matrix4f get2DPoseFromPointsandTheta(const pcl::PointCloud<pcl::PointXYZ>::Ptr& candidates,
    const int& pt_id, const int& theta_id, const float& theta_reso) {

    pcl::PointXYZ xyz_result = candidates->at(pt_id);
    float theta_result = theta_id * theta_reso;
    float cos_theta = std::cos(theta_result);
    float sin_theta = std::sin(theta_result);

    Eigen::Matrix3f rot;
    rot << cos_theta, -sin_theta, 0,
           sin_theta,  cos_theta, 0,
           0,          0,         1;

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = rot.inverse();  
    pose(0, 3) = xyz_result.x;
    pose(1, 3) = xyz_result.y;

    return pose;
}



#endif