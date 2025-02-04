#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#include "for_time.hpp"
#include "for_cloud.hpp"
#include "for_desc.hpp"
#include "for_io.hpp"

#include "pcl/io/pcd_io.h"
#include <pcl/filters/extract_indices.h> 
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

Eigen::MatrixXi circshift(const Eigen::MatrixXi& mat_in, int shift_num) {
    // shift columns to right direction 
    assert(shift_num >= 0);

    if(shift_num == 0) {
        Eigen::MatrixXi shifted_mat(mat_in);
        return shifted_mat; 
    }

    Eigen::MatrixXi shifted_mat = Eigen::MatrixXi::Zero(mat_in.rows(), mat_in.cols() );
    for (int col_id = 0; col_id < mat_in.cols(); col_id++ ) {
        int new_location = (col_id + shift_num) % mat_in.cols();
        shifted_mat.col(new_location) = mat_in.col(col_id);
    }

    return shifted_mat;
} 

void simulateOrientationWithinMap(const PointCloudPtr& cloud_map, const pcl::PointXYZ& cnt_pt,
    int rows, int cols, int length, std::vector<Eigen::MatrixXi>& descs) {
    
    Eigen::MatrixXi desc = Eigen::MatrixXi::Zero(rows, cols);
    if (cloud_map->empty()) {
        descs.resize(cols, desc); 
        return;
    }
    
    desc = makeOPDescMat(cloud_map, cnt_pt, rows, cols, length, -100, 100);
    for (int i = 0; i < desc.cols(); ++i) {
        Eigen::MatrixXi desc_shift = circshift(desc, i);
        descs.push_back(desc_shift);
    }  
}


std::string DIR_MAP = "./";

int main(int argc, char** argv) {
    ros::init(argc, argv, "collect_database");
    ros::NodeHandle nh("~");;

    nh.getParam("DIR_MAP", DIR_MAP);
    nh.getParam("PARA_LENGTH", PARA_LENGTH);

    std::cout << "**************************************************\n";
    std::cout << "***                                            ***\n";
    std::cout << "***  Loading Offline Map and Candidate Points  ***\n";
    std::cout << "***    Please do not turn off the terminal.    ***\n";
    std::cout << "***                                            ***\n";
    std::cout << "**************************************************\n";

    PointCloudPtr candidate_pts(new PointCloud), pass_map(new PointCloud);

    // candidate pts
    if (pcl::io::loadPCDFile(DIR_MAP + "candidate_pts.pcd", *candidate_pts) == 0) {
        std::cout << "Loaded candidate pts contains: " << candidate_pts->size() << " pts..\n";
    } else {
        ROS_ERROR("Failed to load candidate points from %s", (DIR_MAP + "candidate_pts.pcd").c_str());
    }
    for (auto &pt : candidate_pts->points) {
        pt.z = 0.0f;
    }

    // pass map 
    if (pcl::io::loadPCDFile(DIR_MAP + "pass_map.pcd", *pass_map) == 0) {
        std::cout << "Loaded pass map contains: " << pass_map->size() << " pts..\n";
    } else {
        ROS_ERROR("Failed to load pass map from %s", (DIR_MAP + "pass_map.pcd").c_str());
    }
    // removeOutliersSOR(pass_map, pass_map, 50, 1.0);
    // pcl::io::savePCDFileASCII(DIR_MAP + "pass_map_sor.pcd", *pass_map);
    
    // project to x-o-y
    for (auto &pt : pass_map->points) {
        pt.z = 0.0;
    }
    pcl::io::savePCDFileASCII(DIR_MAP + "xoy_map.pcd", *pass_map);

    std::cout << "**************************************************\n";
    std::cout << "***   Collecting map database, please don't    ***\n";
    std::cout << "***        shut down... Just a moment!         ***\n";
    std::cout << "**************************************************\n";
    float radius = PARA_ROWS * PARA_LENGTH;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pass_map);
    pcl::ExtractIndices<pcl::PointXYZ>::Ptr index_extractor(new pcl::ExtractIndices<pcl::PointXYZ>);
    index_extractor->setInputCloud(pass_map);
    
    // store matrix (PARA_ROWS * PARA_COLS)
    std::vector<Eigen::MatrixXi> descs_total;
    for (int i = 0; i < candidate_pts->size(); ++i) {
        if (!ros::ok()) {
            ROS_WARN("Program interrupted! Stopping processing.");
            break;
        }

        // display progress
        int progress = std::ceil((static_cast<float>(i) / candidate_pts->size()) * 100);
        if (i % 100 == 0 || i == candidate_pts->size() - 1) {
            std::cout << "### Progress: " << progress << "% (" << i << "/" << candidate_pts->size() << ") ###\n";
        }

        const pcl::PointXYZ& pt = candidate_pts->points[i];
        std::vector<int> indices;
        std::vector<float> distances_sq;
        tree->radiusSearch(pt, radius, indices, distances_sq);

        pcl::IndicesPtr indices_ptr = boost::make_shared<std::vector<int>>(indices);
        index_extractor->setIndices(indices_ptr);
        index_extractor->setNegative(false);
        PointCloudPtr near_map(new PointCloud);
        index_extractor->filter(*near_map);
        
        std::vector<Eigen::MatrixXi> desc_onept;
        simulateOrientationWithinMap(near_map, pt, PARA_ROWS, PARA_COLS, PARA_LENGTH, desc_onept);
        for (int j = 0; j < desc_onept.size(); ++j) {
            descs_total.push_back(desc_onept[j]);
        }
    }
    
    std::cout << "**************************************************\n";
    std::cout << "***      Finding hash keys, please don't       ***\n";
    std::cout << "***        shut down... Just a moment!         ***\n";
    std::cout << "**************************************************\n";
    std::unordered_map<size_t, std::vector<size_t>> key_locations; 
    for (size_t i = 0; i < descs_total.size(); ++i) {
        int progress = std::ceil((static_cast<float>(i) / descs_total.size()) * 100);
        if (i % 50000 == 0 || i == descs_total.size() - 1) {
            std::cout << "### Progress: " << progress << "% (" << i << "/" << descs_total.size() << ") ###\n";
        }

        for (size_t key = 0; key < PARA_CELL_NUM; ++key) {
            size_t r = key / PARA_COLS;
            size_t c = key % PARA_COLS;
            if (descs_total[i](r, c) > 0.5) {
                key_locations[key].push_back(i);
            }
        }   
    }

    std::cout << "**************************************************\n";
    std::cout << "***      Saving offline files, please don't    ***\n";
    std::cout << "***        shut down... Just a moment!         ***\n";
    std::cout << "**************************************************\n";
    for (const auto& pair : key_locations) {
        size_t key = pair.first;
        saveIntegersAsBinary(pair.second, DIR_MAP + "database/" + std::to_string(key) + ".bin");
    }

    return 0;
}




