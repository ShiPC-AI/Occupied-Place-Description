#ifndef _LIDAR_200_UTILS_CLOUD_H_
#define _LIDAR_200_UTILS_CLOUD_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h> 

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

void randomSampleCloud(const PointCloudPtr& cloud_in, PointCloudPtr& cloud_out, int N) {
    pcl::RandomSample<pcl::PointXYZ> random_sampler;
    random_sampler.setInputCloud(cloud_in);
    random_sampler.setSample(N);
    random_sampler.filter(*cloud_out);
}

void voxelSampleCloud(const PointCloudPtr& cloud_in, PointCloudPtr& cloud_out,
    const float leaf_x, const float leaf_y, const float leaf_z) {
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setInputCloud(cloud_in);
    grid.setLeafSize(leaf_x, leaf_y, leaf_z);
    grid.filter(*cloud_out);
}

void removeOutliersSOR(const PointCloudPtr& cloud_in, PointCloudPtr& cloud_out, 
    int mean_k, double stddev_mul_thresh) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev_mul_thresh);
    sor.setNegative(false);
    sor.filter(*cloud_out); 
}

float distBtnTwoPts(const pcl::PointXYZ& pt_1, const pcl::PointXYZ& pt_2) {
    return std::sqrt(std::pow(pt_1.x - pt_2.x, 2) + std::pow(pt_1.y - pt_2.y, 2) + std::pow(pt_1.z - pt_2.z, 2));
}

void distSampleCloud(const PointCloudPtr& cloud_in, PointCloudPtr& cloud_out, double max_dist) {
    std::vector<int> selected_indices;
    for (size_t i = 0; i < cloud_in->size(); ++i) {
        bool keep_point = true;
        for (size_t j = 0; j < selected_indices.size(); ++j) {
            float dist = distBtnTwoPts(cloud_in->at(i), cloud_in->at(selected_indices[j]));
            if (dist < max_dist) {
                keep_point = false;
                break;
            }
        }
        if (keep_point) {
            selected_indices.push_back(i);
        }
    }
    pcl::ExtractIndices<pcl::PointXYZ>::Ptr index_extractor(new pcl::ExtractIndices<pcl::PointXYZ>);
    index_extractor->setInputCloud(cloud_in);
    pcl::IndicesPtr indices_ptr = boost::make_shared<std::vector<int>>(selected_indices);
    index_extractor->setIndices(indices_ptr);
    index_extractor->setNegative(false);
    index_extractor->filter(*cloud_out);
}

#endif