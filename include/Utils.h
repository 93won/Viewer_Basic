#pragma once
#ifndef UTILS_H
#define UTILS_H

#include <Common.h>


bool readTrajectory(const std::string traj_file_path,
                    std::vector<double> &timestamp_trajectory,
                    std::vector<std::vector<double>> &pose_trajectory);

void readPointCloudByIndex(std::string pcd_save_path, int id, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double voxel_size);

void lidarToCamera(const int img_width,
                   const int img_height,
                   const double max_distance,
                   const Eigen::Matrix3d R_cl,
                   const Eigen::Vector3d t_cl,
                   const Eigen::Matrix3d K,
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_valid,
                   std::vector<Eigen::Vector2d> &uv_valid,
                   Eigen::MatrixXd &depth_img);


Eigen::Matrix4d vector2Transform(const std::vector<double> xyzq);

void transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<double> xyzq);

#endif