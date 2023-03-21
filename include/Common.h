#pragma once
#ifndef COMMON_H
#define COMMON_H

// std
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <typeinfo>
#include <unordered_map>
#include <vector>
#include <fstream> // ifstream header
#include <sstream>
#include <chrono>
#include <limits>
#include <random>
#include <numeric>
#include <algorithm>

// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// glog
#include <glog/logging.h>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// split function
static std::vector<std::string> split(std::string input, char delimiter)
{
    std::vector<std::string> answer;
    std::stringstream ss(input);
    std::string temp;

    

    while (getline(ss, temp, delimiter))
    {
        answer.push_back(temp);
    }

    return answer;
}


static std::vector<size_t> argsort_d(const std::vector<double> &v)
{

    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);

    std::stable_sort(idx.begin(), idx.end(),
                     [&v](size_t i1, size_t i2)
                     { return v[i1] < v[i2]; });

    return idx;
}

static std::vector<size_t> argsort_i(const std::vector<int> &v)
{

    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);

    std::stable_sort(idx.begin(), idx.end(),
                     [&v](size_t i1, size_t i2)
                     { return v[i1] < v[i2]; });

    return idx;
}


#endif