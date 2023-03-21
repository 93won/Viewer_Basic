#pragma once

#ifndef FRAME_H
#define FRAME_H

#include <Common.h>

struct Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    // std::mutex pose_mutex;         // Pose lock
    Eigen::Matrix3d K;    // Intrinsic
    Eigen::Matrix4d pose; // T_wc
    int img_width;
    int img_height;

    Frame(const Eigen::Matrix4d &pose_, const Eigen::Matrix3d &K_, const int img_width_, const int img_height_)
    {
        pose = pose_;
        K = K_;
        img_width = img_width_;
        img_height = img_height_;
    }

    // // coordinate transform: world, camera, pixel
    // Vec3 world2camera(const Vec3 &p_w);
    // Vec3 camera2world(const Vec3 &p_c);
    // Vec2 camera2pixel(const Vec3 &p_c);
    // Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);
    // Vec3 pixel2world(const Vec2 &p_p, double depth = 1);
    // Vec2 world2pixel(const Vec3 &p_w);
};

#endif