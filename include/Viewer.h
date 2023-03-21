#pragma once
#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <Common.h>
#include <Config.h>


#include <ctime>
#include <Utils.h>
#include <Frame.h>

// pangoline
#include <pangolin/pangolin.h>
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>

class Viewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();

    void spinOnce();
    void showResult();
    void initialize();


    std::mutex viewer_data_mutex_;
    pangolin::View vis_display;
    pangolin::OpenGlRenderState vis_camera;

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;
    std::vector<std::vector<int>> current_triangles;
    Frame::Ptr current_frame = nullptr;

    pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud;
    std::vector<std::vector<int>> ref_triangles;
    Frame::Ptr ref_frame = nullptr;

    int id;

    std::string result_save_path;

    std::vector<std::vector<float>> colors;

    Eigen::Matrix4d current_Twc;
    // plane

    void followCurrentFrame(pangolin::OpenGlRenderState &vis_camera);

    // Functions
    void setLocalFrame(Frame::Ptr &frame)
    {

        if (current_frame != nullptr)
        {
            Frame::Ptr copy_frame(new Frame(current_frame->pose, current_frame->K, current_frame->img_width, current_frame->img_height));
            ref_frame = copy_frame;
        }

        current_frame = frame;
    }
    void drawFrame(const Frame::Ptr frame, std::vector<float> rgb);
};

#endif
