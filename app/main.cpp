#include <Common.h>
#include <Viewer.h>
#include <Utils.h>

#include <time.h>

// Debug
cv::Scalar red(0, 0, 255);
cv::Scalar blue(255, 0, 0);
cv::Scalar green(0, 255, 0);
cv::Scalar black(0, 0, 0);
cv::Scalar purple(255, 0, 255);


int main(int argc, char **argv)
{

    Config::SetParameterFile("../config/config.yaml");
    

    Viewer::Ptr viewer = Viewer::Ptr(new Viewer);
    viewer->initialize();
    
    while (true)
    {
        viewer->spinOnce();
    }
}