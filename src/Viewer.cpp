#include <Viewer.h>

Viewer::Viewer()
{
}

void Viewer::initialize()
{
    const int UI_WIDTH = 200;
    pangolin::CreateWindowAndBind("GUI", 1024 * 2, 768 * 2);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState vis_camera_(pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
                                            pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    vis_camera = vis_camera_;

    vis_display = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f).SetHandler(new pangolin::Handler3D(vis_camera));
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
    id = 0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1);

    for (int i = 0; i < 100000; i++)
    {
        std::vector<float> color = {dis(gen), dis(gen), dis(gen)};
        colors.push_back(color);
    }
}

void Viewer::followCurrentFrame(pangolin::OpenGlRenderState &vis_camera)
{
    pangolin::OpenGlMatrix m(current_Twc);
    vis_camera.Follow(m, true);
}



void Viewer::drawFrame(const Frame::Ptr frame, std::vector<float> rgb)
{
    Eigen::Matrix4d T_wc = frame->pose;
    const float sz = 0.3;
    const int line_width = 2.0;
    double fx = frame->K(0, 0);
    double fy = frame->K(1, 1);
    double cx = frame->K(1, 2);
    double cy = frame->K(2, 2);
    int width = frame->img_width;
    int height = frame->img_height;

    glPushMatrix();

    Eigen::Matrix4f m = T_wc.template cast<float>();
    glMultMatrixf((GLfloat *)m.data());

    glColor3f(rgb[0], rgb[1], rgb[2]);

    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    glPopMatrix();
}

// void increaseID()
// {
//     id += 1;
// }

void Viewer::spinOnce()
{

    // Debug
    cv::Scalar red(0, 0, 255);
    cv::Scalar blue(255, 0, 0);
    cv::Scalar green(0, 255, 0);
    cv::Scalar black(0, 0, 0);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    vis_display.Activate(vis_camera);
    pangolin::Var<bool> button_show_triangles = pangolin::Var<bool>("ui.Show_triangles", false, false);
    pangolin::Var<bool> button_save_obj = pangolin::Var<bool>("ui.Save_Mesh", false, false);

    // followCurrentFrame(vis_camera);

    // if (pangolin::Pushed(button_show_triangles))
    // {
    //     if (!flagDrawCurrentTriangleMesh)
    //     {
    //         flagDrawCurrentTriangleMesh = true;
    //         std::cout << "Draw Triangles (" <<local_triangles.size()<<")"<< std::endl;
            
    //     }

    //     else
    //     {
    //         flagDrawCurrentTriangleMesh = false;
    //         std::cout << "Hide Triangles" << std::endl;
    //     }
    // }

   

    






    


    pangolin::FinishFrame();
}
