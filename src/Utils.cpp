#include <Utils.h>

bool readTrajectory(const std::string traj_file_path,
                    std::vector<double> &timestamp_trajectory,
                    std::vector<std::vector<double>> &pose_trajectory)
{
    std::string line;
    std::ifstream myfile(traj_file_path);

    bool head = true;
    if (myfile.is_open())
    {
        while (getline(myfile, line))
        {
            if (head)
            {
                head = false;
                continue;
            }
            std::vector<std::string> data = split(line, ' ');
            double time = std::stod(data[0]);
            std::vector<double> pose;

            for (int i = 0; i < 7; i++)
            {
                pose.push_back(std::stod(data[i + 1]));
            }

            timestamp_trajectory.push_back(time);
            pose_trajectory.push_back(pose);
        }
        myfile.close();

        return true;
    }

    else
    {
        std::cout << "Unable to open trajectory file (exit)" << std::endl;
        return false;
    }
}

void readPointCloudByIndex(std::string pcd_save_path, int id, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double voxel_size)
{
    std::vector<double> pcd_time;
    std::vector<std::string> pcd_path;

    // Read pcd name
    char path_char[pcd_save_path.length() + 1];
    strcpy(path_char, pcd_save_path.c_str());

    DIR *dr;
    struct dirent *en;
    dr = opendir(path_char); // open all directory
    if (dr)
    {
        while ((en = readdir(dr)) != NULL)
        {
            std::vector<std::string> association = split(en->d_name, '.');

            if (association.size() == 3 && association[2] == "txt")
            {
                pcd_path.push_back(en->d_name);
                std::string str_time = association[0] + "." + association[1];
                pcd_time.push_back(std::stod(str_time));
            }
        }
        closedir(dr);
    }

    // Make order
    std::vector<size_t> idxs = argsort_d(pcd_time);
    std::vector<std::string> pcd_path_temp;

    for (size_t i = 0; i < idxs.size(); i++)
        pcd_path_temp.push_back(pcd_path[idxs[i]]);

    pcd_path = pcd_path_temp;

    std::vector<std::string> lines;

    std::string pcd_path_full = pcd_save_path + "/" + pcd_path[id];
    std::ifstream myfile(pcd_path_full);

    std::string line;

    if (myfile.is_open())
    {
        while (std::getline(myfile, line))
            lines.push_back(line);

        myfile.close();
    }

    for (size_t i = 0; i < lines.size(); i++)
    {
        std::vector<std::string> xyzi = split(lines[i], ' ');
        Eigen::Vector3d xyz = {std::stod(xyzi[0]), std::stod(xyzi[1]), std::stod(xyzi[2])};

        // front
        if (xyz[0] > 0.3)
        {
            pcl::PointXYZ p;
            p.x = xyz[0];
            p.y = xyz[1];
            p.z = xyz[2];

            cloud->points.push_back(p);
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize((float)voxel_size, (float)voxel_size, (float)voxel_size);
    sor.filter(*cloud_filtered);

    cloud = std::move(cloud_filtered);
}

void lidarToCamera(const int img_width,
                   const int img_height,
                   const double max_distance,
                   const Eigen::Matrix3d R_cl,
                   const Eigen::Vector3d t_cl,
                   const Eigen::Matrix3d K,
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_valid,
                   std::vector<Eigen::Vector2d> &uv_valid,
                   Eigen::MatrixXd &depth_img)
{

    depth_img.resize(img_width, img_height);

    for (size_t i = 0; i < cloud->points.size(); i++)
    {

        Eigen::Vector3d xyz = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};

        // front
        Eigen::Vector3d uvw = K * (R_cl * xyz + t_cl);

        double u = uvw[0] / uvw[2];
        double v = uvw[1] / uvw[2];

        if (u >= 100 && u <= (double)img_width - 100 && v >= 100 && v <= (double)img_height - 100)
        {
            if (uvw[2] <= max_distance)
            {
                depth_img((int)u,(int)v) = uvw[2];
                cloud_valid->points.push_back(cloud->points[i]);
                Eigen::Vector2d uv = {u, v};
                uv_valid.push_back(uv);
            }
        }
    }
}

Eigen::Matrix4d vector2Transform(const std::vector<double> xyzq)
{
    Eigen::Quaterniond quat;
    quat.x() = xyzq[3];
    quat.y() = xyzq[4];
    quat.z() = xyzq[5];
    quat.w() = xyzq[6];
    Eigen::Vector3d translation = {xyzq[0], xyzq[1], xyzq[2]};

    Eigen::Matrix4d rigidTransform = Eigen::Matrix4d::Identity();
    rigidTransform.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
    rigidTransform.block<3, 1>(0, 3) = translation;

    return rigidTransform;
}

void transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<double> xyzq)
{
    Eigen::Matrix4f transform = vector2Transform(xyzq).template cast<float>();
    pcl::transformPointCloud(*cloud, *cloud, transform);
}

