#include <AndreiUtils/classes/PythonInterface.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <AndreiUtils/utilsJson.h>
#include <AndreiUtils/classes/DualQuaternion.hpp>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

using json = nlohmann::json;
using namespace AndreiUtils;
using namespace std;
using namespace Eigen;
using namespace pcl;


int main() {
    cout << "Hello World!" << endl;

    json data = readJsonFile("../data/demonstration_2023-05-08-13-32-14_980300108.json");

    PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Generate pointcloud data
    cloud->width = 500;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (int i = 0; i < 400; ++i) {
        vector<double> src = data[i]["objects"]["CerealBoxKelloggsMuslixInstance"]["geometryPose"];
        Posed q;
        q.fromCoefficients(src);
        auto t = q.getTranslation();
        (*cloud)[i].x = t.x();
        (*cloud)[i].y = t.y();
        (*cloud)[i].z = t.z();
        cout << t.transpose() << endl;
    }

    float resolution = 0.001f;
    resolution = 1.f;

    octree::OctreePointCloud<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    octree::OctreePointCloud<pcl::PointXYZ>::AlignedPointTVector ab;
    cout << octree.getOccupiedVoxelCenters(ab) << endl;
    // cout<<
}

