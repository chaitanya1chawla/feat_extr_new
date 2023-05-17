#include <AndreiUtils/classes/PythonInterface.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <nlohmann/json.hpp>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

using json = nlohmann::json;
using namespace AndreiUtils;
using namespace std;
using namespace Eigen;

int main() {
    cout << "Hello World!" << endl;

    std::ifstream demo_file("../data/demonstration_2023-05-08-13-32-14_980300108.json");
    json data = json::parse(demo_file);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // Generate pointcloud data
    cloud->width = 500;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size (); ++i)
    {
        vector<double> vec = data[0]["objects"]["CerealBoxKelloggsMuslixInstance"]["geometryPose"];

        (*cloud)[i].x = src[5];
        (*cloud)[i].y = src[6];
        (*cloud)[i].z = src[7];
    } 

    float resolution = 128.0f;

    pcl::octree::OctreePointCloud<pcl::PointXYZ> octree (resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    
    octree.getOccupiedVoxelCenters()


}