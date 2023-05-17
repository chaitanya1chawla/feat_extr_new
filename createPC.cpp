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
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size (); ++i)
    {
      (*cloud)[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
      (*cloud)[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
      (*cloud)[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
    } 

    while(ctr < 500){


    }
    cout << data[0];

}