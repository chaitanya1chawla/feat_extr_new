#include <AndreiUtils/classes/PythonInterface.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <AndreiUtils/utilsJson.h>
#include <AndreiUtils/classes/DualQuaternion.hpp>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_impl.h>

using json = nlohmann::json;
using namespace AndreiUtils;
using namespace std;
using namespace Eigen;
using namespace pcl;


int main() {
    cout << "Hello World!" << endl;
    int number_of_clouds = 1;
    json data[number_of_clouds];

    data[0] = readJsonFile("../data/demonstration_2023-05-08-13-32-14_980300108.json");
    data[1] = readJsonFile("../data/demonstration_2023-05-08-13-32-14_980300108.json");

    // PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    PointCloud<pcl::PointXYZ>::Ptr clouds[number_of_clouds];
    Posed q;

    float resolution = 0.001f;
    resolution = 0.01f;
    vector <octree::OctreePointCloudDensity<pcl::PointXYZ>> octree(number_of_clouds,resolution) ;

    int voxelNum [number_of_clouds];
    octree::OctreePointCloud<pcl::PointXYZ>::AlignedPointTVector centers[number_of_clouds];

    //Point Cloud and Octree generation
    for(int k = 0; k < number_of_clouds; k++){

        clouds[k] = PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
        // Generate pointcloud data
        clouds[k]->width = 500;
        clouds[k]->height = 1;
        clouds[k]->points.resize(clouds[k]->width * clouds[k]->height);

        for (int i = 0; i < 450; ++i) {
            vector<double> src = data[k][i]["objects"]["CerealBoxKelloggsMuslixInstance"]["geometryPose"];
            q.fromCoefficients(src);
            auto t  = q.getTranslation();
            (*clouds[k])[i].x = t.x();
            (*clouds[k])[i].y = t.y();
            (*clouds[k])[i].z = t.z();
            cout << t.transpose() << endl;
        }

        octree[k].setInputCloud(clouds[k]);         // octree2.OctreePointCloud::setInputCloud(clouds[1]);
        octree[k].addPointsFromInputCloud();
        voxelNum[k] = octree[k].getOccupiedVoxelCenters(centers[k]);

    }


    double dist[100][100] = {0};
/*
    for( int i=0; i < octree.getOccupiedVoxelCenters(centers[0]); i++ ) {

        cout<<centers[0][i];

        // OctreePointCloud inherits from OctreePointCloudDensity
        cout << "   No. of points at this voxel = " << octree.getVoxelDensityAtPoint(centers[0][i])<<endl;
    }

*/

    for(int i=0; i < min(voxelNum, voxelNum2); i++) {

        for (int j =0; j < max(voxelNum, voxelNum2); j++){

            dist[i][j] = pow((centers[0][i].x - centers[1][j].x),2) +
                         pow((centers[0][i].y - centers[1][j].y),2) +
                         pow((centers[0][i].z - centers[1][j].z),2);
        }

    }

    double *min = min_element( begin( dist[3] ), std::end( dist[3] ) );





}

