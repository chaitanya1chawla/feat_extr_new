#include <AndreiUtils/classes/PythonInterface.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <utility>
#include <algorithm>
#include <cstring>

#include <bits/stdc++.h>
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
int const maxIntermediaryPoints = 3;
int const number_of_clouds = 2;


bool sortbysec(pair<PointXYZ, int> const &a,
               pair<PointXYZ, int> const &b) {
    return (a.second < b.second);
}


int findMin(double *x) {

    cout << "x = " << x[0] << endl << x[1] << endl;
    int min_val = 100;
    int min_index = 0;
    for (int i = 0; i < maxIntermediaryPoints; ++i) {
        if (x[i] < min_val) {
            min_index = i;
            min_val = x[i];
        }
    }
    return min_index;
}


int main() {
    cout << "Hello World!" << endl;
    json data[number_of_clouds];

    data[0] = readJsonFile("../data/demonstration_2023-05-19-16-24-31_679894510.json");
    data[1] = readJsonFile("../data/demonstration_2023-05-19-16-23-07_771535671.json");

    // PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    PointCloud<pcl::PointXYZ>::Ptr clouds[number_of_clouds];
    Posed q;

    float resolution = 0.001f;
    resolution = 0.01f;
    vector<octree::OctreePointCloudDensity<pcl::PointXYZ>> octree(number_of_clouds, resolution);

    // saves number of voxels in each point cloud
    int voxelNum[number_of_clouds];
    // saves location of centers of voxels for each point cloud
    octree::OctreePointCloud<pcl::PointXYZ>::AlignedPointTVector centers[number_of_clouds];
    // for each point cloud exists a vector of pairs (of coord of center's voxel and number of points in that voxel)
    vector<vector<pair<PointXYZ, int>>> pairs(number_of_clouds);

    //Point Cloud and Octree generation
    for (int k = 0; k < number_of_clouds; k++) {

        clouds[k] = PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        // Generate pointcloud data
        clouds[k]->width = 500;
        clouds[k]->height = 1;
        clouds[k]->points.resize(clouds[k]->width * clouds[k]->height);

        for (int i = 0; i < 450; ++i) {
            vector<double> src = data[k][i]["objects"]["MilkCartonLidlInstance1"]["geometryPose"];
            q.fromCoefficients(src);
            auto t = q.getTranslation();
            (*clouds[k])[i].x = t.x();
            (*clouds[k])[i].y = t.y();
            (*clouds[k])[i].z = t.z();
            cout << t.transpose() << endl;
        }

        octree[k].setInputCloud(clouds[k]);         // octree2.OctreePointCloud::setInputCloud(clouds[1]);
        octree[k].addPointsFromInputCloud();
        voxelNum[k] = octree[k].getOccupiedVoxelCenters(centers[k]);

    }

    cout << "TOTAL NUMBER OF VOXELS = " << voxelNum[0] << " and " << voxelNum[1];


    for (int k = 0; k < number_of_clouds; k++) {
        for (int i = 0; i < voxelNum[k]; i++) {

            cout << centers[k][i]; // printing i-th center of k-th point cloud
            // OctreePointCloud inherits from OctreePointCloudDensity
            cout << "   No. of points at this voxel = " << octree[k].getVoxelDensityAtPoint(centers[k][i]) << endl;
            pairs[k].push_back({centers[k][i], octree[k].getVoxelDensityAtPoint(centers[k][i])});
        }
        // sorting on basis of density of voxels, because voxels with high densities would be intermediatery points as the user would stop at that point shortly
        sort(pairs[k].begin(), pairs[k].end(), sortbysec);
        cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;
    }

    // Maximum number of Intermediary points expected in all the demonstrations -

    cout << "no problemo" << endl;

    // setting all distances as 100 meters, so that it isnt recognized as minimum
    double dist[number_of_clouds - 1][number_of_clouds][maxIntermediaryPoints][maxIntermediaryPoints] = {0};
    for (int k = 0; k < number_of_clouds - 1; ++k) {
        for (int h = 0; h < number_of_clouds; ++h) {
            for (int i = 0; i < maxIntermediaryPoints; ++i) {
                for (int j = 0; j < maxIntermediaryPoints; ++j) {
                    dist[k][h][i][j] = 100.0;
                }
            }
        }
    }
    //cout<<dist[0][1][1][0]<<endl<<dist[0][1][1][1]<<endl<<dist[0][1][1][2]<<endl<<dist[0][1][1][3]<<endl;

    for (int k = 0; k < number_of_clouds - 1; k++) {
        for (int h = k + 1; h < number_of_clouds; h++) {

            for (int i = 0; i < maxIntermediaryPoints; i++) {
                for (int j = 0; j < maxIntermediaryPoints; j++) {
                    dist[k][h][i][j] = sqrt(pow((pairs[k][i].first.x - pairs[h][j].first.x), 2) +
                                            pow((pairs[k][i].first.y - pairs[h][j].first.y), 2) +
                                            pow((pairs[k][i].first.z - pairs[h][j].first.z), 2));
                }

            }
        }
    }


    for (int k = 0; k < maxIntermediaryPoints; k++) {

        int min_index[number_of_clouds];
        min_index[0] = k;

        for (int h = 1; h < number_of_clouds; h++) {

            int a = findMin(dist[0][h][k]);
            cout << "a = " << a << endl;
            if (dist[0][h][k][a] < 0.05) {
                min_index[h] = a;
            }
        }
        cout << min_index[1];
/*
        int flag = 0;
        for (int i = 0; i < number_of_clouds - 1; i++) {
            for (int j = i + 1; j < number_of_clouds; j++) {
                int ctr1=min_index[i];
                int ctr2=min_index[j];
                if (!(dist[i][j][ctr1][ctr2] < 0.5)) {
                    flag = 1;
                }
            }
        }

        if (flag == 0) {
            cout << "Intermediary Point found at " << pairs[0][k].first;
        }
*/
    }


/*
 * for(int h = k+1; h < number_of_clouds; h++){
            for(int i = 0; i < maxIntermediaryPoints; i++){

                double *min = min_element( begin( dist[k][h][i] ), end( dist[k][h][i] ) );

                auto itr = find(begin(dist[k][h][i]), end(dist[k][h][i]), *min);
                auto loc = distance(dist[k][h][i], itr);
            }
        }
 *
 *
 *
 *
 *
    double dist[100][100] = {0};

    for(int i=0; i < min(voxelNum, voxelNum2); i++) {

        for (int j =0; j < max(voxelNum, voxelNum2); j++){

            dist[i][j] = pow((centers[0][i].x - centers[1][j].x),2) +
                         pow((centers[0][i].y - centers[1][j].y),2) +
                         pow((centers[0][i].z - centers[1][j].z),2);
        }

    }

    double *min = min_element( begin( dist[3] ), std::end( dist[3] ) );

*/



}

