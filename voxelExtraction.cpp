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


using json = nlohmann::json;
using namespace AndreiUtils;
using namespace std;
using namespace Eigen;
using namespace pcl;
int const maxIntermediaryPoints = 3;
int const number_of_clouds = 3;
// minimum distance between voxels to detect
double const maxDistance = 0.25;


bool sortbysec(pair<PointXYZ, int> const &a,
               pair<PointXYZ, int> const &b) {
    return (a.second < b.second);
}


int findMin(double *x) {

    cout << "x = " << x[0] << endl << x[1] << endl<<x[2]<<endl;
    double min_val = 100.0;
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
    data[2] = readJsonFile("../data/demonstration_2023-05-19-16-25-09_343253856.json");

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
    Posed qq;
    vector<double> src2 =data[0][10]["objects"]["BowlGreyIkeaInstance"]["geometryPose"];
    qq.fromCoefficients(src2);
    auto t2 = qq.getTranslation();
    cout<<"bowl location = "<<t2.x()<<", "<<t2.y()<<", "<<t2.z()<<endl;

    vector<double> src3 = data[0][10]["objects"]["DrinkingMugTUM-MPIInstance"]["geometryPose"];
    qq.fromCoefficients(src3);
    auto t3 = qq.getTranslation();
    cout<<"mug location = "<<t3.x()<<", "<<t3.y()<<", "<<t3.z()<<endl;

    //Point Cloud and Octree generation
    for (int k = 0; k < number_of_clouds; k++) {

        clouds[k] = PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        // Generate pointcloud data
        clouds[k]->width = 600;
        clouds[k]->height = 1;
        clouds[k]->points.resize(clouds[k]->width * clouds[k]->height);

        for (int i = 0; i < data[k].size()-1; ++i) {
            // the manipulated object instance here is assumed to be MilkCartonLidlInstance1
            vector<double> src = data[k][i]["objects"]["MilkCartonLidlInstance1"]["geometryPose"];
            q.fromCoefficients(src);
            auto t = q.getTranslation();
            (*clouds[k])[i].x = t.x();
            (*clouds[k])[i].y = t.y();
            (*clouds[k])[i].z = t.z();
            cout << t.transpose() << endl;
            cout<<"i = "<<i<<endl;
            if(i == 285){
                cout<<endl<<"done"<<endl;
            }
        }
        cout<<"CLOUD FINISHED!!!!!";
        octree[k].setInputCloud(clouds[k]);         // octree2.OctreePointCloud::setInputCloud(clouds[1]);
        octree[k].addPointsFromInputCloud();
        voxelNum[k] = octree[k].getOccupiedVoxelCenters(centers[k]);
        cout<<"CLOUD SET!";

    }

    cout << "TOTAL NUMBER OF VOXELS = " << voxelNum[0] << " and " << voxelNum[1];


    for (int k=0; k < number_of_clouds; k++) {
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
    for (auto & k : dist) {
        for (auto & h : k) {
            for (auto & i : h) {
                for (double & j : i) {
                    j = 100.0;
                }
            }
        }
    }

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

        cout<<endl<<"Intermediary Point no. - "<<k<<endl;
        vector<int> min_index;
        min_index.push_back(k);

        for (int h = 1; h < number_of_clouds; h++) {

            cout<<"Cloud no.- " <<h<<endl;
            int a = findMin(dist[0][h][k]);
            cout << "a = " << a << endl;

            if (dist[0][h][k][a] < maxDistance) {
                min_index.push_back(a);
            }
            else{
                min_index.push_back(-1);
            }
        }
        cout << "min_index = "<<min_index[1]<<" and "<<min_index[2]<<endl;

        int flag = 0;
        for (int i = 1; i < number_of_clouds - 1; i++) {
            for (int j = i + 1; j < number_of_clouds; j++) {

                int ctr1;
                ctr1 = min_index[i];
                int ctr2;
                ctr2 = min_index[j];

                if(ctr1 < 0 || ctr2 < 0 || ctr1 > maxIntermediaryPoints || ctr2 > maxIntermediaryPoints){
                    flag = 1;
                    continue;
                }
                if (dist[i][j][ctr1][ctr2] >= maxDistance) {
                    flag = 1;
                }
            }
        }

        char ch;
        string str;
        if (flag == 0) {

            cout << "Intermediary Point found at " << pairs[0][k].first<<endl;
            cout << "Was the detected Intermediary Point intended in the demonstrations? y/N"<<endl;
            cin >> ch;

            if(ch == 'y' || ch == 'Y'){
                cout << "Please give the object's name - "<<endl;
                cin >> str;
                cout << "Object - "<<str<<" set at "<<pairs[0][k].first<<endl;
            }
            if(ch == 'n' || ch == 'N'){
                cout << "Point deleted";
            }
        }

        else {
            cout<< "No point found";
        }


    }

    return 0;
}

