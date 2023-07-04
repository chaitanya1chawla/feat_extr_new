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
#include <AndreiUtils/utilsJsonEigen.hpp>
#include <AndreiUtils/classes/DualQuaternion.hpp>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_density.h>


using json = nlohmann::json;
using namespace AndreiUtils;

using namespace std;
using namespace Eigen;
using namespace pcl;

// set here the maximum acceptable velocity -
double const max_vel = 0.25;

bool velocityLessThan(double dist, double time){
    return dist/time < max_vel;
}

void velocityExtract()
{

    vector<nlohmann::json> data = readJsonFile("../data/demonstration_2023-06-21-18-53-52_129873292.json").get<vector<json>>();

    cout << "Json data size = " << data.size() << endl;
    // to check if pose has been saved
    int flag = 0;
    int velocityLessThanCheck = 0;
    int velocityCtr = 0;
    double initialTimeStamp = 0;
    double finalTimeStamp = 0;
    Posed q;

    for (int i = 0; i < data.size() - 1; i++)
    {

        if (!data[i].contains("objects") || !data[i + 1].contains("objects"))
        {
            continue;
        }
        auto &objects = data[i].at("objects");
        if (!objects.contains("MilkCartonLidlInstance1") || !data[i + 1].at("objects").contains("MilkCartonLidlInstance1"))
        {
            continue;
        }
        auto &milkCarton = objects.at("MilkCartonLidlInstance1");
        q = milkCarton.at("geometryPose").get<Posed>();
        flag++;
        auto initialPose = q.getTranslation();
        // initial timeframe
        double lastTimeStamp = data[i].at("timeStamp");

        if (flag == 1)
        {            continue;
        }

        auto currentPose = data[i + 1].at("objects").at("MilkCartonLidlInstance1").at("geometryPose").get<Posed>().getTranslation();
        double currentTimeStamp = data[i + 1].at("timeStamp");

        double dist = sqrt(pow(currentPose.x() - initialPose.x(), 2) 
                           + pow(currentPose.y() - initialPose.y(), 2)
                           + pow(currentPose.z() - initialPose.z(), 2));
        double time = currentTimeStamp - lastTimeStamp;

        // this "if" - is used when velocityLessThan returns true for the first time.
        if (velocityLessThan(dist, time) && velocityLessThanCheck == 0){
            velocityCtr = 0;
            velocityLessThanCheck = 1;
            initialTimeStamp = currentTimeStamp;
        }
        // this "else if" - is used to count how many time consecutively did the velocityLessThan returned true.
        else if(velocityLessThan(dist, time) && velocityLessThanCheck == 1){
            velocityCtr++;
        }
        // this "else if" - is used when velocityLessThan is no more true and to see final time.
        else if (!velocityLessThan(dist, time) && velocityLessThanCheck == 1){
            finalTimeStamp = currentTimeStamp;
            velocityLessThanCheck = 0;
        }

        // if velocity was kept less than max_vel for 3-4 seconds -
        if(velocityCtr > 100){
            cout << "You kept the velocity of the object less than "<< max_vel;
            cout << " between time period - " << initialTimeStamp << " and " << finalTimeStamp << endl;
        }

    }
}

int main()
{
    cout << "Hello World!" << endl;

    velocityExtract();

    return 0;
}