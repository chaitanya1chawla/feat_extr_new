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

using json = nlohmann::json;
using namespace AndreiUtils;

using namespace std;
using namespace Eigen;
using namespace pcl;

// set here the acceptable distance from surface in z direction in meters -
double const z_dist = 0.10;

void velocityExtract()
{

    vector<nlohmann::json> data = readJsonFile("../data/demonstration_2023-06-21-18-53-52_129873292.json").get<vector<json>>();
    cout << "Json data size = " << data.size() << endl;
    // to check if pose has been saved
    int flag = 0;
    int velocityCheck = 0;
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
        auto &lastTimeStamp = data[i].at("timeStamp");

        if (flag == 1)
        {
            continue;
        }

        auto currentPose = data[i + 1].at("objects").at("MilkCartonLidlInstance1").getTranslation();
        auto &currentTimeStamp = data[i + 1].at("timeStamp");
    cout << "Hello World!" << endl;

        double dist = sqrt(pow(currentPose.x() - initialPose.x(), 2) 
                        + pow(currentPose.y() - initialPose.y(), 2) 
                        8+ pow(currentPose.z() - initialPose.z(), 2))


            // check for stopping
            if (dist < min_dist)
        {
            velocityCheck++;
            if (velocityCheck > 150){
                 K
            }
        }
    }
}

int main()
{
    cout << "Hello World!" << endl;

    velocityExtract();

    return 0;
}