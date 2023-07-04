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
#include <stack>


using json = nlohmann::json;
using namespace AndreiUtils;

using namespace std;
using namespace Eigen;
using namespace pcl;

// set here the maximum acceptable velocity -
double const max_vel = 0.25;

int velocityLessThan(double dist, double time, int &velocityLessThanCheck, int &velocityCtr, double currentTimeStamp,
                      double &initialTimeStamp, double &finalTimeStamp) {

    // this "if" - is used when velocityLessThan returns true for the first time.
    if (dist / time < max_vel && velocityLessThanCheck == 0) {
        velocityCtr = 0;
        velocityLessThanCheck = 1;
        initialTimeStamp = currentTimeStamp;
    }
        // this "else if" - is used to count how many times consecutively did the velocityLessThan returned true.
    else if (dist / time < max_vel && velocityLessThanCheck == 1) {
        velocityCtr++;
    }
        // this "else if" - is used when velocityLessThan is no more true and to see final time.
    else if (!dist / time < max_vel && velocityLessThanCheck == 1) {
        finalTimeStamp = currentTimeStamp;
        velocityLessThanCheck = 0;
    }

    return velocityCtr;
}

int velocityConstant(double val, int &velocityCtr, double &initialTimeStamp, double &finalTimeStamp, int &velocityConstantCheck){

    // this "if" - is used when velocityConstant returns true for the first time.
    if ((val > -0.1 || val < 0.1 ) && velocityConstantCheck == 0) {
        velocityCtr = 0;
        velocityConstantCheck = 1;
        initialTimeStamp = currentTimeStamp;
    }
    // this "else if" - is used to count how many times consecutively did the velocityConstant returned true.
    else if ((val > -0.1 || val < 0.1 ) && velocityConstantCheck == 1) {
        velocityCtr++;
    }
    // this "else if" - is used when velocityConstant is no more true and to see final time.
    else if (!(val > -0.1 || val < 0.1 ) && velocityConstantCheck == 1) {
        finalTimeStamp = currentTimeStamp;
        velocityConstantCheck = 0;
    }

    return velocityCtr;
}

int avg_queue(queue<double> q) {

    int ctr = 0;
    if (q.empty()) {
        cout << "Error: Queue is empty" << endl;
        return 0;
    }
    int sum = 0;
    for (int i = q.front(); i <= q.back(); i++) {
        sum += q.front();
        q.pop();
        ctr++;
    }
    return sum / ctr;
}

void velocityExtract() {

    vector<nlohmann::json> data = readJsonFile(
            "../data/demonstration_2023-06-21-18-53-52_129873292.json").get<vector<json>>();

    cout << "Json data size = " << data.size() << endl;
    int velocityLessThanCheck = 0;
    int velocityCtr = 0;
    double initialTimeStamp = 0;
    double finalTimeStamp = 0;
    Posed q;
    queue<double> distQueue;
    vector<double> movingAvg;
    int ctr = 0;

    for (int i = 0; i < data.size() - 1; i++) {

        if (!data[i].contains("objects") || !data[i + 1].contains("objects")) {
            continue;
        }
        auto &objects = data[i].at("objects");
        if (!objects.contains("MilkCartonLidlInstance1") ||
            !data[i + 1].at("objects").contains("MilkCartonLidlInstance1")) {
            continue;
        }
        auto &milkCarton = objects.at("MilkCartonLidlInstance1");
        q = milkCarton.at("geometryPose").get<Posed>();
        auto initialPose = q.getTranslation();
        // initial timeframe
        double lastTimeStamp = data[i].at("timeStamp");


        auto currentPose = data[i + 1].at("objects").at("MilkCartonLidlInstance1").at(
                "geometryPose").get<Posed>().getTranslation();
        double currentTimeStamp = data[i + 1].at("timeStamp");

        double dist = sqrt(pow(currentPose.x() - initialPose.x(), 2)
                           + pow(currentPose.y() - initialPose.y(), 2)
                           + pow(currentPose.z() - initialPose.z(), 2));
        double time = currentTimeStamp - lastTimeStamp;
        ctr++;


        // checking if velocity was kept less than a value in a certain time period
        velocityCtr = velocityLessThan(dist, time, velocityLessThanCheck, velocityCtr,
                                       currentTimeStamp, initialTimeStamp, finalTimeStamp);
        // if velocity was kept less than max_vel for 3-4 seconds (100 frames) -
        if (velocityCtr != 0 && velocityCtr % 50 == 0) {
            cout << "You kept the velocity of the object less than - " << max_vel;
            cout << " between time period - " << initialTimeStamp << " and " << finalTimeStamp << endl;
        }



        distQueue.push(dist/time);
        if(ctr > 50){
            movingAvg.push_back(avg_queue(distQueue));
            distQueue.pop();
        }



    }

    velocityCtr = 0;
    initialTimeStamp = 0;
    finalTimeStamp = 0;
    int velocityConstantCheck = 0;

    // comparing here the moving Averages -
    for (int i = 0; i < movingAvg.size() - 1; i++) {

        double val = (movingAvg.at(i+1) - movingAvg.at(i))/ movingAvg.at(i);
        velocityCtr = velocityConstant(val, velocityCtr, initialTimeStamp, finalTimeStamp,
                                       velocityConstantCheck);
        if (velocityCtr != 0 && velocityCtr % 50 == 0) {
            cout << "You kept constant velocity of -  " << max_vel;
            cout << " between time period - " << initialTimeStamp << " and " << finalTimeStamp << endl;
        }

    }


}

int main() {
    cout << "Hello World!" << endl;

    velocityExtract();

    return 0;
}