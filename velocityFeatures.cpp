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

// set here the maximum acceptable speed -
double const max_spd = 0.25;

int speedLessThan(double dist, double time, int &speedLessThanCheck, int &speedCtr, double currentTimeStamp,
                  double &initialTimeStamp, double &finalTimeStamp) {

    // this "if" - is used when speedLessThan returns true for the first time.
    if (dist / time < max_spd && speedLessThanCheck == 0) {
        speedCtr = 0;
        speedLessThanCheck = 1;
        initialTimeStamp = currentTimeStamp;
    }
        // this "else if" - is used to count how many times consecutively did the speedLessThan returned true.
    else if (dist / time < max_spd && speedLessThanCheck == 1) {
        speedCtr++;
    }
        // this "else if" - is used when speedLessThan is no more true and to see final time.
    else if (!dist / time < max_spd && speedLessThanCheck == 1) {
        finalTimeStamp = currentTimeStamp;
        speedLessThanCheck = 0;
    }

    return speedCtr;
}

int speedConstant(double val, int &speedCtr, int &speedConstantCheck) {

    // this "if" - is used when speedConstant returns true for the first time.
    if ((val > -0.1 || val < 0.1) && speedConstantCheck == 0) {
        speedCtr = 0;
        speedConstantCheck = 1;
    }
        // this "else if" - is used to count how many times consecutively did the speedConstant returned true.
    else if ((val > -0.1 || val < 0.1) && speedConstantCheck == 1) {
        speedCtr++;
    }
        // this "else if" - is used when speedConstant is no more true and to see final time.
    else if (!(val > -0.1 || val < 0.1) && speedConstantCheck == 1) {
        speedConstantCheck = 0;
    }

    return speedCtr;
}

int speedLinIncDec(double val, int &accCtr, int &speedLinCheck, int &speedLinflag) {

    // TODO : check if the values for val condition are correct
    // this "if" - is used when speedConstant returns true for the first time.
    if ((val > -0.01 || val < 0.1) && speedLinCheck == 0) {
        accCtr = 0;
        speedLinCheck = 1;
    }
    // this "else if" - is used to count how many times consecutively did the speedConstant returned true.
    else if ((val > -0.01 || val < 0.1) && speedLinCheck == 1) {
        accCtr++;
    }
    // this "else if" - is used when speedConstant is no more true and to see final time.
    else if (!(val > -0.01 || val < 0.1) && speedLinCheck == 1) {
        speedLinCheck = 0;
        speedLinflag = 1;
    }

    // this "if" - is used when speedConstant returns true for the first time.
    if ((val > -0.1 || val < 0.01) && speedLinCheck == 0) {
        accCtr = 0;
        speedLinCheck = 1;
    }
    // this "else if" - is used to count how many times consecutively did the speedConstant returned true.
    else if ((val > -0.1 || val < 0.01) && speedLinCheck == 1) {
        accCtr++;
    }
    // this "else if" - is used when speedConstant is no more true and to see final time.
    else if (!(val > -0.1 || val < 0.01) && speedLinCheck == 1) {
        speedLinCheck = 0;
        speedLinflag = -1;
    }

    return accCtr;
}

double avg_queue(queue<double> q) {

    double ctr = 0;
    if (q.empty()) {
        cout << "Error: Queue is empty" << endl;
        return 0;
    }
    double sum = 0;
    for (double i = q.front(); i <= q.back(); i = i + 1.0) {
        sum += q.front();
        q.pop();
        ctr++;
    }
    return sum / ctr;
}

void speedExtract() {

    vector<nlohmann::json> data = readJsonFile(
            "../data/demonstration_2023-06-21-18-53-52_129873292.json").get<vector<json>>();

    cout << "Json data size = " << data.size() << endl;
    int speedLessThanCheck = 0;
    int speedCtr = 0;
    double initialTimeStamp = 0;
    double finalTimeStamp = 0;
    Posed q;
    queue<double> distQueue;
    queue<double> spdQueue;
    vector<double> movingAvgSpd;
    // TODO : check which of these moving average accelerations work better?
    // one calculates the direct velocity, get the acc, and then takes moving average of acceleration.
    // second takes the moving average velocities to calculate acc.
    vector<double> movingAvgAcc_1;
    vector<double> movingAvgAcc_2;
    int ctr = 0;

    // checking speedLessThan and calculating moving averages
    for (int i = 0; i < data.size() - 2; i++) {

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
        double lastTimeStamp = data[i].at("timestamp");


        auto currentPose = data[i + 1].at("objects").at("MilkCartonLidlInstance1").at(
                "geometryPose").get<Posed>().getTranslation();
        double currentTimeStamp = data[i + 1].at("timestamp");

        auto nextPose = data[i + 2].at("objects").at("MilkCartonLidlInstance1").at(
                "geometryPose").get<Posed>().getTranslation();
        double nextTimeStamp = data[i + 2].at("timestamp");

        double dist_1 = sqrt(pow(currentPose.x() - initialPose.x(), 2)
                             + pow(currentPose.y() - initialPose.y(), 2)
                             + pow(currentPose.z() - initialPose.z(), 2));
        double time_1 = currentTimeStamp - lastTimeStamp;

        double dist_2 = sqrt(pow(nextPose.x() - currentPose.x(), 2)
                             + pow(nextPose.y() - currentPose.y(), 2)
                             + pow(nextPose.z() - currentPose.z(), 2));
        double time_2 = nextTimeStamp - currentTimeStamp;
        // to count how many poses were successfully detected
        ctr++;


        // checking if speed was kept less than a value in a certain time period
        speedCtr = speedLessThan(dist_1, time_1, speedLessThanCheck, speedCtr,
                                 currentTimeStamp, initialTimeStamp, finalTimeStamp);
        // if speed was kept less than max_spd for 3-4 seconds (100 frames) -
        if (speedCtr != 0 && speedCtr % 50 == 0) {
            cout << "You kept the speed of the object less than - " << max_spd;
            cout << " between time period - " << initialTimeStamp << " and " << finalTimeStamp << endl;
        }


        distQueue.push(dist_1 / time_1);
        // avg acc = Vbc - Vab / (tc-ta)
        spdQueue.push((dist_2 / time_2) - (dist_1 / time_1) / (time_2 + time_1));

        // taking moving average of last 50 values
        if (ctr >= 50) {
            movingAvgSpd.push_back(avg_queue(distQueue));
            movingAvgAcc_1.push_back(avg_queue(spdQueue));
            distQueue.pop();
            spdQueue.pop();
        }
        if (movingAvgSpd.size() > 1) {
            // finds moving average acc with prev 25 values and next 25 values
            double time_diff = (double) data[i - 24].at("timestamp") - (double) data[i - 25].at("timestamp");
            movingAvgAcc_2.push_back((movingAvgSpd.at(ctr - 50) - movingAvgSpd.at(ctr - 51)) / time_diff);
        }


    }

    speedCtr = 0;
    initialTimeStamp = 0;
    finalTimeStamp = 0;
    int speedConstantCheck = 0;

    // comparing here the moving Averages for constant speed -
    for (int i = 0; i < movingAvgSpd.size() - 1; i++) {

        // Percent increase in speed wont work here, as it might differ for bigger/smaller values of speeds.
        // Thus I used absolute difference.
        double val = (movingAvgSpd.at(i + 1) - movingAvgSpd.at(i)) / movingAvgSpd.at(i);
        speedCtr = speedConstant(val, speedCtr, speedConstantCheck);
        if (speedCtr == 1) {
            // the moving averages are associated with frames, s.t. we are taking
            // moving average of 25 previous values and 25 next values
            // Thus we take timestamp of the i+25th value, so that our avg is
            // calculated as described
            initialTimeStamp = data[i + 25].at("timestamp");
        }

        // if speed was kept constant for 3-4 seconds (100 frames) -
        if (speedCtr != 0 && speedCtr % 50 == 0) {
            finalTimeStamp = data[i + 25].at("timestamp");
            cout << "You kept constant speed of -  " << val;
            cout << " between time period - " << initialTimeStamp << " and " << finalTimeStamp << endl;
        }

    }

    int accCtr = 0;
    // speedLinflag is +1 for linear increasing speed and -1 for linear decreasing speed
    int speedLinflag = 0;
    initialTimeStamp = 0;
    finalTimeStamp = 0;
    int speedLinCheck = 0;

    // checking here for linearly increasing and decreasing
    for (int i = 0; i < movingAvgAcc_1.size() - 1; i++) {

        // Percent increase wont work here, as it might differ for bigger/smaller values of speeds.
        // Thus I used absolute difference.
        double val = (movingAvgAcc_1.at(i + 1) - movingAvgAcc_1.at(i)) / movingAvgAcc_1.at(i);
        accCtr = speedLinIncDec(val, accCtr, speedLinCheck, speedLinflag);

        if (accCtr == 1) {
            // the moving averages are associated with frames, s.t. we are taking
            // moving average of 25 previous values and 25 next values
            // Thus we take timestamp of the i+25th value, so that our avg is
            // calculated as described
            initialTimeStamp = data[i + 25].at("timestamp");
        }

        // if speed was kept constant for 3-4 seconds (100 frames) -
        if (accCtr != 0 && accCtr % 50 == 0) {
            finalTimeStamp = data[i + 25].at("timestamp");
            if (speedLinflag == 1){
                cout << "You kept constant roughly + ve constant acceleration of -  " << val;
                cout << " between time period - " << initialTimeStamp << " and " << finalTimeStamp << endl;
            }
            else if (speedLinflag == 1){
                cout << "You kept constant roughly -ve constant acceleration of -  " << val;
                cout << " between time period - " << initialTimeStamp << " and " << finalTimeStamp << endl;
            }
        }
    }


}

int main() {
    cout << "Hello World!" << endl;

    speedExtract();

    return 0;
}