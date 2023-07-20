#include <AndreiUtils/classes/PythonInterface.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <utility>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <cfloat>

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
    // cout<<"speedLessThanCheck = "<<speedLessThanCheck<<endl;
    // cout<<"speedCtr = " <<speedCtr<<endl;
    // this "if" - is used when speedLessThan returns true for the first time.
    if ((dist / time) < max_spd && speedLessThanCheck == 0) {
        speedCtr = 0;
        speedLessThanCheck = 1;
        initialTimeStamp = currentTimeStamp;
        // cout << "initialTimeStamp = "<<initialTimeStamp<<endl;
    }
        // this "else if" - is used to count how many times consecutively did the speedLessThan returned true.
    else if ((dist / time) < max_spd && speedLessThanCheck == 1) {
        speedCtr++;
    }
        // this "else if" - is used when speedLessThan is no more true and to see final time.
    else if (!((dist / time) < max_spd) && speedLessThanCheck == 1) {
        finalTimeStamp = currentTimeStamp;
        speedLessThanCheck = 2;
        // cout << "finalTimeStamp = "<<finalTimeStamp<<endl;
    }

    return speedCtr;
}

int speedConstant(double val, int &speedCtr, int &speedConstantCheck) {

    // this "if" - is used when speedConstant returns true for the first time.
    if ((val > -0.10 && val < 0.10) && speedConstantCheck == 0) {
        speedCtr = 0;
        speedConstantCheck = 1;
    }
        // this "else if" - is used to count how many times consecutively did the speedConstant returned true.
    else if ((val > -0.10 && val < 0.10) && speedConstantCheck == 1) {
        speedCtr++;
    }
        // this "else if" - is used when speedConstant is no more true and to see final time.
    else if (!(val > -0.10 && val < 0.10) && speedConstantCheck == 1) {
        speedConstantCheck = 2;
    }

    return speedCtr;
}

int speedLinIncDec(double val, int &accCtr, int &speedLinCheck, int &speedLinflag) {
    cout<<"speedLinCheck = "<< speedLinCheck << endl;
    cout<<"accCtr = " << accCtr << endl;

    // TODO : check if the values for val condition are correct
    // Speed Linear Increase --
    // this "if" - is used when speedLinIncDec returns true for the first time.
    if ((val > -0.1 && val < 0.1) && speedLinCheck == 0) {
        accCtr = 0;
        speedLinCheck = 1;
    }
    // this "else if" - is used to count how many times consecutively did the speedConstant returned true.
    else if ((val > -0.1 && val < 0.1) && speedLinCheck == 1) {
        accCtr++;
    }
    // this "else if" - is used when speedConstant is no more true and to see final time.
    else if (!(val > -0.1 && val < 0.1) && speedLinCheck == 1) {
        speedLinCheck = 2;
        speedLinflag = 1;
    }

    // Speed Linear Decrease --
    // this "if" - is used when speedConstant returns true for the first time.
    //if ((val > -0.1 && val < 0.01) && speedLinCheck == 0) {
    //    accCtr = 0;
    //    speedLinCheck = 1;
    //}
    //    // this "else if" - is used to count how many times consecutively did the speedConstant returned true.
    //else if ((val > -0.1 && val < 0.01) && speedLinCheck == 1) {
    //    accCtr++;
    //}
    //    // this "else if" - is used when speedConstant is no more true and to see final time.
    //else if (!(val > -0.1 && val < 0.01) && speedLinCheck == 1) {
    //    speedLinCheck = 2;
    //    speedLinflag = -1;
    //}

    return accCtr;
}

int angleConstant(double val, int &angleCtr, int &angleConstantCheck) {

    // this "if" - is used when speedConstant returns true for the first time.
    if ((val < M_PI / 10) && angleConstantCheck == 0) {
        angleCtr = 0;
        angleConstantCheck = 1;
    }
        // this "else if" - is used to count how many times consecutively did the speedConstant returned true.
    else if ((val < M_PI / 10) && angleConstantCheck == 1) {
        angleCtr++;
    }
        // this "else if" - is used when speedConstant is no more true and to see final time.
    else if (!(val < M_PI / 10) && angleConstantCheck == 1) {
        angleConstantCheck = 0;
    }

    return angleCtr;
}

double avg_queue(queue<double> q) {

    double ctr = 0;
    if (q.empty()) {
        cout << "Error: Queue is empty" << endl;
        return 0;
    }
    double sum = 0;
    while (q.size() != 0) {
        sum += q.front();
        q.pop();
        ctr++;
    }
    // cout<<"SUM = "<<sum<<endl;
    // cout<<"ctr = "<<ctr<<endl;
    return sum / ctr;
}

// let the user decide give 3 points of speed and their % change and do quadratic interpolation on that
double v1 = 0.5;
double v2 = 1.0;
double v3 = 2.0;
double p1 = 0.20;
double p2 = 0.10;
double p3 = 0.05;



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
    queue<double> dirQueue;
    vector<double> movingAvgSpd;

    // TODO : check which of these moving average accelerations work better?
    // first one calculates the direct velocity, get the acc, and then takes moving average of acceleration.
    // second one takes the moving average velocities to calculate acc.
    vector<double> movingAvgAcc_1;
    vector<double> movingAvgAcc_2;

    vector<double> movingAvgAcc_used;
    // moving average for the direction of velocity - 
    vector<double> movingAvgDir;
    int ctr = 0;



    // checking speedLessThan and calculating all moving averages
    // DONE
    for (int i = 0; i < data.size() - 2; i++) {

        if (!data[i].contains("objects") || !data[i + 1].contains("objects") || !data[i + 2].contains("objects")) {
            continue;
        }
        auto &objects = data[i].at("objects");
        if (!objects.contains("MilkCartonLidlInstance1") ||
            !data[i + 1].at("objects").contains("MilkCartonLidlInstance1") ||
            !data[i + 2].at("objects").contains("MilkCartonLidlInstance1")) {
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

        // TODO : check the following calculation of angle
        Vector3d v1 = currentPose - initialPose;
        Vector3d v2 = nextPose - currentPose;
        double r = v1.transpose() * v2;
        double angle = acos(r / (v1.norm() * v2.norm()));

        // to count how many poses were successfully detected
        ctr++;


        // checking if speed was kept less than a value in a certain time period
        // DONE
        /*
        speedCtr = speedLessThan(dist_1, time_1, speedLessThanCheck, speedCtr,
                                 currentTimeStamp, initialTimeStamp, finalTimeStamp);
        // if speed was kept less than max_spd for 3-4 seconds (100 frames) -
        cout << "speed = "<<dist_1/time_1<<endl;
        if (speedLessThanCheck == 2 && (finalTimeStamp - initialTimeStamp) > 0.09) {
            speedLessThanCheck = 0;
            cout << "You kept the speed of the object less than " << max_spd;
            cout << " between time period " << initialTimeStamp << " and " << finalTimeStamp << endl;
        }
        */

        distQueue.push(dist_1 / time_1);
        // avg acc = Vbc - Vab / (tc-ta)
        spdQueue.push((dist_2 / time_2) - (dist_1 / time_1) / (time_2 + time_1));
        dirQueue.push(angle);

        // taking moving average of last 20 values
        if (ctr >= 20) {
            movingAvgSpd.push_back(avg_queue(distQueue));
            movingAvgAcc_1.push_back(avg_queue(spdQueue));
            movingAvgDir.push_back(avg_queue(dirQueue));
            distQueue.pop();
            spdQueue.pop();
            dirQueue.pop();
        }
        if (movingAvgSpd.size() > 1) {
            // finds moving average acc with prev 10 values and next 10 values
            double time_diff = (double) data[i - 9].at("timestamp") - (double) data[i - 10].at("timestamp");
            movingAvgAcc_2.push_back((movingAvgSpd.at(ctr - 20) - movingAvgSpd.at(ctr - 21)) / time_diff);
        }

    }


    speedCtr = 0;
    initialTimeStamp = 0;
    finalTimeStamp = 0;
    int speedConstantCheck = 0;

    // comparing here the moving Averages for constant speed -
    /*
    for (int i = 0; i < movingAvgSpd.size() - 1; i++) {
        if (isnan(movingAvgSpd.at(i))) {
            cout << "WTF!!";
        }
        // Percent increase in speed would vary for smaller and bigger values of speeds
        cout << "moving AvgSpd = " << movingAvgSpd.at(i) << endl;
        double val = (movingAvgSpd.at(i + 1) - movingAvgSpd.at(i)) / movingAvgSpd.at(i);
        speedCtr = speedConstant(val, speedCtr, speedConstantCheck);
        if (speedCtr == 1) {
            // the moving averages are associated with frames, s.t. we are taking
            // moving average of 10 previous values and 10 next values
            // Thus we take timestamp of the i+10th value, so that our avg is
            // calculated as described
            initialTimeStamp = data[i + 10].at("timestamp");
        }

        // if speed was kept constant for 3-4 seconds (100 frames) -
        if (speedConstantCheck == 2) {
            speedConstantCheck = 0;
            finalTimeStamp = data[i + 10].at("timestamp");
            // for a long enough time period, we tell the user that the speed was held constant.
            if ((finalTimeStamp - initialTimeStamp) > 0.50) {
                cout << "You kept constant speed of  " << movingAvgSpd.at(i);
                cout << " between time period " << initialTimeStamp << " and " << finalTimeStamp << endl;
            }
        }

    }
    */

    int accCtr = 0;
    // speedLinflag is +1 for linear increasing speed and -1 for linear decreasing speed  --- not used anymore
    int speedLinflag = 0;
    initialTimeStamp = 0;
    finalTimeStamp = 0;
    int speedLinCheck = 0;

    // checking here for linearly increasing and decreasing
    // toggle here, which movingAvgAcc you want to use --
    movingAvgAcc_used = movingAvgAcc_1;
    for (int i = 0; i < movingAvgAcc_used.size() - 1; i++) {
        cout << "Moving AvgAcc = " << movingAvgAcc_used.at(i) << endl;

        double val = (movingAvgAcc_used.at(i + 1) - movingAvgAcc_used.at(i)) / movingAvgAcc_used.at(i);
        cout << "val = " << val << endl;
        accCtr = speedLinIncDec(val, accCtr, speedLinCheck, speedLinflag);

        if (accCtr == 1) {
            // the moving averages are associated with frames, s.t. we are taking
            // moving average of 10 previous values and 10 next values
            // Thus we take timestamp of the i+10th value, so that our avg is
            // calculated as described above.
            initialTimeStamp = data[i + 10].at("timestamp");
        }

        // if speed was kept constant for 3-4 seconds (100 frames) -
        if (speedLinCheck == 2) {
            speedLinCheck = 0;
            finalTimeStamp = data[i + 10].at("timestamp");
            if((finalTimeStamp - initialTimeStamp) > 0.5){
                if (movingAvgAcc_used.at(i) > 0) {
                    cout << "You kept roughly constant +ve constant acceleration of " << movingAvgAcc_used.at(i);
                    cout << " between time period " << initialTimeStamp << " and " << finalTimeStamp << endl;
                }
                else if (movingAvgAcc_used.at(i) < 0) {
                    cout << "You kept roughly constant -ve constant acceleration of " << movingAvgAcc_used.at(i);
                    cout << " between time period " << initialTimeStamp << " and " << finalTimeStamp << endl;
                }
            }
        }
    }


    int angleCtr = 0;
    initialTimeStamp = 0;
    finalTimeStamp = 0;
    int angleConstantCheck = 0;

    // checking for spikes in velocity (sudden direction change)
    /*
    for (int i = 0; i < movingAvgDir.size() - 1; i++) {
        if(isnan(movingAvgDir.at(i))){
            cout << "WTF!!";
        }
        cout << "moving AvgDir = " << movingAvgDir.at(i)<<endl;
        angleCtr = angleConstant(movingAvgDir.at(i), angleCtr, angleConstantCheck);
        if (angleCtr == 1) {
            // the moving averages are associated with frames, s.t. we are taking
            // moving average of 10 previous values and 10 next values
            // Thus we take timestamp of the i+10th value, so that our avg is
            // calculated as described
            initialTimeStamp = data[i + 10].at("timestamp");
        }

        // if speed was kept constant for 3-4 seconds (100 frames) -
        if (angleCtr != 0 && angleCtr % 50 == 0) {
            finalTimeStamp = data[i + 10].at("timestamp");
            cout << "You kept constant direction of motion";
            cout << " between time period - " << initialTimeStamp << " and " << finalTimeStamp << endl;
        }

        // TODO : check this value - 
        if (movingAvgDir.at(i) > M_PI/10){
            cout<<"There was a sudden change in direction!";
        }

    }
    */
}

int main() {
    cout << "Hello World!" << endl;

    speedExtract();

    return 0;
}