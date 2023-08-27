//
// Created by chaitanya11 on 27.08.23.
//
#include <AndreiUtils/classes/PythonInterface.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include <bits/stdc++.h>
#include <AndreiUtils/utilsJson.h>
#include <AndreiUtils/utilsJsonEigen.hpp>
#include <AndreiUtils/classes/DualQuaternion.hpp>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <AndreiUtils/utilsTime.h>

using json = nlohmann::json;
using namespace AndreiUtils;

using namespace std;
using namespace Eigen;
using namespace pcl;


void dataGenerate(const json& config){
    Vector3d pos = (config.at("trajectoryGenerator").at("initialPoint").at("translation").at("x"),
    config.at("trajectoryGenerator").at("initialPoint").at("translation").at("y"),
    config.at("trajectoryGenerator").at("initialPoint").at("translation").at("z"));

    double r_x = config.at("trajectoryGenerator").at("initialPoint").at("rotation").at("x");
    double r_y = config.at("trajectoryGenerator").at("initialPoint").at("rotation").at("y");
    double r_z = config.at("trajectoryGenerator").at("initialPoint").at("rotation").at("z");
    double r_w = config.at("trajectoryGenerator").at("initialPoint").at("rotation").at("w");

    double dt = config.at("trajectoryGenerator").at("deltaT");

    Vector3d v = (config.at("trajectoryGenerator").at("constantVelocity").at("value").at("x"),
         config.at("trajectoryGenerator").at("constantVelocity").at("value").at("y"),
         config.at("trajectoryGenerator").at("constantVelocity").at("value").at("z"));

    double v_1 = config.at("trajectoryGenerator").at("constantVelocity").at("startTime");
    double v_2 = config.at("trajectoryGenerator").at("constantVelocity").at("endTime");


    Vector3d a = (config.at("trajectoryGenerator").at("constantAcceleration").at("value").at("x"),
          config.at("trajectoryGenerator").at("constantAcceleration").at("value").at("y"),
          config.at("trajectoryGenerator").at("constantAcceleration").at("value").at("z"));

    double a_1 = config.at("trajectoryGenerator").at("constantAcceleration").at("startTime");
    double a_2 = config.at("trajectoryGenerator").at("constantAcceleration").at("endTime");


    Vector3d w = (config.at("trajectoryGenerator").at("constantAngularVelocity").at("value").at("x"),
         config.at("trajectoryGenerator").at("constantAngularVelocity").at("value").at("y"),
         config.at("trajectoryGenerator").at("constantAngularVelocity").at("value").at("z") );

    double w_1 = config.at("trajectoryGenerator").at("constantAngularVelocity").at("startTime");
    double w_2 = config.at("trajectoryGenerator").at("constantAngularVelocity").at("endTime");

    double time = 0;
    int frame_index=0;

    vector<nlohmann::json> tracking;

    for(time = v_1; time < v_2; time += dt){
        json frame;
        frame["frameIndex"] = frame_index++;
        frame["time"] = time;
        json objects;
        json DrinkingMugTUM;
        pos += v * dt;

        //TODO: check the conversion from vector to quaternion
        vector<double> vect{ pos[0], pos[1], pos[2], r_x, r_y, r_z, r_w };

        DrinkingMugTUM["geometryPose"] = vect;
        objects["DrinkingMugTUM-MPIInstance"] = DrinkingMugTUM;
        frame["objects"] = objects;
        tracking.push_back(frame);
    }

    for(time = a_1; time < a_2; time += dt){
        json frame;
        frame["frameIndex"] = frame_index++;
        frame["time"] = time;
        json objects;
        json DrinkingMugTUM;

        pos += v*dt + 0.5*a*dt*dt;
        v += a * dt;

        //TODO: check the conversion from vector to quaternion
        vector<double> vect{ pos[0], pos[1], pos[2], r_x, r_y, r_z, r_w };

        DrinkingMugTUM["geometryPose"] = vect;
        objects["DrinkingMugTUM-MPIInstance"] = DrinkingMugTUM;
        frame["objects"] = objects;
        tracking.push_back(frame);
    }

    for(time = w_1; time < w_2; time += dt){
        json frame;
        frame["frameIndex"] = frame_index++;
        frame["time"] = time;
        json objects;
        json DrinkingMugTUM;

        pos += v*dt + 0.5*a*dt*dt;
        v += a * dt;

        //TODO: check the conversion from vector to quaternion
        vector<double> vect{ pos[0], pos[1], pos[2], r_x, r_y, r_z, r_w };

        DrinkingMugTUM["geometryPose"] = vect;
        objects["DrinkingMugTUM-MPIInstance"] = DrinkingMugTUM;
        frame["objects"] = objects;
        tracking.push_back(frame);
    }

    string timeStr = convertChronoToStringWithSubsecondsCustomJoin(SystemClock::now(), "_");
    string demonstrationFile = "../data/surfaceData_" + timeStr + ".json";
    writeJsonFile(demonstrationFile, tracking);

}


int main() {
    cout << "Hello World!" << endl;

    auto config = readJsonFile("../config/dataGeneratorParameters.json");

    dataGenerate(config);

    return 0;
}