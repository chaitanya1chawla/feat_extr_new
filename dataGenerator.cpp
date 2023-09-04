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
#include <AndreiUtils/utilsEigenGeometry.h>
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

    Vector3d init_pos(config.at("trajectoryGenerator").at("initialPoint").at("translation").at("x"),
                      config.at("trajectoryGenerator").at("initialPoint").at("translation").at("y"),
                      config.at("trajectoryGenerator").at("initialPoint").at("translation").at("z"));
    Vector3d pos = init_pos;

    Quaterniond qr(config.at("trajectoryGenerator").at("initialPoint").at("rotation").at("w"),
                      config.at("trajectoryGenerator").at("initialPoint").at("rotation").at("x"),
                      config.at("trajectoryGenerator").at("initialPoint").at("rotation").at("y"),
                      config.at("trajectoryGenerator").at("initialPoint").at("rotation").at("z") );

    Posed q;
    // q.identity();
    // Matrix xv(2,2,2);
    // ////////////////////////////////////////////////////////
    q = Posed::identity().addTranslation(init_pos);
    q = q.addRotation(qr);
    cout<<"initial pose = "<<q<<endl;

    double dt = config.at("trajectoryGenerator").at("deltaT");

    Vector3d vel(config.at("trajectoryGenerator").at("constantVelocity").at("value").at("x"),
                    config.at("trajectoryGenerator").at("constantVelocity").at("value").at("y"),
                    config.at("trajectoryGenerator").at("constantVelocity").at("value").at("z"));

    double vel_t1 = config.at("trajectoryGenerator").at("constantVelocity").at("startTime");
    double vel_t2 = config.at("trajectoryGenerator").at("constantVelocity").at("endTime");


    Vector3d acc(config.at("trajectoryGenerator").at("constantAcceleration").at("value").at("x"),
                    config.at("trajectoryGenerator").at("constantAcceleration").at("value").at("y"),
                    config.at("trajectoryGenerator").at("constantAcceleration").at("value").at("z"));

    double acc_t1 = config.at("trajectoryGenerator").at("constantAcceleration").at("startTime");
    double acc_t2 = config.at("trajectoryGenerator").at("constantAcceleration").at("endTime");


    Vector3d angularVel(config.at("trajectoryGenerator").at("constantAngularVelocity").at("value").at("x"),
                        config.at("trajectoryGenerator").at("constantAngularVelocity").at("value").at("y"),
                        config.at("trajectoryGenerator").at("constantAngularVelocity").at("value").at("z") );

    double angularVel_t1 = config.at("trajectoryGenerator").at("constantAngularVelocity").at("startTime");
    double angularVel_t2 = config.at("trajectoryGenerator").at("constantAngularVelocity").at("endTime");

    double time = 0;
    int frame_index=0;

    vector<nlohmann::json> tracking;

    for(time = vel_t1; time < vel_t2; time += dt){
        json frame;
        frame["frameIndex"] = frame_index++;
        frame["time"] = time;
        json objects;
        json DrinkingMugTUM;
        pos += vel * dt;

        // Posed q_transform = Posed::identity().addTranslation(pos);
        // cout<<"new pos = "<<pos.transpose()<<endl;
        // cout<<"transform quaternion = "<<q_transform<<endl;
        q = q.addTranslation(vel*dt);
        // cout<<"new quaternion = "<<q.getTranslation()<<endl;
        
        vector<double> vect = q.coefficients();
        DrinkingMugTUM["geometryPose"] = vect;
        objects["DrinkingMugTUM-MPIInstance"] = DrinkingMugTUM;
        frame["objects"] = objects;
        tracking.push_back(frame);
    }

    for(time = acc_t1; time < acc_t2; time += dt){
        json frame;
        frame["frameIndex"] = frame_index++;
        frame["time"] = time;
        json objects;
        json DrinkingMugTUM;

        Vector3d last_pos = pos;
        pos += vel*dt + 0.5*acc*dt*dt;

        // Posed q_transform;
        // q_transform.addTranslation(pos);
        // q = q_transform * q;

        // cout<<"new pos = "<<pos.transpose()<<endl;
        q = q.addTranslation(pos - last_pos);
        // cout<<"new quaternion = "<<q.getTranslation()<<endl;
        vel += acc * dt;

        vector<double> vect = q.coefficients();
        DrinkingMugTUM["geometryPose"] = vect;
        objects["DrinkingMugTUM-MPIInstance"] = DrinkingMugTUM;
        frame["objects"] = objects;
        tracking.push_back(frame);
    }

    /*
    for(time = angularVel_t1; time < angularVel_t2; time += dt){
        json frame;
        frame["frameIndex"] = frame_index++;
        frame["time"] = time;
        json objects;
        json DrinkingMugTUM;

        //TODO: check the conversion from vector to quaternion
        Posed q_transform;
        q_transform = quaternionFromAngularVelocity(&angularVel, dt);
        q = q_transform * q;

        vector<double> vect = q.coefficients();

        DrinkingMugTUM["geometryPose"] = vect;
        objects["DrinkingMugTUM-MPIInstance"] = DrinkingMugTUM;
        frame["objects"] = objects;
        tracking.push_back(frame);
    }
    */

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