//
// Created by chaitanya11 on 05.05.23.
//

#include <AndreiUtils/classes/PythonInterface.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include <json/value.h>
#include <fstream>

using namespace AndreiUtils;
using namespace std;
using namespace Eigen;


int main() {
    cout << "Hello World!" << endl;

    std::ifstream demo_file("data/demonstration_2023-03-17-17-11-02_426849468.json", std::ifstream::binary);
    Json::Value demo_data; 
    demo_file >> demo_data;
    cout << demo_data[0];

//    PythonInterface python;
//    double myNum[8] = {10, 20, 30, 1, 0, 3,4,5}; // input quaternion
//    Eigen::Vector4d pos(myNum[5],myNum[6],myNum[7],1.0);
//
//    Eigen::MatrixXd m(4,4);
//
//    double theta = acos(myNum[0]); // CHECK - see function in script at page 31 - Rotations Quaternion
//    double cos_t = cos(theta);
//    double sin_t = sin(theta);
//    double vers_t = 1 - cos_t;
//    Eigen::Vector3d k_axis = {myNum[1],myNum[2],myNum[3]} ;
//    k_axis /= sin_t;
//
//    // CHECK - see function in script at page 28 - Transformation
//    m.col(0) = Eigen::Vector4d{k_axis[0]*k_axis[0]*vers_t + cos_t ,           k_axis[0]*k_axis[1]*vers_t + k_axis[2]* sin_t,  k_axis[0]*k_axis[2]*vers_t -
//                                                                                                                    k_axis[1]*sin_t, 0};
//    m.col(0) = Eigen::Vector4d{k_axis[0]*k_axis[1]*vers_t - k_axis[2]*sin_t , k_axis[1]*k_axis[1]*vers_t + cos_t,             k_axis[1]*k_axis[2]*vers_t +
//                                                                                                                    k_axis[0]*sin_t, 0};
//    m.col(0) = Eigen::Vector4d{k_axis[0]*k_axis[2]*vers_t + k_axis[1]*sin_t , k_axis[1]*k_axis[2]*vers_t - k_axis[1] * sin_t, k_axis[2]*k_axis[2]*vers_t +
//                                                                                                                   cos_t,             0};
//    m.col(3) = pos;
//
//    python.reInitialize("scripts.plotting", {"plot_coord"});
//    python.callFunction("plot_coord", m);
//
    return 0;
}
