//
// Created by chaitanya11 on 05.05.23.
//

#include <AndreiUtils/classes/PythonInterface.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <pybind11/eigen.h>

using json = nlohmann::json;
using namespace AndreiUtils;
using namespace std;
using namespace Eigen;


int main() {
    cout << "Hello World!" << endl;

    ifstream demo_file("../data/demonstration_2023-05-08-13-32-14_980300108.json");
//    ifstream demo_file("../data/sample.json");

    json data = json::parse(demo_file);

    vector<double> src = data[0]["objects"]["CerealBoxKelloggsMuslixInstance"]["geometryPose"];

    PythonInterface python;
    Vector4d pos(src[5],src[6],src[7],1.0);
    cout << pos<<endl;
    MatrixXd m(4,4);

    double theta = acos(src[0]); // CHECK - see function in script at page 31 - Rotations Quaternion
    double cos_t = cos(theta);
    double sin_t = sin(theta);
    double vers_t = 1 - cos_t;
    Eigen::Vector3d k_axis = {src[1],src[2],src[3]} ;
    k_axis /= sin_t;

    // CHECK - see function in script at page 28 - Transformation
    m.col(0) = Eigen::Vector4d{k_axis[0]*k_axis[0]*vers_t + cos_t ,           k_axis[0]*k_axis[1]*vers_t + k_axis[2]* sin_t,  k_axis[0]*k_axis[2]*vers_t -
                                                                                                                    k_axis[1]*sin_t, 0};
    m.col(0) = Eigen::Vector4d{k_axis[0]*k_axis[1]*vers_t - k_axis[2]*sin_t , k_axis[1]*k_axis[1]*vers_t + cos_t,             k_axis[1]*k_axis[2]*vers_t +
                                                                                                                    k_axis[0]*sin_t, 0};
    m.col(0) = Eigen::Vector4d{k_axis[0]*k_axis[2]*vers_t + k_axis[1]*sin_t , k_axis[1]*k_axis[2]*vers_t - k_axis[1] * sin_t, k_axis[2]*k_axis[2]*vers_t +
                                                                                                                   cos_t,             0};
    m.col(3) = pos;

    python.reInitialize("scripts.plotting", {"print_greeting"});
    python.callFunction("print_greeting", m);

    return 0;
}
