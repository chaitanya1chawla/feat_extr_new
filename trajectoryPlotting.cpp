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
#include <AndreiUtils/classes/DualQuaternion.hpp>
#include <AndreiUtils/utilsJson.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

using json = nlohmann::json;
using namespace AndreiUtils;
using namespace std;
using namespace Eigen;


int main() {
    cout << "Hello World!" << endl;

    json data = readJsonFile("../data/demonstration_2023-05-19-16-23-07_771535671.json");
    vector<Matrix<double,4,4>> mm;
    Posed q;

//    for(int i = 0; i <data.size(); i ++){
//
//        vector<double> src = data[i]["objects"]["CerealBoxKelloggsMuslixInstance"]["geometryPose"];
//        q.fromCoefficients(src);
//        mm.push_back(q.getTransformationMatrix());
//    }
//    Posed q;
//    q.fromCoefficients(src);
//    auto t  = q.getTranslation();

    PythonInterface python;
/*
    Vector4d pos(t.x(),t.y(),t.z(),1.0);
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
    m.col(1) = Eigen::Vector4d{k_axis[0]*k_axis[1]*vers_t - k_axis[2]*sin_t , k_axis[1]*k_axis[1]*vers_t + cos_t,             k_axis[1]*k_axis[2]*vers_t +
                                                                                                                    k_axis[0]*sin_t, 0};
    m.col(2) = Eigen::Vector4d{k_axis[0]*k_axis[2]*vers_t + k_axis[1]*sin_t , k_axis[1]*k_axis[2]*vers_t - k_axis[1] * sin_t, k_axis[2]*k_axis[2]*vers_t +
                                                                                                                   cos_t,             0};
    Eigen::AngleAxis<double> aa(1, Vector3d{1, 2, 3});
    aa.toRotationMatrix();
    m.col(3) = pos;

    m = q.getTransformationMatrix();
*/
    cout<<"norms";
    python.reInitialize("scripts.plotting", {"plot_coord", "trafo_mat", "print_greeting"});
    //python.callFunction("print_greeting");
    //python.callFunction("plot_coord");

    for(int i = 0; i <data.size(); i ++) {

        vector<double> src = data[i]["objects"]["MilkCartonLidlInstance1"]["geometryPose"];
        q.fromCoefficients(src);
        MatrixXd m(4,4);
        m = q.getTransformationMatrix();
        cout<<m;
        //python.callFunction("trafo_mat", m);
    }

    return 0;
}
