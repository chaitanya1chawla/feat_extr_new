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

void trajectoryPlottingChaitanya() {
    json data = readJsonFile("../data/demonstration_2023-05-19-16-23-07_771535671.json");
    vector<Matrix<double, 4, 4>> mm;
    Posed q;

    for (int i = 0; i < data.size(); i++) {

        vector<double> src = data[i]["objects"]["MilkCartonLidlInstance1"]["geometryPose"];
        q.fromCoefficients(src);
        mm.push_back(q.getTransformationMatrix());
    }
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
    cout << "norms" << endl;
    python.reInitialize("scripts.plotting", {"plot_coord", "trafo_mat", "print_greeting", "plot_show"});
    python.callFunction("print_greeting");
    python.callFunction("plot_coord");

    python.callFunction("trafo_mat", mm);
    /*
    for(int i = 0; i <data.size(); i ++) {

        vector<double> src = data[i]["objects"]["MilkCartonLidlInstance1"]["geometryPose"];
        q.fromCoefficients(src);
        MatrixXd m(4,4);
        m = q.getTransformationMatrix();
        cout<<m<<endl;
        python.callFunction("trafo_mat", m);
    }
    python.callFunction("plot_show");
*/
}

void trajectoryPlottingAndrei() {
    json data = readJsonFile("../data/demonstration_2023-05-19-16-23-07_771535671.json");
    Posed q;

    cout << "Data points = " << data.size() << endl;
    int eigenPoints = ceil(data.size() / 1.);
    cout << "Reduce to " << eigenPoints << endl;
    Eigen::MatrixX3d origins = MatrixX3d::Zero(eigenPoints, 3);
    Eigen::MatrixX3d orientations = MatrixX3d::Zero(3 * eigenPoints, 3);

    for (int i = 0; i < data.size(); i += 2) {
        vector<double> src = data[i]["objects"]["MilkCartonLidlInstance1"]["geometryPose"];
        q.fromCoefficients(src);

        int eigenIndex = i / 2;
        origins.row(eigenIndex) = q.getTranslation();
        auto const &r = q.getRotationAsMatrix();
        orientations.row(3 * eigenIndex) = r.col(0);
        orientations.row(3 * eigenIndex + 1) = r.col(1);
        orientations.row(3 * eigenIndex + 2) = r.col(2);
    }

    PythonInterface python;
    python.reInitialize("scripts.plot_trajectory", {"plot_trajectory"});
    python.getFunctions()["plot_trajectory"](origins, orientations, 0.5, 1, 0.02, false);
}

int main() {
    cout << "Hello World!" << endl;

    trajectoryPlottingAndrei();

    return 0;
}
