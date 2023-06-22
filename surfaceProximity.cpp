//
// Created by chaitanya11 on 16.06.23.
//

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <algorithm>

#include <iterator>
#include <AndreiUtils/utilsJson.h>
#include <AndreiUtils/utilsJsonEigen.hpp>
#include <AndreiUtils/classes/DualQuaternion.hpp>


using json = nlohmann::json;
using namespace AndreiUtils;
using namespace std;
using namespace Eigen;

// set here the index number of surface (starts from 0)-
int const surface_index = 1;

int main() {
    cout << "Hello World!" << endl;
    vector<json> data = readJsonFile("../data/demonstration_2023-06-21-18-53-52_129873292.json").get<vector<json>>();
    cout << "Json data size = " << data.size() << endl;
    // TODO: fill address -
    vector<json> surface = readJsonFile("../data/surfaceData_2023-06-21-18-56-48_899605557.json").get<vector<jsin>>();

    Eigen::Matrix3d A;

    Posed q;
    
    // extracting the axes of the surface -
    // Taking data of surface number 0 -
    vector<double> x_axis = surface[20].at("surfacesData")[surface_index].at("xAxis");
    vector<double> y_axis = surface[20].at("surfacesData")[surface_index].at("yAxis");
    vector<double> z_axis = surface[20].at("surfacesData")[surface_index].at("zAxis");
    vector<double> x_range = surface[20].at("surfacesData")[surface_index].at("xRange");
    vector<double> y_range = surface[20].at("surfacesData")[surface_index].at("yRange");


    for (int i = 0; i < min(surface.size(), data.size()) - 2; ++i) {
        if (!data[i].contains("objects")) {
            continue;
        }
        auto &objects = data[i].at("objects");
        if (!objects.contains("MilkCartonLidlInstance1")) {
            continue;
        }
        auto &milkCarton = data[i].at("objects").at("MilkCartonLidlInstance1");
        q = milkCarton.at("geometryPose").get<Posed>();
        cout << q << endl;

        auto t = q.getTranslation();

        A.col(0) = Vector3d(x_axis.data());
        A.col(1) = Vector3d(y_axis.data());
        A.col(2) = Vector3d(z_axis.data());

        // check!
        Eigen::Vector3d newCoordinates = A * Eigen::Vector3d(t.x(), t.y(), t.z());

        try {
            if (newCoordinates(0) > x_range[0] && newCoordinates(0) < x_range[1] &&
                newCoordinates(1) > y_range[0] && newCoordinates(1) < y_range[1] &&
                newCoordinates(2) < 0.10) {

                // z coordinate is max 15 cm away from the surface
                cout << "Object is in surface limits" << endl;
            } else
                cout << "Outside bounds!" << endl;
        }

        catch (const exception &e) {

        }


    }

    return 0;
}