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
#include <AndreiUtils/classes/DualQuaternion.hpp>


using json = nlohmann::json;
using namespace AndreiUtils;
using namespace std;
using namespace Eigen;


int main(){
    cout << "Hello World!" << endl;
    json data = readJsonFile("../data/demonstration_2023-05-19-16-25-09_343253856.json");
    // TODO: fill address -
    json surface = readJsonFile("../data/surfaceData_2023-06-21-11-41-19_799229444.json");

    Eigen::Matrix3d A;

    Posed q;
    for (int i = 0; i < min(surface.size(), data.size()) - 1; ++i) {

        vector<double> src = data[i]["objects"]["BowlGreyIkeaInstance"]["geometryPose"];
        q.fromCoefficients(src);
        auto t = q.getTranslation();

        // extracting the axes of the surface -
        // Taking data of surface number 0 -
        vector<double> x_axis = surface[i]["surfacesData"][0]["xAxis"];
        vector<double> y_axis = surface[i]["surfacesData"][0]["yAxis"];
        vector<double> z_axis = surface[i]["surfacesData"][0]["zAxis"];
        vector<double> x_range = surface[i]["surfacesData"][0]["xRange"];
        vector<double> y_range = surface[i]["surfacesData"][0]["yRange"];
        
        A.col(0) = Vector3d(x_axis.data());
        A.col(1) = Vector3d(y_axis.data());
        A.col(2) = Vector3d(z_axis.data());

        // check!
        Eigen::Vector3d newCoordinates =  Eigen::Vector3d(t.x(), t.y(), t.z()).transpose() * A;


        if (newCoordinates(0) > x_range[0] && newCoordinates(0) < x_range[1] &&
            newCoordinates(1) > y_range[0] && newCoordinates(1) < y_range[1] &&
            newCoordinates(2) < 0.15 ) {
                
                // z coordinate is max 15 cm away from the surface
                cout << "Object is in surface limits"<<endl;
            }
        else
            cout<<"Outside bounds!"<<endl;
        

    }

    return 0;
}