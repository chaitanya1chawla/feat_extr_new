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


    Posed q;
    vector<double> src2 = data[0][10]["objects"]["BowlGreyIkeaInstance"]["geometryPose"];
    q.fromCoefficients(src2);
    auto t2 = q.getTranslation();


    return 0;
}