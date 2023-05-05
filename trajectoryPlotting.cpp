//
// Created by chaitanya11 on 05.05.23.
//

#include <AndreiUtils/classes/PythonInterface.h>
#include <iostream>

using namespace AndreiUtils;
using namespace std;

int main() {
    cout << "Hello World!" << endl;

    PythonInterface python;
    python.reInitialize("scripts.plotting", {"print_greeting"});
    python.callFunction("print_greeting");



    return 0;
}
