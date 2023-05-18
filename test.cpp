#include <iostream>

using namespace std;

class left {
public:
    void foo(){
        cout<<"right";
    };
};

class right {
public:
    void foo(){
        cout<<"left";
    };
};

class bottom : public left, public right {
public:
    void foo()
    {

        // and when foo() is not called for 'this':

    }
};

int main(){
    bottom b;
    b.left::foo();  // calls b.foo() from 'left'
    b.right::foo();  // call b.foo() from 'right'
}