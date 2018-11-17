#include <iostream>
#include <Eigen/Eigen>

using namespace eigen;
using namespace std;

int main(int argc, char* argv[]){
    VectorXd m;
    m << 0, 1, 2, 3;

    cout << m[0];
}
