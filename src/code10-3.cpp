#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

int main() {
    MatrixXf m = MatrixXf::Random(4,4);
    cout << "m: " << endl << m << endl;

    // m.block<i,j> (a,b) 表示从第a行b列开始,截取i行,j列
    cout << "Block in the middle" << endl;
    cout << m.block<2, 2>(1, 1) << endl << endl;

    for (int i = 1; i <= 3; ++i)
    {
        cout << "Block of size " << i << "x" << i << endl;
        //m.block(a,b,i,j) 表示从第a行b列开始,截取i行,j列
        cout << m.block(0, 0, i, i) << endl << endl;
    }
}