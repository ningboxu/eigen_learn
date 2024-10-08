#include <iostream>
#include <Eigen/Dense>

//using Eigen::MatrixXd;
//using Eigen::VectorXd;

using namespace std;
using namespace Eigen;

int main()
{
    MatrixXd m = MatrixXd::Random(3, 3);
    m = (m + MatrixXd::Constant(3, 3, 1.2)) * 50;
    std::cout << "m =" << std::endl << m << std::endl;
    VectorXd v(3);
    v << 1, 2, 3;   // 逗号初始化
    std::cout << "m * v =" << std::endl << m * v << std::endl;

    // 获取元素:
    // 通过中括号获取元素，对于矩阵是：（行，列）；
    // 对于向量，只是传递它的索引，以0为起始。
    cout << m(0, 0) << endl;
    cout << v(0) << endl;
    // m(index)也可以用于获取矩阵元素，但取决于matrix的存储顺序，
    // 默认是按列存储的，当然也可以改为按行。
    cout << m(0) << endl;
    //cout << m[0] << endl;
    cout << v[0] << endl;

    // matrix的大小可以通过rows()、cols()、size()获取，
    cout << "m.rows(): " << m.rows() << endl;
    cout << "m.cols(): " << m.cols() << endl;
    cout << "m.size(): " << m.size() << endl;

    // resize()可以重新调整动态matrix的大小
    m.resize(4, 4);
    cout << "m resize: " << m.size() << endl;
    
    Vector3i v1;
    v1 << 1, 2, 3;
    VectorXi v2(9);
    v2 << 3, 6, 9, 3, 5, 2, 4, 1, 10;
    RowVector2d v3;
    v3 << 3.6, 6.9;
    // v3 << 3.6, 6.9, 2.5;
}