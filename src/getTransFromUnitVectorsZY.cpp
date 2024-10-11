#include <iostream>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>

int main()
{
    // 定义目标 Z 轴方向和 Y 轴方向
    Eigen::Vector3f link_line(0, 0, 1);  // Z 轴方向
    Eigen::Vector3f cross(0, 1, 0);      // Y 轴方向

    // 定义一个仿射变换矩阵来存储结果
    Eigen::Affine3f transform;

    // 调用 getTransFromUnitVectorsZY 函数
    pcl::getTransFromUnitVectorsZY(link_line, cross, transform);

    // 输出生成的仿射变换矩阵
    std::cout << "Generated Affine Transformation Matrix:\n"
              << transform.matrix() << std::endl;

    return 0;
}
