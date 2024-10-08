#include <Eigen/Geometry>
#include <iostream>
// 默认构造未初始化的四元数：展示如何使用默认构造函数。
// 通过标量构造四元数：直接使用 w, x, y, z 初始化四元数。
// 从数组构造四元数：从一个数组中提取四元数的四个分量。
// 使用角轴表示构造四元数：通过指定的旋转轴和旋转角度生成四元数。
// 从旋转矩阵构造四元数：通过 3x3 旋转矩阵生成四元数。
// 从两个向量生成四元数：使用 FromTwoVectors 方法计算从向量 v1 到 v2 的旋转。
// Slerp 插值：在两个四元数之间进行球面线性插值。
// 归一化四元数：将四元数归一化，使其表示合法的旋转。
// 旋转向量：使用四元数旋转一个三维向量。
// 四元数转换为旋转矩阵：将四元数转换为 3x3 旋转矩阵。
// 四元数转换为角轴表示：将四元数转换为角度和旋转轴。
// 生成随机四元数：生成一个随机的单位四元数，用于随机旋转。

int main()
{
    // 1. 默认构造未初始化的四元数
    Eigen::Quaternionf quat1;
    std::cout << "Uninitialized quaternion (quat1): "
              << quat1.coeffs().transpose() << std::endl;

    // 2. 使用 w, x, y, z 构造四元数
    Eigen::Quaternionf quat2(1.0, 0.0, 1.0, 0.0);  // 实部w, 虚部(x, y, z)
    std::cout << "Quaternion constructed with x,y,z,w (quat2): "
              << quat2.coeffs().transpose() << std::endl;

    // 3. 从四个元素数组构造四元数
    float data[] = {0.0, 1.0, 0.0, 1.0};  // {x, y, z, w}
    Eigen::Quaternionf quat3(data);
    std::cout << "Quaternion from array (quat3): " << quat3.coeffs().transpose()
              << std::endl;

    // 4. 使用角轴表示法构造四元数
    Eigen::AngleAxisf angleAxis(M_PI / 4,
                                Eigen::Vector3f(0, 0, 1));  // 绕 Z 轴旋转 45 度
    Eigen::Quaternionf quat4(angleAxis);
    std::cout << "Quaternion from Angle-Axis (quat4): "
              << quat4.coeffs().transpose() << std::endl;

    // 5. 从旋转矩阵构造四元数
    Eigen::Matrix3f rotationMatrix;
    rotationMatrix = Eigen::AngleAxisf(
        M_PI / 4, Eigen::Vector3f(0, 1, 0));  // 绕 Y 轴旋转 45 度
    Eigen::Quaternionf quat5(rotationMatrix);
    std::cout << "Quaternion from rotation matrix (quat5): "
              << quat5.coeffs().transpose() << std::endl;

    // 6. 使用 FromTwoVectors 计算从 v1 到 v2 的旋转
    Eigen::Vector3f v1(1, 0, 0);  // 初始向量
    Eigen::Vector3f v2(0, 1, 0);  // 目标向量
    Eigen::Quaternionf quat6 = Eigen::Quaternionf::FromTwoVectors(v1, v2);
    std::cout << "Quaternion from two vectors (quat6): "
              << quat6.coeffs().transpose() << std::endl;

    // 7. 使用 Slerp 进行球面线性插值
    Eigen::Quaternionf quat7 =
        quat2.slerp(0.5, quat5);  // 在 quat2 和 quat5 之间插值
    std::cout << "Slerp interpolated quaternion (quat7): "
              << quat7.coeffs().transpose() << std::endl;

    // 8. 四元数归一化
    quat2.normalize();
    std::cout << "Normalized quat2: " << quat2.coeffs().transpose()
              << std::endl;

    // 9. 四元数与向量旋转
    Eigen::Vector3f rotated_v1 = quat6 * v1;  // 将 v1 旋转到与 v2 对齐
    std::cout << "v1 rotated by quat6: " << rotated_v1.transpose() << std::endl;

    // 10. 将四元数转换为旋转矩阵
    Eigen::Matrix3f rotMatFromQuat = quat6.toRotationMatrix();
    std::cout << "Rotation matrix from quat6:\n" << rotMatFromQuat << std::endl;

    // 11. 将四元数转换为角轴表示
    Eigen::AngleAxisf angleAxisFromQuat(quat6);
    float angle          = angleAxisFromQuat.angle();
    Eigen::Vector3f axis = angleAxisFromQuat.axis();
    std::cout << "Angle-Axis from quat6: angle = " << angle
              << ", axis = " << axis.transpose() << std::endl;

    // 12. 使用随机单位四元数
    Eigen::Quaternionf randomQuat = Eigen::Quaternionf::UnitRandom();
    std::cout << "Random unit quaternion: " << randomQuat.coeffs().transpose()
              << std::endl;

    return 0;
}
