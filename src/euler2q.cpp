#include <Eigen/Geometry>  // 包含Eigen中的四元数类和矩阵运算
#include <iostream>

// 封装欧拉角 (roll, pitch, yaw) 转换为四元数的函数
Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw)
{
    // 创建Eigen中的角度轴序列，顺序为ZYX，即先绕Z轴，再绕Y轴，最后绕X轴
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());  // 绕X轴的旋转
    Eigen::AngleAxisd pitchAngle(pitch,
                                 Eigen::Vector3d::UnitY());  // 绕Y轴的旋转
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());  // 绕Z轴的旋转

    // 通过乘法来得到总的旋转，四元数顺序为 ZYX
    Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
    return quaternion;
}

int main()
{
    // 示例欧拉角（单位：弧度）
    double rx = 0.0096;   // Roll (绕X轴)
    double ry = -0.0569;  // Pitch (绕Y轴)
    double rz = 0.0045;   // Yaw (绕Z轴)

    // 调用封装好的函数，将欧拉角转换为四元数
    Eigen::Quaterniond quaternion = eulerToQuaternion(rx, ry, rz);

    // 打印结果
    std::cout << "四元数 (q_w, q_x, q_y, q_z): " << quaternion.w() << ", "
              << quaternion.x() << ", " << quaternion.y() << ", "
              << quaternion.z() << std::endl;

    return 0;
}
