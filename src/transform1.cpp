#include <Eigen/Dense>
#include <Eigen/Geometry>  // For Eigen::Quaternion
#include <iostream>

// 欧拉角转换为四元数 (ZYX顺序)
Eigen::Quaternionf eulerToQuaternion(const Eigen::Vector3f& euler_angles)
{
    // ZYX顺序：先绕z轴旋转，再绕y轴，最后绕x轴
    Eigen::AngleAxisf rollAngle(euler_angles(0),
                                Eigen::Vector3f::UnitX());  // 绕x轴
    Eigen::AngleAxisf pitchAngle(euler_angles(1),
                                 Eigen::Vector3f::UnitY());  // 绕y轴
    Eigen::AngleAxisf yawAngle(euler_angles(2),
                               Eigen::Vector3f::UnitZ());  // 绕z轴

    // 四元数表示的旋转
    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    return q;
}

// 计算相机到基坐标系的变换矩阵
void computeCameraToBaseTransform(const Eigen::Vector3f& translation_CT,
                                  const Eigen::Vector3f& euler_CT,
                                  const Eigen::Vector3f& translation_TB,
                                  const Eigen::Quaternionf& quat_TB)
{
    // 相机到工具的旋转四元数 (欧拉角 -> 四元数)
    Eigen::Quaternionf quat_CT = eulerToQuaternion(euler_CT);

    // 相机到工具的变换矩阵
    Eigen::Matrix4f T_CT   = Eigen::Matrix4f::Identity();
    T_CT.block<3, 3>(0, 0) = quat_CT.toRotationMatrix();  // 旋转部分
    T_CT.block<3, 1>(0, 3) = translation_CT;              // 平移部分

    // 工具到基坐标系的变换矩阵
    Eigen::Matrix4f T_TB   = Eigen::Matrix4f::Identity();
    T_TB.block<3, 3>(0, 0) = quat_TB.toRotationMatrix();  // 旋转部分
    T_TB.block<3, 1>(0, 3) = translation_TB;              // 平移部分

    // 相机到基坐标系的变换
    Eigen::Matrix4f T_CB = T_TB * T_CT;

    // 提取相机到基坐标系的平移和旋转（四元数）
    Eigen::Vector3f translation_CB     = T_CB.block<3, 1>(0, 3);
    Eigen::Matrix3f rotation_CB_matrix = T_CB.block<3, 3>(0, 0);
    Eigen::Quaternionf quat_CB(rotation_CB_matrix);

    // 打印信息
    std::cout << "相机到工具的平移: " << translation_CT.transpose()
              << std::endl;
    std::cout << "相机到工具的旋转 (四元数): " << quat_CT.coeffs().transpose()
              << std::endl;

    std::cout << "工具到基坐标系的平移: " << translation_TB.transpose()
              << std::endl;
    std::cout << "工具到基坐标系的旋转 (四元数): "
              << quat_TB.coeffs().transpose() << std::endl;

    std::cout << "相机到基坐标系的平移: " << translation_CB.transpose()
              << std::endl;
    std::cout << "相机到基坐标系的旋转 (四元数): "
              << quat_CB.coeffs().transpose() << std::endl;
}

int main()
{
    // 相机到工具的变换 (平移和欧拉角)
    Eigen::Vector3f translation_CT(0.1, 0.2, 0.3);  // 相机到工具的平移
    Eigen::Vector3f euler_CT(0.5, 0.1,
                             0.2);  // 相机到工具的旋转 (ZYX顺序欧拉角)

    // 工具到基坐标系的变换 (平移和四元数)
    Eigen::Vector3f translation_TB(0.4, 0.5, 0.6);  // 工具到基坐标系的平移
    Eigen::Quaternionf quat_TB(0.923, 0.382, 0.0,
                               0.0);  // 工具到基坐标系的旋转 (四元数)

    // 计算相机到基坐标系的变换
    computeCameraToBaseTransform(translation_CT, euler_CT, translation_TB,
                                 quat_TB);

    return 0;
}
