#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>  // 确保包含PCL点类型的头文件
#include <pcl/common/transforms.h>  // 包含点云变换的函数，如transformPointCloud
#include <Eigen/Geometry>  // Eigen库的四元数功能

int main()
{
    // 1. 使用四个标量构造四元数
    Eigen::Quaternionf quat1(
        1.0, 0.0, 1.0, 0.0);  // 实部 w = 1.0, 虚部 x = 0.0, y = 1.0, z = 0.0
    std::cout << "Quaternion1 (w, x, y, z): " << quat1.coeffs().transpose()
              << std::endl;

    // 2. 从旋转矩阵构造四元数
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix =
        Eigen::AngleAxisf(M_PI / 4, Eigen::Vector3f::UnitZ());  // 绕Z轴旋转45度
    Eigen::Quaternionf quat2(rotation_matrix);  // 从旋转矩阵生成四元数
    std::cout << "Quaternion2 from rotation matrix: "
              << quat2.coeffs().transpose() << std::endl;

    // 3. 将四元数转换为旋转矩阵
    Eigen::Matrix3f rotation_matrix_from_quat = quat2.toRotationMatrix();
    std::cout << "Rotation matrix from Quaternion2:\n"
              << rotation_matrix_from_quat << std::endl;

    // 4. 使用角轴表示法构造四元数
    Eigen::AngleAxisf angleAxis(M_PI / 4,
                                Eigen::Vector3f(0, 1, 0));  // 绕Y轴旋转45度
    Eigen::Quaternionf quat3(angleAxis);
    std::cout << "Quaternion3 from Angle-Axis (quat3): "
              << quat3.coeffs().transpose() << std::endl;

    //! 5. 欧拉角与四元数转换
    Eigen::Vector3f euler_angles =
        quat3.toRotationMatrix().eulerAngles(0, 1, 2);  // XYZ顺序的欧拉角
    std::cout << "Euler angles from Quaternion3: " << euler_angles.transpose()
              << std::endl;

    // 6. 归一化四元数
    quat1.normalize();
    std::cout << "Normalized Quaternion1: " << quat1.coeffs().transpose()
              << std::endl;

    //! 7. 使用四元数旋转向量
    Eigen::Vector3f v1(1, 0, 0);              // 初始向量
    Eigen::Vector3f rotated_v1 = quat3 * v1;  // 旋转向量 v1
    std::cout << "Rotated vector v1 by Quaternion3: " << rotated_v1.transpose()
              << std::endl;

    //! 8. 使用四元数旋转点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(1.0, 0.0, 0.0));  // 添加一个点

    // 定义变换矩阵
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(quat3);  // 将四元数旋转应用到变换矩阵
    transform.translation() << 1.0, 2.0, 3.0;  // 平移
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*cloud, transformed_cloud, transform);  // 旋转点云

    std::cout << "Original point: (" << cloud->points[0].x << ", "
              << cloud->points[0].y << ", " << cloud->points[0].z << ")"
              << std::endl;
    std::cout << "Transformed point: (" << transformed_cloud.points[0].x << ", "
              << transformed_cloud.points[0].y << ", "
              << transformed_cloud.points[0].z << ")" << std::endl;

    return 0;
}
