#include <iostream>
#include <Eigen/Geometry>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

int main()
{
    // 定义两个旋转四元数
    Eigen::Quaternionf q1(Eigen::AngleAxisf(
        M_PI / 4, Eigen::Vector3f::UnitX()));  // 绕X轴旋转45度
    Eigen::Quaternionf q2(Eigen::AngleAxisf(
        M_PI / 6, Eigen::Vector3f::UnitY()));  // 绕Y轴旋转30度

    // 复合两个旋转
    Eigen::Quaternionf q_combined = q1 * q2;

    // 定义旋转前的向量
    Eigen::Vector3f v(1, 0, 0);  // 沿X轴的向量

    // 使用复合四元数进行旋转
    Eigen::Vector3f v_rotated = q_combined * v;

    std::cout << "Original vector: " << v.transpose() << std::endl;
    std::cout << "Rotated vector: " << v_rotated.transpose() << std::endl;

    // 创建PCL可视化工具
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer(
            "Quaternion Rotation Visualization"));

    viewer->setBackgroundColor(0, 0, 0);  // 设置背景颜色为黑色

    // 绘制原始向量 (从原点到v)
    pcl::PointXYZ origin(0, 0, 0);
    pcl::PointXYZ original_vector(v.x(), v.y(), v.z());
    viewer->addArrow(original_vector, origin, 1.0, 0.0, 0.0, false,
                     "original vector");  // 红色箭头表示原始向量

    // 绘制旋转后的向量 (从原点到 v_rotated)
    pcl::PointXYZ rotated_vector(v_rotated.x(), v_rotated.y(), v_rotated.z());
    viewer->addArrow(rotated_vector, origin, 0.0, 1.0, 0.0, false,
                     "rotated vector");  // 绿色箭头表示旋转后的向量

    // 添加坐标系
    viewer->addCoordinateSystem(1.0);  // 长度为1.0的坐标系

    // 绘制旋转轴X（红色）
    pcl::PointXYZ axis_x_start(0, 0, 0);
    pcl::PointXYZ axis_x_end(2, 0, 0);  // 绘制X轴方向
    viewer->addArrow(axis_x_end, axis_x_start, 1.0, 0.0, 0.0, false, "axis_x");

    // 绘制旋转轴Y（绿色）
    pcl::PointXYZ axis_y_start(0, 0, 0);
    pcl::PointXYZ axis_y_end(0, 2, 0);  // 绘制Y轴方向
    viewer->addArrow(axis_y_end, axis_y_start, 0.0, 1.0, 0.0, false, "axis_y");

    // 添加点云并展示旋转后的效果
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(1, 0, 0));  // 初始点云
    cloud->push_back(pcl::PointXYZ(0, 1, 0));
    cloud->push_back(pcl::PointXYZ(0, 0, 1));

    // 对点云应用旋转
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(q_combined);  // 使用复合四元数旋转点云
    pcl::transformPointCloud(*cloud, transformed_cloud, transform);

    // 在可视化中添加原始点云（蓝色）
    viewer->addPointCloud(cloud, "original cloud");

    // 在可视化中添加旋转后的点云（黄色）
    viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud.makeShared(),
                                         "rotated cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0,
        "rotated cloud");  // 黄色

    // 启动可视化
    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return 0;
}
