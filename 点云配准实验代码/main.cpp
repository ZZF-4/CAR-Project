#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>

#include "optimized_ICP_GN.h"

using namespace std;

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_opti_transformed_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_svd_transformed_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile("../data/room_scan2.pcd", *cloud_target_ptr);
    pcl::io::loadPCDFile("../data/room_scan1.pcd", *cloud_source_ptr);

    Eigen::Matrix4f T_predict, T_final;
    T_predict.setIdentity();
    T_predict << 0.765, 0.643, -0.027, -1.472,
        -0.6, 0.765, -0.023, 1.3,
        0.006, 0.035, 0.999, -0.1,
        0, 0, 0, 1;

    std::cout << "Wait, matching..." << std::endl;

    // =======================   optimized icp   =======================
    OptimizedICPGN optimized_icp;
    optimized_icp.SetTargetCloud(cloud_target_ptr);
    optimized_icp.SetMaxIterations(30);
    optimized_icp.SetMaxCorrespondDistance(0.3);
    optimized_icp.SetTransformationEpsilon(1e-4);
    bool success = optimized_icp.Match(cloud_source_ptr, T_predict,
                                       cloud_source_opti_transformed_ptr, T_final);
    if (success)
    {
        std::cout << "\n============== Optimized ICP =================" << std::endl;
        std::cout << "T final: \n"
                  << T_final << std::endl;
        std::cout << "fitness score: " << optimized_icp.GetFitnessScore() << std::endl;
    }
    else
    {
        std::cout << "Optimized ICP matching failed." << std::endl;
    }

    // =======================   optimized icp   =======================

    // =======================   svd icp   =======================
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_svd; //声明了一个ICP配准对象icp_svd
    icp_svd.setInputTarget(cloud_target_ptr); //包含x、y、z坐标
    icp_svd.setInputSource(cloud_source_ptr);//设置ICP算法的目标点云，进行变换以匹配目标点云的数据
    icp_svd.setMaxCorrespondenceDistance(0.3);//设置最大对应距离
    icp_svd.setMaximumIterations(30);//设置为30次迭代
    icp_svd.setEuclideanFitnessEpsilon(1e-4);//设置欧几里得拟合度阈值
    icp_svd.setTransformationEpsilon(1e-4);//设置变换矩阵的变化阈值
    icp_svd.align(*cloud_source_svd_transformed_ptr, T_predict);//将结果存储在cloud_source_svd_transformed_ptr指向的点云中
    std::cout << "\n============== SVD ICP =================" << std::endl;
    std::cout << "T final: \n"//控制台输出一行文本，表示ICP配准过程的开始
              << icp_svd.getFinalTransformation() << std::endl;//输出ICP算法最终计算得到的变换矩阵
    std::cout << "fitness score: " << icp_svd.getFitnessScore() << std::endl;//输出ICP算法的拟合度分数
    // =======================   svd icp   =======================

    // 可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->initCameraParameters();

    int v1(0);
    int v2(1);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("Optimized ICP", 10, 10, "optimized icp", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_opti_color(
        cloud_source_opti_transformed_ptr, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_source_opti_transformed_ptr, source_opti_color, "source opti cloud", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color_0(cloud_target_ptr, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_target_ptr, target_color_0, "target cloud1", v1);

    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.0, 0.0, 0.0, v2);
    viewer->addText("SVD ICP", 10, 10, "svd icp", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color_1(cloud_target_ptr, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_target_ptr, target_color_1, "target cloud2", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_svd_color(cloud_source_svd_transformed_ptr,
                                                                                     0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_source_svd_transformed_ptr, source_svd_color, "source svd cloud", v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source opti cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source svd cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target cloud2");
    viewer->addCoordinateSystem(1.0);

    viewer->setCameraPosition(0, 0, 20, 0, 10, 10, v1);
    viewer->setCameraPosition(0, 0, 20, 0, 10, 10, v2);

    viewer->spin();

    return 0;
}