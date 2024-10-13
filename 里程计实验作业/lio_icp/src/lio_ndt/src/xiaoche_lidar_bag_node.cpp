#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>
#include <glog/logging.h>

// 定义全局的ofstream对象来保存数据
std::ofstream laser_file;

// 回调函数处理激光雷达数据
void laserCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 将PointCloud2消息转换为pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // 遍历点云数据，提取x, y, z坐标
    for (const auto& point : cloud) {
        laser_file << point.x << " " << point.y << " " << point.z << " "
                   << 0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << " "
                   << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << std::endl;
    }
}

int main(int argc, char *argv[]) {
    // 初始化ros节点与句柄
    ros::init(argc, argv, "pointcloud_subscriber_node");
    ros::NodeHandle nh;

    // 打开文件以保存数据
    laser_file.open("./src/lio_ndt/src/xx.txt");

    // 检查文件是否成功打开
    if (!laser_file.is_open()) {
        ROS_ERROR("Failed to open laser data file.");
        return -1;
    }

    // 订阅激光雷达数据
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::PointCloud2>("rslidar_points", 1000, laserCallback);

    ros::spin(); // 进入自循环，等待回调函数被调用

    // 关闭文件
    laser_file.close();

    return 0;
}
