#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <lio_ndt/subscriber/gnss_subscriber.hpp>
#include <glog/logging.h>

// 定义全局的ofstream对象来保存数据
std::ofstream laser_odom_file;
std::ofstream gnss_file;

// 回调函数处理laser_odom数据
void laserOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    laser_odom_file << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.position.z << " "
                     << msg->pose.pose.orientation.x << " " << msg->pose.pose.orientation.y << " " << msg->pose.pose.orientation.z << " "
                     << msg->pose.pose.orientation.w << " "
                     << msg->twist.twist.linear.x << " " << msg->twist.twist.linear.y << " " << msg->twist.twist.linear.z << " "
                     << msg->twist.twist.angular.x << " " << msg->twist.twist.angular.y <<std::endl; // " " << msg->twist.twist.angular.z << 
}


// 回调函数处理gnss数据
//void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
//    gnss_file << msg->latitude << " " << msg->longitude << " " << msg->altitude << std::endl;
//}
void gnssCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    gnss_file << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.position.z << " "
                     << msg->pose.pose.orientation.x << " " << msg->pose.pose.orientation.y << " " << msg->pose.pose.orientation.z << " "
                     << msg->pose.pose.orientation.w << " "
                     << msg->twist.twist.linear.x << " " << msg->twist.twist.linear.y << " " << msg->twist.twist.linear.z << " "
                     << msg->twist.twist.angular.x << " " << msg->twist.twist.angular.y << " " << msg->twist.twist.angular.z << std::endl;
}

int main(int argc, char *argv[]) {
    //google::InitGoogleLogging(argv[0]); // 初始化日志函数
    //FLAGS_log_dir = WORK_SPACE_PATH + "/Log"; // 设置日志输出目录
    //FLAGS_alsologtostderr = 1; // 是否将所有日志输出到文件

    // 初始化ros节点与句柄
    ros::init(argc, argv, "odometry_subscriber_node");
    ros::NodeHandle nh;

    // 打开文件以保存数据
    laser_odom_file.open("./src/lio_ndt/src/laser_odom_data.txt");
    gnss_file.open("./src/lio_ndt/src/gnss_data.txt");

    // 检查文件是否成功打开
    if (!laser_odom_file.is_open() || !gnss_file.is_open()) {
        ROS_ERROR("Failed to open odometry data files.");
        return -1;
    }

    // 订阅激光里程计和GNSS里程计数据
    ros::Subscriber laser_odom_sub = nh.subscribe<nav_msgs::Odometry>("laser_odom", 1000, laserOdomCallback);
    //ros::Subscriber gnss_sub = nh.subscribe<sensor_msgs::NavSatFix>("gnss", 100, gnssCallback);
    ros::Subscriber gnss_sub = nh.subscribe<nav_msgs::Odometry>("gnss", 1000, gnssCallback);

    ros::spin(); // 进入自循环，等待回调函数被调用

    // 关闭文件
    laser_odom_file.close();
    gnss_file.close();

    return 0;
}
