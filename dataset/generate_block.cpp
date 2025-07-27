#include <ros/ros.h>
#include <Eigen/Dense>
#include <mutex>
#include <queue>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>
#include <csignal>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

std::string lidar_topic,odometry_topic,dataFolder,pointsFolder;

std::queue<nav_msgs::Odometry::ConstPtr> odom_buf;
std::queue<sensor_msgs::PointCloud2::ConstPtr> pcl_buf;
std::mutex bufMutex;

// CSV相关变量
std::ofstream csv_file;
std::string csv_filename;
int odom_counter = 0;
std::mutex csv_mutex;
double last_timestamp = -1.0;  // 记录上一次保存的时间戳

void pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    bufMutex.lock();
    // ROS_INFO("receive pcl");
    pcl_buf.push(msg);
    bufMutex.unlock();
}

// 新增：保存里程计数据到CSV文件的函数
void saveOdometryToCSV(const nav_msgs::Odometry::ConstPtr &msg)
{
    csv_mutex.lock();
    
    if (csv_file.is_open()) {
        double timestamp = msg->header.stamp.toSec();
        
        // 检查时间间隔，如果小于0.0001秒就跳过
        if (last_timestamp > 0 && (timestamp - last_timestamp) < 0.0001) {
            csv_mutex.unlock();
            return;
        }
        
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        
        // 写入CSV格式：num t x y z qx qy qz qw
        csv_file << odom_counter << "," 
                 << std::fixed << std::setprecision(9) << timestamp << ","
                 << std::fixed << std::setprecision(6) << x << ","
                 << std::fixed << std::setprecision(6) << y << ","
                 << std::fixed << std::setprecision(6) << z << ","
                 << std::fixed << std::setprecision(6) << qx << ","
                 << std::fixed << std::setprecision(6) << qy << ","
                 << std::fixed << std::setprecision(6) << qz << ","
                 << std::fixed << std::setprecision(6) << qw << std::endl;
        
        csv_file.flush(); // 确保数据及时写入
        last_timestamp = timestamp;  // 更新上一次保存的时间戳
        odom_counter++;
        
        if (odom_counter % 100 == 0) {
            ROS_INFO("Saved %d odometry messages to CSV", odom_counter);
        }
    }
    
    csv_mutex.unlock();
}

void odom_cbk(const nav_msgs::Odometry::ConstPtr &msg) 
{
    bufMutex.lock();
    // ROS_INFO("receive odometry");
    odom_buf.push(msg);
    bufMutex.unlock();
    
    // 同时保存到CSV文件
    saveOdometryToCSV(msg);
}

void savePoints(std::vector<nav_msgs::Odometry> currentOdoMsg,
 std::vector<sensor_msgs::PointCloud2> currentPclMsg)
{
    if(currentPclMsg.size() > 0 && currentOdoMsg.size() > 0 && currentPclMsg.size() == currentOdoMsg.size())
    {
        double currentTime = currentOdoMsg[0].header.stamp.toSec();
        char posFileName[256];
        char pclFileName[256];

        uint sec = currentTime;
        uint nsec = (currentTime-sec)*1e9;

        // must use the format of sec_nsec
        sprintf(pclFileName,"%s/%d_%d.pcd",pointsFolder.c_str(),sec,nsec);

        // save pcl
        pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        for (size_t i = 0; i < currentPclMsg.size(); i++)
        {
            Eigen::Quaterniond q;
            q.x() = currentOdoMsg[i].pose.pose.orientation.x;
            q.y() = currentOdoMsg[i].pose.pose.orientation.y;
            q.z() = currentOdoMsg[i].pose.pose.orientation.z;
            q.w() = currentOdoMsg[i].pose.pose.orientation.w;
            q.normalize();
            Eigen::Vector3d t(currentOdoMsg[i].pose.pose.position.x,currentOdoMsg[i].pose.pose.position.y,currentOdoMsg[i].pose.pose.position.z);

            pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_trans(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(currentPclMsg[i],*temp_cloud);

            Eigen::Matrix4f transform; transform.setIdentity();
            // transform.block<3,3>(0,0) = q.toRotationMatrix().cast<float>();
            // transform.block<3,1>(0,3) = t.cast<float>();
            
            pcl::transformPointCloud(*temp_cloud,*temp_cloud_trans,transform);
            //transform to ref
            *all_cloud += *temp_cloud_trans;
        }
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud (all_cloud);
        sor.setLeafSize (0.1f, 0.1f, 0.1f);
        pcl::PointCloud<pcl::PointXYZI>::Ptr down_sample(new pcl::PointCloud<pcl::PointXYZI>);
        sor.filter(*down_sample);
        pcl::io::savePCDFileBinary<pcl::PointXYZI>(pclFileName,*down_sample);
    }
    
}


void transformAndOutput(std::vector<nav_msgs::Odometry> currentOdoMsg,
 std::vector<sensor_msgs::PointCloud2> currentPclMsg)
{
    if(currentPclMsg.size() > 0 && currentOdoMsg.size() > 0 && currentPclMsg.size() == currentOdoMsg.size())
    {
        double currentTime = currentOdoMsg[0].header.stamp.toSec();
        char posFileName[256];
        char pclFileName[256];

        uint sec = currentTime;
        uint nsec = (currentTime-sec)*1e9;

        // must use the format of sec_nsec
        sprintf(posFileName,"%s/%d_%d.odom",dataFolder.c_str(),sec,nsec);
        sprintf(pclFileName,"%s/%d_%d.pcd",dataFolder.c_str(),sec,nsec);

        int id_ref = 0;

        Eigen::Quaterniond ref_q;
        Eigen::Vector3d ref_t(currentOdoMsg[id_ref].pose.pose.position.x,currentOdoMsg[id_ref].pose.pose.position.y,currentOdoMsg[id_ref].pose.pose.position.z);

        ref_q.x() = currentOdoMsg[id_ref].pose.pose.orientation.x;
        ref_q.y() = currentOdoMsg[id_ref].pose.pose.orientation.y;
        ref_q.z() = currentOdoMsg[id_ref].pose.pose.orientation.z;
        ref_q.w() = currentOdoMsg[id_ref].pose.pose.orientation.w;
  
        Eigen::Matrix3d rot = ref_q.toRotationMatrix();

        // save pose
        FILE *fp = fopen(posFileName,"w");
        fprintf(fp,"%f %f %f %f\n %f %f %f %f\n %f %f %f %f\n 0 0 0 1",
                    rot(0,0),rot(0,1),rot(0,2),ref_t(0),
                    rot(1,0),rot(1,1),rot(1,2),ref_t(1),
                    rot(2,0),rot(2,1),rot(2,2),ref_t(2));
        fclose(fp);

        // save pcl
        pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        for (size_t i = 0; i < currentPclMsg.size(); i++)
        {
            Eigen::Quaterniond q;
            q.x() = currentOdoMsg[i].pose.pose.orientation.x;
            q.y() = currentOdoMsg[i].pose.pose.orientation.y;
            q.z() = currentOdoMsg[i].pose.pose.orientation.z;
            q.w() = currentOdoMsg[i].pose.pose.orientation.w;
            q.normalize();
            Eigen::Vector3d t(currentOdoMsg[i].pose.pose.position.x,currentOdoMsg[i].pose.pose.position.y,currentOdoMsg[i].pose.pose.position.z);

            pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_trans(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(currentPclMsg[i],*temp_cloud);

            Eigen::Matrix4f transform; transform.setIdentity();
            transform.block<3,3>(0,0) = (ref_q.inverse()*q).toRotationMatrix().cast<float>();
            transform.block<3,1>(0,3) = (ref_q.inverse()*(-ref_t+t)).cast<float>();
            
            pcl::transformPointCloud(*temp_cloud,*temp_cloud_trans,transform);
            //transform to ref
            *all_cloud += *temp_cloud_trans;
        }
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud (all_cloud);
        sor.setLeafSize (0.2f, 0.2f, 0.2f);
        pcl::PointCloud<pcl::PointXYZI>::Ptr down_sample(new pcl::PointCloud<pcl::PointXYZI>);
        sor.filter(*down_sample);
        pcl::io::savePCDFileBinary<pcl::PointXYZI>(pclFileName,*down_sample);
    }
}

void process()
{
    ros::Rate rate(100);
    std::vector<nav_msgs::Odometry> currentOdoMsg;
    std::vector<sensor_msgs::PointCloud2> currentPclMsg;
    while (ros::ok()) {
        bufMutex.lock();///lock before access Buf

        if (!odom_buf.empty() && !pcl_buf.empty())
        {  
            currentOdoMsg.push_back(*odom_buf.front());
            odom_buf.pop();
            currentPclMsg.push_back(*pcl_buf.front());
            pcl_buf.pop();

            // next image
            if(currentOdoMsg.size() >= 100)
            {
                // save process
                // transformAndOutput(currentOdoMsg,currentPclMsg);
                savePoints(currentOdoMsg,currentPclMsg);
                currentOdoMsg.clear();
                currentPclMsg.clear();
            }
        }
        bufMutex.unlock();
        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generate_block");
    ros::NodeHandle nh;

    nh.param<std::string>("lidar_msg_name",lidar_topic, "/cloud_registered");
    nh.param<std::string>("odometry_msg_name",odometry_topic, "/Odometry"); 
    nh.param<std::string>("dataFolder",dataFolder, "/home/iot/workspace/data/frames");
    nh.param<std::string>("pointsFolder",pointsFolder, "/home/iot/workspace/data/points");

    if(!boost::filesystem::exists(dataFolder))
    {
        boost::filesystem::create_directories(dataFolder);
    }

    if(!boost::filesystem::exists(pointsFolder))
    {
        boost::filesystem::create_directories(pointsFolder);
    }
    
    // 创建CSV文件
    time_t rawtime;
    struct tm * timeinfo;
    char timestamp_str[80];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(timestamp_str, sizeof(timestamp_str), "%Y%m%d_%H%M%S", timeinfo);
    
    csv_filename = pointsFolder + "/odometry_" + std::string(timestamp_str) + ".csv";
    csv_file.open(csv_filename);
    
    if (csv_file.is_open()) {
        // 写入CSV头部
        csv_file << "num\tt\tx\ty\tz\tqx\tqy\tqz\tqw" << std::endl;
        ROS_INFO("Created CSV file: %s", csv_filename.c_str());
    } else {
        ROS_ERROR("Failed to create CSV file: %s", csv_filename.c_str());
        return -1;
    }

    ros::Subscriber sub_pcl = nh.subscribe(lidar_topic, 200000, pcl_cbk);
    ros::Subscriber sub_odom = nh.subscribe(odometry_topic, 200000, odom_cbk);

    std::thread thread_process{process};
    
    // 添加信号处理，确保程序退出时关闭CSV文件
    signal(SIGINT, [](int sig) {
        if (csv_file.is_open()) {
            csv_file.close();
            ROS_INFO("CSV file closed. Total %d odometry messages saved.", odom_counter);
        }
        ros::shutdown();
        exit(0);
    });
    
    ros::spin();
    
    // 程序结束时关闭CSV文件
    if (csv_file.is_open()) {
        csv_file.close();
        ROS_INFO("CSV file closed. Total %d odometry messages saved.", odom_counter);
    }

    return 0;

}