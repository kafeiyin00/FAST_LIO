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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

std::string lidar_topic,odometry_topic,dataFolder;

std::queue<nav_msgs::Odometry::ConstPtr> odom_buf;
std::queue<sensor_msgs::PointCloud2::ConstPtr> pcl_buf;
std::mutex bufMutex;

void pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    bufMutex.lock();
    // ROS_INFO("receive pcl");
    pcl_buf.push(msg);
    bufMutex.unlock();
}

void odom_cbk(const nav_msgs::Odometry::ConstPtr &msg) 
{
    bufMutex.lock();
    // ROS_INFO("receive odometry");
    odom_buf.push(msg);
    bufMutex.unlock();
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
        pcl::io::savePCDFileBinary<pcl::PointXYZI>(pclFileName,*all_cloud);
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
            if(currentOdoMsg.size() >= 50)
            {
                // save process
                transformAndOutput(currentOdoMsg,currentPclMsg);
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

    nh.param<std::string>("lidar_msg_name",lidar_topic, "/cloud_registered_body");
    nh.param<std::string>("odometry_msg_name",odometry_topic, "/Odometry"); 
    nh.param<std::string>("dataFolder",dataFolder, "/home/iot/workspace/data/frames");

    if(!boost::filesystem::exists(dataFolder))
    {
        boost::filesystem::create_directories(dataFolder);
    }

    ros::Subscriber sub_pcl = nh.subscribe(lidar_topic, 200000, pcl_cbk);
    ros::Subscriber sub_odom = nh.subscribe(odometry_topic, 200000, odom_cbk);

    std::thread thread_process{process};
    ros::spin();

}