#include <ros/ros.h>
#include <Eigen/Dense>
#include <mutex>
#include <queue>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/PoseStamped.h>
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

#include <geometry_msgs/QuaternionStamped.h> // 重力方向对齐消息
#include <ros/topic.h>  // waitForMessage 

std::string lidar_topic,odometry_topic,transform_topic,dataFolder,pointsFolder;
int frame_size;

// 与 laserMapping.cpp 保持一致的固定模式名
const std::string REF_MODE_NO_REF = "no_ref";
const std::string REF_MODE_PUB_GRAV_TRUTH = "ref_pub_grav_truth";
const std::string REF_MODE_PUB_REF_TRUTH = "ref_pub_ref_truth";
std::string ref_mode = REF_MODE_NO_REF;

// 真实数据使用配准好的点云
bool save_registered_data;
std ::string world_lidar_topic;
int current_frame_index = 0;

// 全局坐标系转换相关参数
bool apply_global_shift = false;
Eigen::Matrix4d T_global_local = Eigen::Matrix4d::Identity();

std::queue<nav_msgs::Odometry::ConstPtr> odom_buf;
std::queue<geometry_msgs::PoseStamped::ConstPtr> transform_buf;
std::queue<sensor_msgs::PointCloud2::ConstPtr> pcl_buf;
std::queue<sensor_msgs::PointCloud2::ConstPtr> world_pcl_buf;
std::mutex bufMutex;

// CSV相关变量
std::ofstream csv_file;
std::ofstream csv_file_global;  // 新增：全局坐标系CSV文件
std::string csv_filename;
std::string csv_filename_global;  // 新增：全局坐标系CSV文件名
int odom_counter = 0;
std::mutex csv_mutex;
double last_timestamp = -1.0;  // 记录上一次保存的时间戳

bool isValidRefMode(const std::string &mode)
{
    return mode == REF_MODE_NO_REF ||
           mode == REF_MODE_PUB_GRAV_TRUTH ||
           mode == REF_MODE_PUB_REF_TRUTH;
}

// 根据固定模式名自动对齐 generate_block 的输入语义，避免 launch 中重复维护多组 topic
void configureInputByMode()
{
    if (ref_mode == REF_MODE_NO_REF)
    {
        // 原始模式保留两种保存策略：
        //   save_registered_data = false -> 输入 body 系点云，按里程计拼子图
        //   save_registered_data = true  -> 直接保存已经表达在 G 系的整帧点云
        lidar_topic = "/cloud_registered_body";
        odometry_topic = "/Odometry";
        transform_topic.clear();
        world_lidar_topic = "/cloud_registered";
    }
    else if (ref_mode == REF_MODE_PUB_GRAV_TRUTH)
    {
        // grav_truth：输入点云在 G 系，额外关系由固定 T^W_G 提供
        save_registered_data = true;
        world_lidar_topic = "/cloud_registered";
        odometry_topic = "/grav_truth_Odometry";
        transform_topic = "/grav_truth_TWG";
    }
    else
    {
        // ref_truth：输入点云已经在 TLS/W 系，额外关系恒为单位阵
        save_registered_data = true;
        world_lidar_topic = "/ref_truth_cloud_registered";
        odometry_topic = "/ref_truth_Odometry";
        transform_topic = "/ref_truth_TWW";
    }

    ROS_INFO("generate_block mode: %s", ref_mode.c_str());
    ROS_INFO("generate_block topics: lidar=%s, odom=%s, transform=%s, world_lidar=%s, save_registered_data=%s",
             lidar_topic.c_str(),
             odometry_topic.c_str(),
             transform_topic.empty() ? "<none>" : transform_topic.c_str(),
             world_lidar_topic.c_str(),
             save_registered_data ? "true" : "false");
}

void pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    bufMutex.lock();
    // ROS_INFO("receive pcl");
    pcl_buf.push(msg);
    bufMutex.unlock();
}

void world_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    bufMutex.lock();
    // ROS_INFO("receive pcl");
    world_pcl_buf.push(msg);
    bufMutex.unlock();
}

void transform_cbk(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    bufMutex.lock();
    transform_buf.push(msg);
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

        // 如果启用全局坐标系转换，同时保存全局坐标系位姿
        if (apply_global_shift && csv_file_global.is_open()) {
            // 构建局部坐标系下的变换矩阵
            Eigen::Quaterniond q_local(qw, qx, qy, qz);
            q_local.normalize();
            Eigen::Vector3d t_local(x, y, z);
            
            Eigen::Matrix4d T_local = Eigen::Matrix4d::Identity();
            T_local.block<3,3>(0,0) = q_local.toRotationMatrix();
            T_local.block<3,1>(0,3) = t_local;
            
            // 转换到全局坐标系: T^G = T^G_L * T^L
            Eigen::Matrix4d T_global = T_global_local * T_local;
            
            // 提取全局坐标系下的位姿
            Eigen::Vector3d t_global = T_global.block<3,1>(0,3);
            Eigen::Matrix3d rot_global = T_global.block<3,3>(0,0);
            Eigen::Quaterniond q_global(rot_global);
            q_global.normalize();
            
            // 保存全局坐标系位姿到单独的 CSV
            csv_file_global << odom_counter << "," 
                           << std::fixed << std::setprecision(9) << timestamp << ","
                           << std::fixed << std::setprecision(6) << t_global.x() << ","
                           << std::fixed << std::setprecision(6) << t_global.y() << ","
                           << std::fixed << std::setprecision(6) << t_global.z() << ","
                           << std::fixed << std::setprecision(6) << q_global.x() << ","
                           << std::fixed << std::setprecision(6) << q_global.y() << ","
                           << std::fixed << std::setprecision(6) << q_global.z() << ","
                           << std::fixed << std::setprecision(6) << q_global.w() << std::endl;
            csv_file_global.flush();
        }
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

Eigen::Matrix4d poseToMatrix(const geometry_msgs::Pose &pose)
{
    Eigen::Quaterniond q(pose.orientation.w,
                         pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z);
    q.normalize();

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = q.toRotationMatrix();
    transform(0, 3) = pose.position.x;
    transform(1, 3) = pose.position.y;
    transform(2, 3) = pose.position.z;
    return transform;
}

void saveMatrixToFile(const Eigen::Matrix4d &transform, const char *file_name)
{
    FILE *fp = fopen(file_name, "w");
    fprintf(fp, "%f %f %f %f\n %f %f %f %f\n %f %f %f %f\n 0 0 0 1",
            transform(0, 0), transform(0, 1), transform(0, 2), transform(0, 3),
            transform(1, 0), transform(1, 1), transform(1, 2), transform(1, 3),
            transform(2, 0), transform(2, 1), transform(2, 2), transform(2, 3));
    fclose(fp);
}

void closeCsvFiles()
{
    if (csv_file.is_open()) {
        csv_file.close();
    }
    if (csv_file_global.is_open()) {
        csv_file_global.close();
    }
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
            transform.block<3,3>(0,0) = q.toRotationMatrix().cast<float>();
            transform.block<3,1>(0,3) = t.cast<float>();
            
            pcl::transformPointCloud(*temp_cloud,*temp_cloud_trans,transform);
            //transform to ref
            *all_cloud += *temp_cloud_trans;
        }
        // pcl::VoxelGrid<pcl::PointXYZI> sor;
        // sor.setInputCloud (all_cloud);
        // sor.setLeafSize (0.05f, 0.05f, 0.05f);
        // pcl::PointCloud<pcl::PointXYZI>::Ptr down_sample(new pcl::PointCloud<pcl::PointXYZI>);
        // sor.filter(*down_sample);
        pcl::io::savePCDFileBinary<pcl::PointXYZI>(pclFileName,*all_cloud);
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

        // 如果启用了全局坐标系转换，保存全局坐标系下的位姿
        if (apply_global_shift) {
            // 构建局部坐标系下的参考帧变换矩阵 T^L_R
            Eigen::Matrix4d T_local_ref = Eigen::Matrix4d::Identity();
            T_local_ref.block<3,3>(0,0) = rot;
            T_local_ref.block<3,1>(0,3) = ref_t;

            // 转换到全局坐标系: T^G_R = T^G_L * T^L_R
            Eigen::Matrix4d T_global_ref = T_global_local * T_local_ref;

            // 提取全局坐标系下的旋转和平移
            Eigen::Matrix3d rot_global = T_global_ref.block<3,3>(0,0);
            Eigen::Vector3d ref_t_global = T_global_ref.block<3,1>(0,3);

            // 保存全局坐标系下的位姿（global_odom 文件）
            char globalPosFileName[256];
            sprintf(globalPosFileName,"%s/global_%d_%d.odom",dataFolder.c_str(),sec,nsec);
            FILE *fp_global = fopen(globalPosFileName,"w");
            fprintf(fp_global,"%f %f %f %f\n %f %f %f %f\n %f %f %f %f\n 0 0 0 1",
                        rot_global(0,0), rot_global(0,1), rot_global(0,2), ref_t_global(0),
                        rot_global(1,0), rot_global(1,1), rot_global(1,2), ref_t_global(1),
                        rot_global(2,0), rot_global(2,1), rot_global(2,2), ref_t_global(2));
            fclose(fp_global);
        }

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
        //pcl::VoxelGrid<pcl::PointXYZI> sor;
        //sor.setInputCloud (all_cloud);
        //sor.setLeafSize (0.2f, 0.2f, 0.2f);
        //pcl::PointCloud<pcl::PointXYZI>::Ptr down_sample(new pcl::PointCloud<pcl::PointXYZI>);
        //sor.filter(*down_sample);
        //pcl::io::savePCDFileBinary<pcl::PointXYZI>(pclFileName,*down_sample);
        
        pcl::io::savePCDFileBinary<pcl::PointXYZI>(pclFileName,*all_cloud);
    }
}

void directOutput( std::vector<nav_msgs::Odometry> currentOdoMsg, std::vector<sensor_msgs::PointCloud2> currentWorldPclMsg)
{

    if(currentWorldPclMsg.size() > 0 && currentOdoMsg.size() > 0 && currentWorldPclMsg.size() == currentOdoMsg.size())
    {
        double currentTime = currentOdoMsg[0].header.stamp.toSec();
        char posFileName[256];
        char worldPclFileName[256];

        uint sec = currentTime;
        uint nsec = (currentTime-sec)*1e9;

        // must use the format of sec_nsec
        sprintf(posFileName,"%s/%d_%d.odom",dataFolder.c_str(),sec,nsec);
        sprintf(worldPclFileName,"%s/%d_%d.pcd",dataFolder.c_str(),sec,nsec);

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

        // xinzhao 
        Eigen::Matrix4f ref_transform;
        ref_transform.setIdentity();
        ref_transform.block<3,3>(0,0) = ref_q.toRotationMatrix().cast<float>();
        ref_transform.block<3,1>(0,3) = ref_t.cast<float>();

        // 如果启用了全局坐标系转换，保存全局坐标系下的位姿
        if (apply_global_shift) {
            // 构建局部坐标系下的参考帧变换矩阵 T^L_R
            Eigen::Matrix4d T_local_ref = Eigen::Matrix4d::Identity();
            T_local_ref.block<3,3>(0,0) = rot;
            T_local_ref.block<3,1>(0,3) = ref_t;

            // 转换到全局坐标系: T^G_R = T^G_L * T^L_R
            Eigen::Matrix4d T_global_ref = T_global_local * T_local_ref;

            // 提取全局坐标系下的旋转和平移
            Eigen::Matrix3d rot_global = T_global_ref.block<3,3>(0,0);
            Eigen::Vector3d ref_t_global = T_global_ref.block<3,1>(0,3);

            // 保存全局坐标系下的位姿（global_odom 文件）
            char globalPosFileName[256];
            sprintf(globalPosFileName,"%s/global_%d_%d.odom",dataFolder.c_str(),sec,nsec);
            FILE *fp_global = fopen(globalPosFileName,"w");
            fprintf(fp_global,"%f %f %f %f\n %f %f %f %f\n %f %f %f %f\n 0 0 0 1",
                        rot_global(0,0), rot_global(0,1), rot_global(0,2), ref_t_global(0),
                        rot_global(1,0), rot_global(1,1), rot_global(1,2), ref_t_global(1),
                        rot_global(2,0), rot_global(2,1), rot_global(2,2), ref_t_global(2));
            fclose(fp_global);
        }

        // save pcl
        pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        for (size_t i = 0; i < currentWorldPclMsg.size(); i++)
        {


            pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);

            pcl::fromROSMsg(currentWorldPclMsg[i],*temp_cloud);

            *all_cloud += *temp_cloud;
        }
        //pcl::VoxelGrid<pcl::PointXYZI> sor;
        //sor.setInputCloud (all_cloud);
        //sor.setLeafSize (0.2f, 0.2f, 0.2f);
        //pcl::PointCloud<pcl::PointXYZI>::Ptr down_sample(new pcl::PointCloud<pcl::PointXYZI>);
        //sor.filter(*down_sample);
        //pcl::io::savePCDFileBinary<pcl::PointXYZI>(pclFileName,*down_sample);
        pcl::io::savePCDFileBinary<pcl::PointXYZI>(worldPclFileName,*all_cloud);
    }
}

void saveRefFrames(std::vector<geometry_msgs::PoseStamped> currentTransformMsg,
 std::vector<sensor_msgs::PointCloud2> currentWorldPclMsg)
{
    if(currentWorldPclMsg.size() > 0 && currentTransformMsg.size() > 0 && currentWorldPclMsg.size() == currentTransformMsg.size())
    {
        double currentTime = currentTransformMsg[0].header.stamp.toSec();
        char posFileName[256];
        char worldPclFileName[256];

        uint sec = currentTime;
        uint nsec = (currentTime-sec)*1e9;

        sprintf(posFileName,"%s/%d_%d.odom",dataFolder.c_str(),sec,nsec);
        sprintf(worldPclFileName,"%s/%d_%d.pcd",dataFolder.c_str(),sec,nsec);

        // ref 模式下一个 block 内所有帧共享同一个固定关系矩阵：
        //   grav_truth 读取固定 T^W_G
        //   ref_truth 读取固定 T^W_W = I
        // 因此这里取这一组中的首个消息即可。
        Eigen::Matrix4d ref_transform = poseToMatrix(currentTransformMsg[0].pose);
        saveMatrixToFile(ref_transform, posFileName);

        if (apply_global_shift) {
            Eigen::Matrix4d global_transform = T_global_local * ref_transform;
            char globalPosFileName[256];
            sprintf(globalPosFileName,"%s/global_%d_%d.odom",dataFolder.c_str(),sec,nsec);
            saveMatrixToFile(global_transform, globalPosFileName);
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (size_t i = 0; i < currentWorldPclMsg.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(currentWorldPclMsg[i],*temp_cloud);
            *all_cloud += *temp_cloud;
        }

        pcl::io::savePCDFileBinary<pcl::PointXYZI>(worldPclFileName,*all_cloud);
    }
}

void process()
{
    ros::Rate rate(100);
    std::vector<nav_msgs::Odometry> currentOdoMsg;
    std::vector<geometry_msgs::PoseStamped> currentTransformMsg;
    std::vector<sensor_msgs::PointCloud2> currentPclMsg;
    std::vector<sensor_msgs::PointCloud2> currentWorldPclMsg;
    while (ros::ok()) {

        bufMutex.lock();///lock before access Buf
        if(save_registered_data){
            if (ref_mode == REF_MODE_NO_REF)
            {
                // no_ref + save_registered_data: 恢复旧逻辑，直接保存 G 系整帧点云与 /Odometry
                if(!odom_buf.empty() && !world_pcl_buf.empty()){
                    currentOdoMsg.push_back(*odom_buf.front());
                    odom_buf.pop();
                    currentWorldPclMsg.push_back(*world_pcl_buf.front());
                    world_pcl_buf.pop();

                    if(currentWorldPclMsg.size() >= frame_size) {
                        directOutput(currentOdoMsg,currentWorldPclMsg);
                        currentOdoMsg.clear();
                        currentWorldPclMsg.clear();
                    }
                }
            }
            else
            {
                // 两个参考图模式都走这里：直接保存目标坐标系点云，并记录其对应的固定关系矩阵
                if(!transform_buf.empty() && !world_pcl_buf.empty()){
                    currentTransformMsg.push_back(*transform_buf.front());
                    transform_buf.pop();
                    currentWorldPclMsg.push_back(*world_pcl_buf.front());
                    world_pcl_buf.pop();

                    if(currentWorldPclMsg.size() >= frame_size) {
                        saveRefFrames(currentTransformMsg,currentWorldPclMsg);
                        currentTransformMsg.clear();
                        currentWorldPclMsg.clear();
                    }
                }
            }
        }
        else{
            // no_ref 旧模式：输入是局部 body 系点云，需要结合 /Odometry 把一组帧拼成子图
            if (!odom_buf.empty() && !pcl_buf.empty())
            {  
                currentOdoMsg.push_back(*odom_buf.front());
                odom_buf.pop();
                currentPclMsg.push_back(*pcl_buf.front());
                pcl_buf.pop();
    
                // next image
                if(currentOdoMsg.size() >= frame_size)
                {
                    // save process
                    transformAndOutput(currentOdoMsg,currentPclMsg);
                    //savePoints(currentOdoMsg,currentPclMsg);
                    currentOdoMsg.clear();
                    currentPclMsg.clear();
                }
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
    nh.param<int>("frame_number",frame_size, 50);
    nh.param<std::string>("pointsFolder",pointsFolder, "/home/iot/workspace/data/points");
    nh.param<std::string>("ref_map/mode", ref_mode, REF_MODE_NO_REF);

    nh.param<bool>("apply_global_shift", apply_global_shift, false);
   
    nh.param<bool>("save_registered_data", save_registered_data, false);
    nh.param<std::string>("world_lidar_msg_name", world_lidar_topic, "/cloud_registered");

    if (!isValidRefMode(ref_mode)) {
        ROS_WARN("Unsupported ref_map/mode: %s, fallback to no_ref", ref_mode.c_str());
        ref_mode = REF_MODE_NO_REF;
    }
    configureInputByMode();

    // 读取全局坐标系转换矩阵
    std::vector<double> global_shift_list;
    if (nh.getParam("global_shift", global_shift_list)) {
        if (global_shift_list.size() == 16) {
            // 按行优先顺序读取 4x4 矩阵
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    T_global_local(i, j) = global_shift_list[i * 4 + j];
                }
            }
            
            ROS_INFO("Global shift transformation matrix loaded:");
            std::cout << T_global_local << std::endl;
        } else {
            ROS_WARN("global_shift parameter should have 16 elements, got %zu. Using identity.", 
                     global_shift_list.size());
            T_global_local = Eigen::Matrix4d::Identity();
        }
    } else {
        if (apply_global_shift) {
            ROS_WARN("apply_global_shift is true but global_shift matrix not provided. Using identity.");
        }
        T_global_local = Eigen::Matrix4d::Identity();
    }

    if(!boost::filesystem::exists(dataFolder))
    {
        boost::filesystem::create_directories(dataFolder);
    }

    if(!boost::filesystem::exists(pointsFolder))
    {
        boost::filesystem::create_directories(pointsFolder);
    }
    
    if (ref_mode == REF_MODE_NO_REF) {
        // no_ref 下仍保留里程计 CSV 输出，方便调试轨迹
        time_t rawtime;
        struct tm * timeinfo;
        char timestamp_str[80];
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        strftime(timestamp_str, sizeof(timestamp_str), "%Y%m%d_%H%M%S", timeinfo);
        
        csv_filename = pointsFolder + "/odometry_" + std::string(timestamp_str) + ".csv";
        csv_file.open(csv_filename);
        
        if (csv_file.is_open()) {
            csv_file << "num\tt\tx\ty\tz\tqx\tqy\tqz\tqw" << std::endl;
            ROS_INFO("Created CSV file: %s", csv_filename.c_str());
        } else {
            ROS_ERROR("Failed to create CSV file: %s", csv_filename.c_str());
            return -1;
        }

        if (apply_global_shift) {
            csv_filename_global = pointsFolder + "/global_odometry_" + std::string(timestamp_str) + ".csv";
            csv_file_global.open(csv_filename_global);
            
            if (csv_file_global.is_open()) {
                csv_file_global << "num,t,x,y,z,qx,qy,qz,qw" << std::endl;
                ROS_INFO("Created global odometry CSV file: %s", csv_filename_global.c_str());
            } else {
                ROS_ERROR("Failed to create global odometry CSV file: %s", csv_filename_global.c_str());
                return -1;
            }
        }
    }

    ros::Subscriber sub_pcl;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_world_pcl;
    ros::Subscriber sub_transform;
    if (save_registered_data) {
        sub_world_pcl = nh.subscribe(world_lidar_topic, 200000, world_pcl_cbk);
        if (ref_mode == REF_MODE_NO_REF) {
            sub_odom = nh.subscribe(odometry_topic, 200000, odom_cbk);
        } else {
            sub_transform = nh.subscribe(transform_topic, 200000, transform_cbk);
        }
    } else {
        sub_pcl = nh.subscribe(lidar_topic, 200000, pcl_cbk);
        sub_odom = nh.subscribe(odometry_topic, 200000, odom_cbk);
    }

    std::thread thread_process{process};
    
    // 添加信号处理，确保程序退出时关闭CSV文件
    signal(SIGINT, [](int sig) {
        ros::shutdown();
    });
    
    ros::spin();

    if (thread_process.joinable()) {
        thread_process.join();
    }
    
    // 程序结束时关闭CSV文件
    if (csv_file.is_open() || csv_file_global.is_open()) {
        closeCsvFiles();
        ROS_INFO("CSV file closed. Total %d odometry messages saved.", odom_counter);
    }

    return 0;

}
