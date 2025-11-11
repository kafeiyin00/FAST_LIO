#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <random>
#include <mutex>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>
#include <deque>
#include <thread>
#include <condition_variable>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver/CustomMsg.h>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// #include "utility.h"

#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define RESET "\033[0m"

struct PointOuster
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t  ring;
    uint16_t ambient; // Available in NTU VIRAL and multicampus datasets
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointOuster,
                                 (float, x, x) (float, y, y) (float, z, z)
                                 (float, intensity, intensity)
                                 (uint32_t, t, t)
                                 (uint16_t, reflectivity, reflectivity)
                                 (uint8_t,  ring, ring)
                                 (uint16_t, ambient, ambient)
                                 (uint32_t, range, range))

typedef pcl::PointCloud<PointOuster> CloudOuster;
typedef pcl::PointCloud<PointOuster>::Ptr CloudOusterPtr;

struct CloudPacket
{
    double startTime;
    double endTime;

    CloudOusterPtr cloud;
    
    CloudPacket(){};
    CloudPacket(double startTime_, double endTime_, CloudOusterPtr cloud_)
        : startTime(startTime_), endTime(endTime_), cloud(cloud_)
    {}
};

struct ImuPacket
{
    double timestamp;
    sensor_msgs::Imu imu_data;
    
    ImuPacket(){};
    ImuPacket(double timestamp_, const sensor_msgs::Imu &imu_)
        : timestamp(timestamp_), imu_data(imu_)
    {}
};

using namespace std;
using namespace Eigen;

class MergeLidar
{

private:
        
    // Node handler
    ros::NodeHandlePtr nh_ptr;

    // Subscribers
    vector<ros::Subscriber> lidar_sub;
    vector<ros::Subscriber> imu_sub;   // IMU订阅者

    mutex lidar_buf_mtx;
    mutex lidar_leftover_buf_mtx;
    mutex imu_buf_mtx;                 // IMU缓冲区互斥锁

    deque<deque<CloudPacket>> lidar_buf;
    deque<deque<CloudPacket>> lidar_leftover_buf;
    deque<deque<ImuPacket>> imu_buf;   // IMU数据缓冲区

    ros::Publisher merged_pc_pub;
    ros::Publisher merged_livox_pub; // 新增：发布Livox CustomMsg格式的合并点云
    ros::Publisher merged_imu_pub;   // 修改：发布所有转换后的IMU数据（按时间顺序）

    // Lidar extrinsics
    deque<Matrix3d> R_B_L;
    deque<Vector3d> t_B_L;
    
    // IMU extrinsics (IMU to Body frame transformation)
    deque<Matrix3d> R_B_I;           // IMU到Body的旋转矩阵
    deque<Vector3d> t_B_I;           // IMU到Body的平移向量

    vector<int> lidar_channels;
    deque<int> lidar_ring_offset;
    bool lidar_ring_offset_set = false;
    mutex channel_mutex;

    int MAX_THREAD = std::thread::hardware_concurrency();
    int Nlidar;

    double cutoff_time = -1;
    double cutoff_time_new = -1;
    double sync_frequency = 20.0; // 同步频率 20Hz，实现实时性能
    double sync_period = 1.0 / sync_frequency;
    
    // 实时性能监控
    double last_sync_time = 0;
    int sync_count = 0;
    double total_sync_time = 0;

    thread sync_lidar;
    
public:
    // Destructor
    ~MergeLidar() {}

    MergeLidar(ros::NodeHandlePtr &nh_ptr_) : nh_ptr(nh_ptr_)
    {
        // Initialize the variables and subsribe/advertise topics here
        Initialize();
    }

    void Initialize()
    {

        /* #region Lidar --------------------------------------------------------------------------------------------*/
        
        // Read the lidar topic
        vector<string> lidar_topic = {"/os_cloud_node/points"};
        nh_ptr->getParam("lidar_topic", lidar_topic);

        Nlidar = lidar_topic.size();

        // Read IMU topics (optional, same number as lidars or empty)
        vector<string> imu_topic;
        nh_ptr->getParam("imu_topic", imu_topic);
        
        // If no IMU topics specified, create empty list
        if(imu_topic.empty())
        {
            imu_topic.resize(Nlidar);  // 创建与雷达数量相同的空字符串
            printf("No IMU topics specified. IMU fusion disabled.\n");
        }
        else if(imu_topic.size() != Nlidar)
        {
            printf(KRED "Warning: IMU topic count (%zu) does not match Lidar count (%d). IMU fusion disabled." RESET "\n", 
                   imu_topic.size(), Nlidar);
            imu_topic.clear();
            imu_topic.resize(Nlidar);  // 填充空字符串
        }

        // Read the extrincs of lidars
        vector<double> lidar_extr = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        nh_ptr->getParam("lidar_extr", lidar_extr);
        
        // Read IMU extrinsics (optional, default to identity transformation)
        vector<double> imu_extr;
        nh_ptr->getParam("imu_extr", imu_extr);
        
        // If no IMU extrinsics provided, use identity matrices
        if(imu_extr.empty())
        {
            // 为每个IMU创建单位变换矩阵
            imu_extr.resize(Nlidar * 16);
            for(int i = 0; i < Nlidar; i++)
            {
                // 单位矩阵
                for(int j = 0; j < 16; j++)
                    imu_extr[i*16 + j] = (j % 5 == 0) ? 1.0 : 0.0;  // 对角线为1，其余为0
            }
            printf("No IMU extrinsics specified. Using identity transformations.\n");
        }
        else if(imu_extr.size() != Nlidar * 16)
        {
            printf(KRED "Warning: IMU extrinsics size mismatch. Expected %d, got %zu. Using identity transformations." RESET "\n", 
                   Nlidar * 16, imu_extr.size());
            imu_extr.clear();
            imu_extr.resize(Nlidar * 16);
            for(int i = 0; i < Nlidar; i++)
            {
                for(int j = 0; j < 16; j++)
                    imu_extr[i*16 + j] = (j % 5 == 0) ? 1.0 : 0.0;
            }
        }

        ROS_ASSERT_MSG( (lidar_extr.size() / 16) == Nlidar,
                        "Lidar extrinsics not complete: %d < %d (= %d*16)\n",
                         lidar_extr.size(), Nlidar, Nlidar*16);

        printf("Received %d lidar(s) with extrinsics: \n", Nlidar);
        for(int i = 0; i < Nlidar; i++)
        {
            // Confirm the topics
            printf("Lidar topic #%02d: %s (Auto-detecting message type)\n", i, lidar_topic[i].c_str());

            Matrix4d extrinsicTf = Matrix<double, 4, 4, RowMajor>(&lidar_extr[i*16]);
            cout << "extrinsicTf: " << endl;
            cout << extrinsicTf << endl;

            R_B_L.push_back(extrinsicTf.block<3, 3>(0, 0));
            t_B_L.push_back(extrinsicTf.block<3, 1>(0, 3));

            lidar_buf.push_back(deque<CloudPacket>(0));
            lidar_leftover_buf.push_back(deque<CloudPacket>(0));
            
            // 添加IMU相关的初始化
            imu_buf.push_back(deque<ImuPacket>(0));
            
            // 处理IMU外参
            Matrix4d imu_extrinsicTf = Matrix<double, 4, 4, RowMajor>(&imu_extr[i*16]);
            R_B_I.push_back(imu_extrinsicTf.block<3, 3>(0, 0));
            t_B_I.push_back(imu_extrinsicTf.block<3, 1>(0, 3));

            // Subscribe to both message types - the system will auto-detect which one is active
            // PointCloud2 subscriber
            // lidar_sub.push_back(nh_ptr->subscribe<sensor_msgs::PointCloud2>
            //                                 (lidar_topic[i], 1000,
            //                                  boost::bind(&MergeLidar::PcHandlerPointCloud2, this,
            //                                              _1, i, (int)extrinsicTf(3, 3),
            //                                              extrinsicTf(3, 2))));
            
            // Livox CustomMsg subscriber  
            lidar_sub.push_back(nh_ptr->subscribe<livox_ros_driver::CustomMsg>
                                            (lidar_topic[i], 1000,
                                             boost::bind(&MergeLidar::PcHandlerLivox, this,
                                                         _1, i, (int)extrinsicTf(3, 3),
                                                         extrinsicTf(3, 2))));
            
            // IMU subscriber (if topic is specified)
            if(!imu_topic[i].empty())
            {
                imu_sub.push_back(nh_ptr->subscribe<sensor_msgs::Imu>
                                            (imu_topic[i], 1000,
                                             boost::bind(&MergeLidar::ImuHandler, this,
                                                         _1, i)));
                printf("Subscribed to IMU %d: %s\n", i, imu_topic[i].c_str());
                cout << "IMU extrinsicTf: " << endl;
                cout << imu_extrinsicTf << endl;
            }
            else
            {
                // 添加空的订阅者占位
                imu_sub.push_back(ros::Subscriber());
                printf("No IMU topic for lidar %d\n", i);
            }
            
            printf("Subscribed to lidar %d with dual message type support\n", i);
        }
        // nh_ptr->getParam("lidar_channels", lidar_channels);
        // if (lidar_channels.size() != Nlidar)
        // {
        //     printf(KRED "Lidar channel params missing: %d params vs %d Lidar" RESET,
        //             lidar_channels.size(), Nlidar);
        //     exit(-1);
        // }
        // Create the ring offsets
        lidar_channels = vector<int>(Nlidar, -1);
        lidar_ring_offset = deque<int>(Nlidar, 0);
        // for(int i = 1; i < Nlidar; i++)
        //     lidar_ring_offset[i] = lidar_ring_offset[i-1] + lidar_channels[i-1];

        merged_pc_pub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/merged_pointcloud", 1000);
        merged_livox_pub = nh_ptr->advertise<livox_ros_driver::CustomMsg>("/merged_livox", 1000);
        merged_imu_pub = nh_ptr->advertise<sensor_msgs::Imu>("/merged_imu", 1000);

        printf("Initialized MergeLidar with %d lidars\n", Nlidar);
        printf("Target sync frequency: %.1f Hz\n", sync_frequency);
        printf("Sync period: %.3f ms\n", sync_period * 1000);
        printf("Publishing merged PointCloud2 to: /merged_pointcloud\n");
        printf("Publishing merged Livox CustomMsg to: /merged_livox\n");
        printf("Publishing time-sorted IMU data to: /merged_imu\n");

        /* #endregion Lidar -----------------------------------------------------------------------------------------*/

        // Create the synchronizing threads
        sync_lidar = thread(&MergeLidar::SyncLidar, this);

    }

    void PcHandlerPointCloud2(const sensor_msgs::PointCloud2::ConstPtr &msg, int idx, int stamp_type, double time_offset)
    {
        // 性能监控
        static int handler_count = 0;
        static double last_handler_time = ros::Time::now().toSec();
        handler_count++;
        
        double startTime = -1, endTime = -1;
        if (stamp_type == 1)
        {
            startTime = msg->header.stamp.toSec() + time_offset;
            endTime = startTime + 0.1;
        }
        else
        {
            endTime = msg->header.stamp.toSec() + time_offset;
            startTime = endTime - 0.1;
        }

        // Convert the cloud msg to pcl
        CloudOusterPtr cloud_inL(new CloudOuster());
        pcl::fromROSMsg(*msg, *cloud_inL);
        
        // 检查点云数据质量
        if(cloud_inL->empty())
        {
            ROS_WARN("Received empty point cloud from lidar %d", idx);
            return;
        }
        
        // Check for the ring number so we can find the offset
        if(!lidar_ring_offset_set)
            lidar_ring_offset_set = checkLidarChannel(cloud_inL, idx);

        if(!lidar_ring_offset_set)
            return;

        int pointsTotal = cloud_inL->size();
        // Create the body-referenced cloud
        CloudOusterPtr cloud_inB(new CloudOuster()); 
        cloud_inB->resize(pointsTotal);
        cloud_inB->header = cloud_inL->header; // 保持头信息

        // Transform and copy the points - 使用OpenMP加速
        #pragma omp parallel for num_threads(MAX_THREAD) schedule(dynamic, 100)
        for(int i = 0; i < pointsTotal; i++)
        {
            const PointOuster &point_inL = cloud_inL->points[i];

            Vector3d p_inL(point_inL.x, point_inL.y, point_inL.z);
            p_inL = R_B_L[idx]*p_inL + t_B_L[idx];

            // Copy the point
            PointOuster point_inB = point_inL;
            
            // Replace the point coordinates
            point_inB.x = p_inL(0);
            point_inB.y = p_inL(1);
            point_inB.z = p_inL(2);

            // Renumber the ring
            point_inB.ring += lidar_ring_offset[idx];

            // Push the data into buffer
            cloud_inB->points[i] = point_inB;
        }

        // 缓冲区管理 - 防止缓冲区过大
        lidar_buf_mtx.lock();
        lidar_buf[idx].push_back(CloudPacket(startTime, endTime, cloud_inB));
        
        // 限制缓冲区大小，防止内存过度使用
        const size_t MAX_BUFFER_SIZE = 50;
        if(lidar_buf[idx].size() > MAX_BUFFER_SIZE)
        {
            lidar_buf[idx].pop_front();
            if(handler_count % 100 == 0)
                ROS_WARN("Lidar %d buffer overflow, dropping old frames", idx);
        }
        lidar_buf_mtx.unlock();
        
        // 周期性输出处理统计
        if(handler_count % 200 == 0)
        {
            double current_time = ros::Time::now().toSec();
            double elapsed = current_time - last_handler_time;
            double freq = 200.0 / elapsed;
            printf("Lidar %d: Processed %d frames, freq=%.1fHz, points=%d\n", 
                   idx, handler_count, freq, pointsTotal);
            last_handler_time = current_time;
        }
    }

    void PcHandlerLivox(const livox_ros_driver::CustomMsg::ConstPtr &msg, int idx, int stamp_type, double time_offset)
    {
        // 性能监控
        static int handler_count = 0;
        static double last_handler_time = ros::Time::now().toSec();
        handler_count++;
        
        double startTime = -1, endTime = -1;
        if (stamp_type == 1)
        {
            startTime = msg->header.stamp.toSec() + time_offset;
            endTime = startTime + 0.1;
        }
        else
        {
            endTime = msg->header.stamp.toSec() + time_offset;
            startTime = endTime - 0.1;
        }

        // 检查点云数据质量
        if(msg->points.empty())
        {
            ROS_WARN("Received empty Livox point cloud from lidar %d", idx);
            return;
        }
        
        // 创建PCL点云并转换Livox数据，使用Ouster格式以保持兼容性
        CloudOusterPtr cloud_inL(new CloudOuster());
        int pointsTotal = msg->points.size();
        cloud_inL->resize(pointsTotal);
        
        // 转换Livox点云格式到Ouster格式，保留intensity和时间信息
        for(int i = 0; i < pointsTotal; i++)
        {
            const auto &livox_point = msg->points[i];
            PointOuster &ouster_point = cloud_inL->points[i];
            
            // 基本坐标
            ouster_point.x = livox_point.x;
            ouster_point.y = livox_point.y;
            ouster_point.z = livox_point.z;
            
            // 强度信息
            ouster_point.intensity = livox_point.reflectivity;
            ouster_point.reflectivity = livox_point.reflectivity;
            
            // 时间信息 - 保留Livox的偏移时间
            ouster_point.t = livox_point.offset_time;
            
            // 线号信息
            ouster_point.ring = livox_point.line;
            
            // 距离信息
            ouster_point.range = sqrt(livox_point.x * livox_point.x + 
                                    livox_point.y * livox_point.y + 
                                    livox_point.z * livox_point.z) * 1000; // 转换为mm
        }
        
        // Check for the ring number so we can find the offset
        if(!lidar_ring_offset_set)
            lidar_ring_offset_set = checkLidarChannel(cloud_inL, idx);

        if(!lidar_ring_offset_set)
            return;

        // Create the body-referenced cloud
        CloudOusterPtr cloud_inB(new CloudOuster()); 
        cloud_inB->resize(pointsTotal);
        cloud_inB->header.stamp = msg->header.stamp.toNSec() / 1000; // 转换为微秒
        cloud_inB->header.frame_id = msg->header.frame_id;

        // Transform and copy the points - 使用OpenMP加速
        #pragma omp parallel for num_threads(MAX_THREAD) schedule(dynamic, 100)
        for(int i = 0; i < pointsTotal; i++)
        {
            const PointOuster &point_inL = cloud_inL->points[i];

            Vector3d p_inL(point_inL.x, point_inL.y, point_inL.z);
            p_inL = R_B_L[idx]*p_inL + t_B_L[idx];

            // Copy the point
            PointOuster point_inB = point_inL;
            
            // Replace the point coordinates
            point_inB.x = p_inL(0);
            point_inB.y = p_inL(1);
            point_inB.z = p_inL(2);

            // Renumber the ring
            point_inB.ring += lidar_ring_offset[idx];

            // Push the data into buffer
            cloud_inB->points[i] = point_inB;
        }

        // 缓冲区管理 - 防止缓冲区过大
        lidar_buf_mtx.lock();
        lidar_buf[idx].push_back(CloudPacket(startTime, endTime, cloud_inB));
        
        // 限制缓冲区大小，防止内存过度使用
        const size_t MAX_BUFFER_SIZE = 50;
        if(lidar_buf[idx].size() > MAX_BUFFER_SIZE)
        {
            lidar_buf[idx].pop_front();
            if(handler_count % 100 == 0)
                ROS_WARN("Livox Lidar %d buffer overflow, dropping old frames", idx);
        }
        lidar_buf_mtx.unlock();
        
        // 周期性输出处理统计
        if(handler_count % 200 == 0)
        {
            double current_time = ros::Time::now().toSec();
            double elapsed = current_time - last_handler_time;
            double freq = 200.0 / elapsed;
            printf("Livox Lidar %d: Processed %d frames, freq=%.1fHz, points=%d\n", 
                   idx, handler_count, freq, pointsTotal);
            last_handler_time = current_time;
        }
    }

    void ImuHandler(const sensor_msgs::Imu::ConstPtr &msg, int idx)
    {
        // 性能监控
        static int imu_handler_count = 0;
        static double last_imu_handler_time = ros::Time::now().toSec();
        imu_handler_count++;
        
        double timestamp = msg->header.stamp.toSec();
        
        // 创建转换到Body坐标系的IMU数据
        sensor_msgs::Imu transformed_imu = *msg;
        
        // 转换线性加速度 (考虑旋转和平移)
        Vector3d linear_acc_in_imu(msg->linear_acceleration.x, 
                                   msg->linear_acceleration.y, 
                                   msg->linear_acceleration.z);
        Vector3d linear_acc_in_body = R_B_I[idx] * linear_acc_in_imu;
        
        transformed_imu.linear_acceleration.x = linear_acc_in_body(0);
        transformed_imu.linear_acceleration.y = linear_acc_in_body(1);
        transformed_imu.linear_acceleration.z = linear_acc_in_body(2);
        
        // 转换角速度
        Vector3d angular_vel_in_imu(msg->angular_velocity.x, 
                                    msg->angular_velocity.y, 
                                    msg->angular_velocity.z);
        Vector3d angular_vel_in_body = R_B_I[idx] * angular_vel_in_imu;
        
        transformed_imu.angular_velocity.x = angular_vel_in_body(0);
        transformed_imu.angular_velocity.y = angular_vel_in_body(1);
        transformed_imu.angular_velocity.z = angular_vel_in_body(2);
        
        // 转换四元数姿态 (从IMU坐标系到Body坐标系)
        Eigen::Quaterniond q_imu(msg->orientation.w, msg->orientation.x, 
                                  msg->orientation.y, msg->orientation.z);
        Eigen::Quaterniond q_B_I(R_B_I[idx]);
        Eigen::Quaterniond q_body = q_B_I * q_imu;
        
        transformed_imu.orientation.w = q_body.w();
        transformed_imu.orientation.x = q_body.x();
        transformed_imu.orientation.y = q_body.y();
        transformed_imu.orientation.z = q_body.z();
        
        // 更新坐标系
        transformed_imu.header.frame_id = "body";
        
        // 缓冲区管理
        imu_buf_mtx.lock();
        imu_buf[idx].push_back(ImuPacket(timestamp, transformed_imu));
        
        // 限制缓冲区大小 - IMU频率通常较高，需要更大的缓冲区
        const size_t MAX_IMU_BUFFER_SIZE = 200;
        if(imu_buf[idx].size() > MAX_IMU_BUFFER_SIZE)
        {
            imu_buf[idx].pop_front();
            if(imu_handler_count % 500 == 0)
                ROS_WARN("IMU %d buffer overflow, dropping old data", idx);
        }
        imu_buf_mtx.unlock();
        
        // 周期性输出处理统计
        if(imu_handler_count % 1000 == 0)  // IMU频率高，每1000次输出一次
        {
            double current_time = ros::Time::now().toSec();
            double elapsed = current_time - last_imu_handler_time;
            double freq = 1000.0 / elapsed;
            printf("IMU %d: Processed %d samples, freq=%.1fHz\n", 
                   idx, imu_handler_count, freq);
            last_imu_handler_time = current_time;
        }
    }

    bool checkLidarChannel(CloudOusterPtr &cloud_inL, int idx)
    {
        std::lock_guard<mutex> lg(channel_mutex);

        int pointsTotal = cloud_inL->size();

        if(lidar_channels[idx] == -1)
        {
            for(int i = 0; i < pointsTotal; i++)
                lidar_channels[idx] = max(lidar_channels[idx], cloud_inL->points[i].ring + 1);
            printf("Lidar %d is found to have %d channels:\n", idx, lidar_channels[idx]);
        }

        // Exits the callback if any lidar channel has not been checked
        for(int lidx = 0; lidx < lidar_channels.size(); lidx++)
            if(lidar_channels[idx] == -1)
            {
                printf("Lidar %d channel check not completed.\n", idx);
                return false;
            }

        // All lidar has been checked. Calculate the channels offset
        printf("All lidar channels checked.\n");
        for(int lidx = 1; lidx < lidar_channels.size(); lidx++)
        {
            lidar_ring_offset[lidx] = lidar_ring_offset[lidx-1] + lidar_channels[lidx-1];
            printf("Lidar %d ring offset: %d\n", lidx, lidar_ring_offset[lidx]);
        }

        lidar_ring_offset_set = true;

        return true;
    }

    void SyncLidar()
    {
        printf("Starting real-time lidar synchronization thread...\n");
        last_sync_time = ros::Time::now().toSec();
        
        while(ros::ok())
        {
            double current_time = ros::Time::now().toSec();
            
            // Loop if the secondary buffers don't over lap
            if(!LidarBufReady())
            {
                this_thread::sleep_for(chrono::microseconds(1000)); // 减少到1ms以提高响应性
                continue;
            }

            // 性能监控 - 记录同步开始时间
            auto sync_start = chrono::high_resolution_clock::now();
            
            // Extract the points
            CloudPacket extracted_points;
            ExtractLidarPoints(extracted_points);

            // 性能监控 - 计算处理时间
            auto sync_end = chrono::high_resolution_clock::now();
            double processing_time = chrono::duration<double, milli>(sync_end - sync_start).count();
            
            // 更新性能统计
            sync_count++;
            total_sync_time += processing_time;
            double elapsed_time = current_time - last_sync_time;
            
            // 每100次同步输出一次性能统计
            if(sync_count % 100 == 0)
            {
                double avg_processing_time = total_sync_time / sync_count;
                double actual_frequency = sync_count / elapsed_time;
                printf("Sync Performance: Count=%d, Avg=%.2fms, Freq=%.1fHz, Target=%.1fHz\n", 
                       sync_count, avg_processing_time, actual_frequency, sync_frequency);
                
                // 重置统计
                sync_count = 0;
                total_sync_time = 0;
                last_sync_time = current_time;
            }

            // Update the cutoff time
            cutoff_time = cutoff_time_new;

            // Publish the merged pointcloud
            if(extracted_points.cloud->size() != 0)
            {
                ros::Time stamp_time = ros::Time(extracted_points.startTime);
                std::string frame_id = "body";
                
                // 发布PointCloud2格式
                publishCloud(merged_pc_pub, *extracted_points.cloud, stamp_time, frame_id);
                
                // 发布Livox CustomMsg格式
                publishLivoxCloud(merged_livox_pub, *extracted_points.cloud, stamp_time, frame_id);
                
                // 提取并按时间顺序发布所有IMU数据
                PublishTimeSortedImu(extracted_points.startTime, extracted_points.endTime);
                
                // 输出融合后的点云信息
                if(sync_count % 50 == 0) // 每50次输出一次
                {
                    printf("Merged cloud: %zu points, time span: %.3fms (Published as PointCloud2, Livox CustomMsg, and time-sorted IMU)\n", 
                           extracted_points.cloud->size(), 
                           (extracted_points.endTime - extracted_points.startTime) * 1000);
                }
            }
            
            // 自适应延迟控制 - 确保不超过目标频率
            double target_cycle_time = 1000.0 / sync_frequency; // ms
            if(processing_time < target_cycle_time)
            {
                double sleep_time = target_cycle_time - processing_time;
                this_thread::sleep_for(chrono::duration<double, milli>(sleep_time));
            }
        }
    }

    bool LidarBufReady()
    {
        // If any buffer is empty, lidar is not ready
        for(int i = 0; i < Nlidar; i++)
        {
            if (lidar_buf[i].empty())
            {
                return false;
            }
        }
        
        // If any secondary buffer's end still has not passed the first endtime in the primary buffer, loop
        for(int i = 1; i < Nlidar; i++)
        {
            if (lidar_buf[i].back().endTime < lidar_buf[0].front().endTime)
            {
                return false;
            }
        }

        return true;    
    }

    void ExtractLidarPoints(CloudPacket &extracted_points)
    {
        // Initiate the cutoff time.
        if (cutoff_time == -1)
            cutoff_time = lidar_buf[0].front().startTime;

        cutoff_time_new = lidar_buf[0].front().endTime;

        // Go through each buffer and extract the valid points
        deque<CloudOusterPtr> extracted_clouds(Nlidar);
        for(int i = 0; i < Nlidar; i++)
        {
            extracted_clouds[i] = CloudOusterPtr(new CloudOuster());

            CloudPacket leftover_cloud;
            leftover_cloud.startTime = cutoff_time_new;
            leftover_cloud.endTime = -1;
            leftover_cloud.cloud = CloudOusterPtr(new CloudOuster());

            while(!lidar_buf[i].empty())
            {
                CloudPacket &front_cloud = lidar_buf[i].front();

                if ( front_cloud.endTime < cutoff_time )
                {
                    lock_guard<mutex> lock(lidar_buf_mtx);
                    lidar_buf[i].pop_front();
                    continue;
                }
                else if( front_cloud.startTime > cutoff_time_new )
                    break;

                // Copy the points into the extracted pointcloud or the leftover
                for( auto &point : front_cloud.cloud->points )
                {
                    double point_time = point.t / 1.0e9 + front_cloud.startTime;

                    if (point_time >= cutoff_time && point_time <= cutoff_time_new)
                    {
                        extracted_clouds[i]->push_back(point);
                        extracted_clouds[i]->points.back().t = (uint32_t)((point_time - cutoff_time)*1.0e9);
                    }
                    else if (point_time > cutoff_time_new)
                    {
                        leftover_cloud.cloud->push_back(point);
                        leftover_cloud.cloud->points.back().t = (uint32_t)((point_time - cutoff_time_new)*1.0e9);

                        if (point_time > leftover_cloud.endTime)
                            leftover_cloud.endTime = point_time;
                    }
                }

                {
                    lock_guard<mutex> lock(lidar_buf_mtx);
                    lidar_buf[i].pop_front();
                }

                // Check the leftover buffer and insert extra points
                while(lidar_leftover_buf[i].size() != 0)
                {
                    if (lidar_leftover_buf[i].front().endTime < cutoff_time)
                    {
                        lidar_leftover_buf[i].pop_front();
                        continue;
                    }

                    if (lidar_leftover_buf[i].front().startTime > cutoff_time_new)
                        continue;

                    // Extract the first packet
                    CloudPacket leftover_frontcloud = lidar_leftover_buf[i].front();
                    lidar_leftover_buf[i].pop_front();

                    // Insert the leftover points back in the buffer
                    for( auto &point : leftover_frontcloud.cloud->points )
                    {
                        double point_time = point.t / 1.0e9 + leftover_frontcloud.startTime;

                        if (point_time >= cutoff_time && point_time <= cutoff_time_new)
                        {
                            extracted_clouds[i]->push_back(point);
                            extracted_clouds[i]->points.back().t = (uint32_t)((point_time - cutoff_time)*1.0e9);
                        }
                        else if (point_time > cutoff_time_new)
                        {
                            leftover_cloud.cloud->push_back(point);
                            leftover_cloud.cloud->points.back().t = (uint32_t)((point_time - cutoff_time_new)*1.0e9);

                            if (point_time > leftover_cloud.endTime)
                                leftover_cloud.endTime = point_time;
                        }
                    }
                }

                if (i == 0)
                    break;
            }

            if (leftover_cloud.cloud->size() > 0)
                lidar_leftover_buf[i].push_back(leftover_cloud);
        }

        // Merge the extracted clouds
        extracted_points.startTime = cutoff_time;
        extracted_points.endTime = cutoff_time_new;
        extracted_points.cloud = CloudOusterPtr(new CloudOuster());
        for(int i = 0; i < Nlidar; i++)
            *extracted_points.cloud += *extracted_clouds[i];
    }

    void publishCloud(ros::Publisher &pub, CloudOuster &cloud, ros::Time thisStamp, std::string thisFrame)
    {
        sensor_msgs::PointCloud2 cloud_;
        pcl::toROSMsg(cloud, cloud_);
        cloud_.header.stamp = thisStamp;
        cloud_.header.frame_id = thisFrame;
        pub.publish(cloud_);
    }

    void publishLivoxCloud(ros::Publisher &pub, CloudOuster &cloud, ros::Time thisStamp, std::string thisFrame)
    {
        livox_ros_driver::CustomMsg livox_msg;
        
        // 设置消息头
        livox_msg.header.stamp = thisStamp;
        livox_msg.header.frame_id = thisFrame;
        livox_msg.timebase = thisStamp.toNSec();
        livox_msg.point_num = cloud.size();
        livox_msg.lidar_id = 255; // 使用255表示合并后的点云
        
        // 分配点云数据空间
        livox_msg.points.resize(cloud.size());
        
        // 转换点云数据
        for(size_t i = 0; i < cloud.size(); i++)
        {
            const PointOuster &ouster_point = cloud.points[i];
            livox_ros_driver::CustomPoint &livox_point = livox_msg.points[i];
            
            // 基本坐标
            livox_point.x = ouster_point.x;
            livox_point.y = ouster_point.y;
            livox_point.z = ouster_point.z;
            
            // 反射率
            livox_point.reflectivity = ouster_point.reflectivity;
            
            // 时间偏移
            livox_point.offset_time = ouster_point.t;
            
            // 线号
            livox_point.line = ouster_point.ring;
            
            // 标签（设置为0，表示有效点）
            livox_point.tag = 0;
        }
        
        pub.publish(livox_msg);
    }
    
    void PublishTimeSortedImu(double start_time, double end_time)
    {
        // 收集所有在时间范围内的IMU数据
        vector<ImuPacket> all_imu_data;
        
        imu_buf_mtx.lock();
        
        // 遍历所有IMU传感器，收集时间范围内的数据
        for(int i = 0; i < Nlidar; i++)
        {
            if(imu_buf[i].empty()) continue;
            
            // 在此IMU的缓冲区中查找时间范围内的数据
            for(auto it = imu_buf[i].begin(); it != imu_buf[i].end(); ++it)
            {
                if(it->timestamp >= start_time && it->timestamp <= end_time)
                {
                    // 将已经转换到body坐标系的IMU数据添加到集合中
                    all_imu_data.push_back(*it);
                }
                else if(it->timestamp > end_time)
                {
                    break; // 时间已经超出范围，可以跳出
                }
            }
            
            // 清理过期的IMU数据
            while(!imu_buf[i].empty() && imu_buf[i].front().timestamp < start_time - 0.1) // 保留100ms的历史数据
            {
                imu_buf[i].pop_front();
            }
        }
        
        imu_buf_mtx.unlock();
        
        if(all_imu_data.empty())
        {
            return; // 没有IMU数据需要发布
        }
        
        // 按时间戳排序所有IMU数据
        sort(all_imu_data.begin(), all_imu_data.end(), 
             [](const ImuPacket &a, const ImuPacket &b) {
                 return a.timestamp < b.timestamp;
             });
        
        // 按时间顺序发布所有IMU数据
        for(const auto &imu_packet : all_imu_data)
        {
            sensor_msgs::Imu imu_msg = imu_packet.imu_data;
            
            // 确保时间戳和坐标系正确
            imu_msg.header.stamp = ros::Time(imu_packet.timestamp);
            imu_msg.header.frame_id = "body";
            
            // 发布IMU数据
            merged_imu_pub.publish(imu_msg);
        }
        
        // 输出统计信息
        static int publish_count = 0;
        publish_count++;
        if(publish_count % 100 == 0) // 每100次输出一次
        {
            printf("Published %zu time-sorted IMU samples in time range [%.3f, %.3f]s\n", 
                   all_imu_data.size(), start_time, end_time);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "merge_lidar");
    ros::NodeHandle nh("~");
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh);

    ROS_INFO(KGRN "----> Merge Lidar Started." RESET);

    MergeLidar sensor_sync(nh_ptr);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();
    
    return 0;
}