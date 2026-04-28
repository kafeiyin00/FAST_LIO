#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Eigen>
#include <cmath>
#include <queue>
#include <deque>
#include <std_msgs/Int32.h>
#include <fast_lio/StampedInt32.h>
#include <livox_ros_driver/CustomMsg.h>
#include <mutex>
#include <algorithm>

// 存储编码器数据的结构体
struct EncoderData {
    double timestamp;
    double encoder_value;
    
    EncoderData() : timestamp(0.0), encoder_value(0.0) {}
    EncoderData(double t, double v) : timestamp(t), encoder_value(v) {}
};

Eigen::Vector3d p_lidar_body, p_base;
Eigen::Quaterniond q_w_lidar, q_rotorframe_lidar, q_rotor_rotorframe, q_base_rotor;
Eigen::Quaterniond q_base_w;
Eigen::Matrix4f transform_lidar_to_base;

// 编码器数据队列和互斥锁
std::deque<EncoderData> encoder_queue;
std::mutex encoder_mutex;
const size_t MAX_ENCODER_QUEUE_SIZE = 1000;
const double MAX_INTERPOLATION_TIME = 1.0;

double rotor_encoder_value = 0;
double last_used_encoder_value = 0; // 添加上一次使用的编码器值
bool encoder_initialized = false;   // 编码器是否已初始化

// 全局发布器声明
ros::Publisher base_pointcloud_pub;


// 线性插值函数，根据时间戳获取编码器值
double interpolateEncoderValue(double target_timestamp)
{
    std::lock_guard<std::mutex> lock(encoder_mutex);
    
    if (encoder_queue.empty()) {
        ROS_WARN("Encoder queue is empty, using last known value: %.3f", rotor_encoder_value);
        return rotor_encoder_value;
    }
    
    // 如果只有一个数据点，直接返回
    if (encoder_queue.size() == 1) {
        return encoder_queue.front().encoder_value;
    }
    
    // 查找目标时间戳周围的两个数据点
    auto it_upper = std::lower_bound(encoder_queue.begin(), encoder_queue.end(), target_timestamp,
                                     [](const EncoderData& data, double timestamp) {
                                         return data.timestamp < timestamp;
                                     });
    
    // 如果目标时间在队列范围之前
    if (it_upper == encoder_queue.begin()) {
        double time_diff = encoder_queue.front().timestamp - target_timestamp;
        if (time_diff > MAX_INTERPOLATION_TIME) {
            ROS_WARN("Target timestamp %.6f is too old (%.3fs before first data), using first available value", 
                     target_timestamp, time_diff);
        }
        return encoder_queue.front().encoder_value;
    }
    
    // 如果目标时间在队列范围之后，进行外推
    if (it_upper == encoder_queue.end()) {
        double time_diff = target_timestamp - encoder_queue.back().timestamp;
        if (time_diff > MAX_INTERPOLATION_TIME) {
            ROS_WARN("Target timestamp %.6f is too new (%.3fs after last data), using extrapolated value", 
                     target_timestamp, time_diff);
        }
        
        // 使用最后两个数据点进行线性外推
        if (encoder_queue.size() >= 2) {
            auto it_last = encoder_queue.end() - 1;
            auto it_second_last = encoder_queue.end() - 2;
            
            double t1 = it_second_last->timestamp;
            double t2 = it_last->timestamp;
            double v1 = it_second_last->encoder_value;
            double v2 = it_last->encoder_value;
            
            // 处理角度跳变
            double angle_diff = v2 - v1;
            if (angle_diff > 180.0) {
                v2 -= 360.0;
            } else if (angle_diff < -180.0) {
                v2 += 360.0;
            }
            
            // 计算变化率并外推
            double rate = (v2 - v1) / (t2 - t1);
            double extrapolated_value = v2 + rate * (target_timestamp - t2);
            
            // 确保角度在0-360度范围内
            while (extrapolated_value < 0.0) extrapolated_value += 360.0;
            while (extrapolated_value >= 360.0) extrapolated_value -= 360.0;
            
            ROS_DEBUG("Extrapolated encoder value: %.3f at timestamp %.6f (rate: %.3f)", 
                      extrapolated_value, target_timestamp, rate);
            
            return extrapolated_value;
        }
        
        return encoder_queue.back().encoder_value;
    }
    
    // 进行线性插值
    auto it_lower = it_upper - 1;
    
    double t1 = it_lower->timestamp;
    double t2 = it_upper->timestamp;
    double v1 = it_lower->encoder_value;
    double v2 = it_upper->encoder_value;
    
    // 处理角度跳变（0度到360度或360度到0度）
    double angle_diff = v2 - v1;
    if (angle_diff > 180.0) {
        v2 -= 360.0;  // 从小角度跳到大角度
    } else if (angle_diff < -180.0) {
        v2 += 360.0;  // 从大角度跳到小角度
    }
    
    // 线性插值
    double weight = (target_timestamp - t1) / (t2 - t1);
    double interpolated_value = v1 + weight * (v2 - v1);
    
    // 确保角度在0-360度范围内
    while (interpolated_value < 0.0) interpolated_value += 360.0;
    while (interpolated_value >= 360.0) interpolated_value -= 360.0;
    
    ROS_DEBUG("Interpolated encoder value: %.3f at timestamp %.6f (between %.6f and %.6f)", 
              interpolated_value, target_timestamp, t1, t2);
    
    return interpolated_value;
}

void pointcloud_callback(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    ROS_INFO("Received Livox pointcloud with %d points", msg->point_num);
    
    // 基础时间戳（消息头时间戳）
    double base_timestamp = msg->header.stamp.toSec();
    
    // 计算点云时间范围
    double beg_lidar_time = base_timestamp + msg->points.front().offset_time / 1e9;
    double end_lidar_time = base_timestamp + msg->points.back().offset_time / 1e9;
    
    // 获取有效的编码器数据（扩展时间范围以确保覆盖）
    std::vector<std::pair<double, double>> valid_encoder_data;
    {
        std::lock_guard<std::mutex> lock(encoder_mutex);
        
        // 扩大搜索范围，特别是在前期编码器数据稀少时
        double search_margin = 0.1; // 增加搜索边界到0.5秒
        double search_start = beg_lidar_time - search_margin;
        double search_end = end_lidar_time + search_margin;
        
        ROS_DEBUG("Encoder queue size: %d, searching time range: [%.6f, %.6f]", 
                 (int)encoder_queue.size(), search_start, search_end);
        
        for (const auto& data : encoder_queue) {
            if (data.timestamp >= search_start && data.timestamp <= search_end) {
                valid_encoder_data.push_back({data.timestamp, data.encoder_value * M_PI / 180.0}); // 转换为弧度
                ROS_DEBUG("Found valid encoder data: t=%.6f, angle=%.3f°", data.timestamp, data.encoder_value);
            }
        }
        
        // 按时间戳排序，确保插值的正确性
        std::sort(valid_encoder_data.begin(), valid_encoder_data.end());
        
        // 如果扩大范围后仍然没有数据，尝试获取最近的编码器数据
        if (valid_encoder_data.empty()) {
            return;
        }
    }
    
    ROS_INFO("Valid encoder measurements: %d", (int)valid_encoder_data.size());
    
    // 如果编码器数据不足，使用当前编码器值进行处理
    if (valid_encoder_data.size() < 2) {
        return;
    }
    
    // 创建变换后的点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    transformed_cloud->points.reserve(msg->point_num);
    
    // 固定的旋转矩阵（激光雷达到转子坐标系）
    Eigen::Matrix3d R_rotor_lidar = Eigen::AngleAxisd(-60 * M_PI/180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    
    // 为每个点单独计算变换
    for (uint32_t i = 0; i < msg->point_num; ++i) {
        const auto& point = msg->points[i];
        
        // 过滤低反射率点
        // if (point.reflectivity < 1) continue;
        
        // 计算该点的绝对时间戳
        double point_timestamp = base_timestamp + point.offset_time / 1e9;
        
        // 在编码器数据中插值旋转角度
        double rot_angular = interpolateEncoderValue(point_timestamp);
        
        // 点坐标变换
        Eigen::Vector3d pt_lidar(point.x, point.y, point.z);
        
        // 应用变换：激光雷达 -> 转子坐标系 -> 旋转 -> 基座坐标系
        Eigen::Vector3d pt_rotor = R_rotor_lidar * pt_lidar + Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d pt_base = Eigen::AngleAxisd(rot_angular * M_PI /180, Eigen::Vector3d::UnitZ()) * pt_rotor;
        
        // 添加变换后的点到点云
        pcl::PointXYZI transformed_point;
        transformed_point.x = pt_base[0];
        transformed_point.y = pt_base[1];
        transformed_point.z = pt_base[2];
        transformed_point.intensity = point.reflectivity;
        transformed_cloud->points.push_back(transformed_point);
    }
    
    // 设置点云属性
    transformed_cloud->width = transformed_cloud->points.size();
    transformed_cloud->height = 1;
    transformed_cloud->is_dense = true;
    
    // 转换回ROS消息并发布
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*transformed_cloud, output_msg);
    output_msg.header.stamp = msg->header.stamp;
    output_msg.header.frame_id = "motor_base";  // 基座坐标系
    
    base_pointcloud_pub.publish(output_msg);
    
    ROS_DEBUG("Transformed pointcloud with %d points using per-point interpolation", (int)transformed_cloud->points.size());
}

void rotor_encoder_callback(const fast_lio::StampedInt32::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(encoder_mutex);
    
    // 转换编码器值为角度（度）
    double new_encoder_value = msg->data * 360.0 / 65535.0;
    
    rotor_encoder_value = new_encoder_value;
    double current_timestamp = msg->header.stamp.toSec();
    
    EncoderData new_data;
    new_data.timestamp = current_timestamp;
    new_data.encoder_value = rotor_encoder_value; // 保持度数，在使用时转换为弧度

    if(encoder_queue.size() >0 && fabs(new_data.encoder_value - encoder_queue.back().encoder_value) < 1e-6)
    {
        return;
    }
    
    encoder_queue.push_back(new_data);
    
    ROS_DEBUG("Received encoder data: t=%.6f, raw=%d, angle=%.3f°, queue_size=%d", 
             current_timestamp, msg->data, rotor_encoder_value, (int)encoder_queue.size());
    
    // 维护队列大小
    while (encoder_queue.size() > MAX_ENCODER_QUEUE_SIZE) {
        encoder_queue.pop_front();
    }
    
    // 移除过时数据
    double cutoff_time = current_timestamp - MAX_INTERPOLATION_TIME * 2; // 增加保留时间
    while (!encoder_queue.empty() && encoder_queue.front().timestamp < cutoff_time) {
        encoder_queue.pop_front();
    }
    
    encoder_initialized = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_to_base");
    ros::NodeHandle nh("~");
    
    // 检查是否有测试模式参数
    bool test_mode = false;
    nh.param("test_mode", test_mode, false);
 
    ROS_INFO("Starting lidar_to_base node...");
    if (test_mode) {
        ROS_INFO("Running in test mode - will use simulated encoder data");
    }
    
    // 订阅原始点云（来自激光雷达传感器）
    ros::Subscriber pointcloud_sub = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 10, pointcloud_callback);
    ROS_INFO("Subscribed to /livox/lidar");
    
    // // 订阅里程计（如果还需要位姿信息）
    // ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("/livox/imu", 200, vins_callback);
    // ROS_INFO("Subscribed to /livox/imu");

    // 订阅编码器
    ros::Subscriber rotor_encoder_sub = nh.subscribe<fast_lio::StampedInt32>("/rotor_encoder", 10, rotor_encoder_callback);
    ROS_INFO("Subscribed to /rotor_encoder");
    
    // 等待一段时间确保编码器数据开始接收
    ROS_INFO("Waiting for encoder data...");
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    
    // 检查是否有编码器数据
    {
        std::lock_guard<std::mutex> lock(encoder_mutex);
        if (encoder_queue.empty()) {
            if (!test_mode) {
                ROS_WARN("No encoder data received after waiting. Will use static angle of 0 degrees.");
                ROS_WARN("Make sure /rotor_encoder topic is being published.");
                ROS_WARN("You can also run with _test_mode:=true to use simulated encoder data.");
            } else {
                ROS_INFO("Test mode: Using simulated encoder data starting at 0 degrees");
            }
            // 初始化默认编码器值
            encoder_initialized = true;
            last_used_encoder_value = 0.0;
        } else {
            ROS_INFO("Encoder data received successfully. Queue size: %d", (int)encoder_queue.size());
        }
    }
 
    // 发布基座坐标系的点云
    base_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/base_pointcloud", 10);
    ROS_INFO("Publishing to /base_pointcloud");
    
    ROS_INFO("Node initialization complete");
 
    ros::Rate rate(20.0);
 
    while(ros::ok()){
        // 每隔一段时间检查编码器队列状态
        {
            std::lock_guard<std::mutex> lock(encoder_mutex);
            if (!encoder_queue.empty()) {
                ROS_DEBUG("Encoder queue status: size=%d, latest_time=%.6f, latest_angle=%.1f°",
                         (int)encoder_queue.size(), 
                         encoder_queue.back().timestamp, 
                         encoder_queue.back().encoder_value);
            }
        }
        
        ROS_INFO("Base position: x=%.3f, y=%.3f, z=%.3f | Encoder: %.1f°", 
                 p_base[0], p_base[1], p_base[2], rotor_encoder_value);

        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}