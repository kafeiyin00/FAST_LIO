#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>
#include <queue>
#include <deque>
#include <std_msgs/Int32.h>
#include <fast_lio/StampedInt32.h>
#include <mutex>
#include <algorithm>

// 存储编码器数据的结构体
struct EncoderData {
    double timestamp;
    double encoder_value;
    
    EncoderData() : timestamp(0.0), encoder_value(0.0) {}
    EncoderData(double t, double v) : timestamp(t), encoder_value(v) {}
};

Eigen::Vector3d p_lidar_body, p_enu;
Eigen::Quaterniond q_w_lidar,q_rotorframe_lidar,q_rotor_rotorframe, q_mav_rotor;
Eigen::Quaterniond q_mav;

// 编码器数据队列和互斥锁
std::deque<EncoderData> encoder_queue;
std::mutex encoder_mutex;
const size_t MAX_ENCODER_QUEUE_SIZE = 1000;  // 最大队列大小
const double MAX_INTERPOLATION_TIME = 1.0;   // 最大插值时间范围（秒）

double rotor_encoder_value = 0; // Store the latest encoder value

double fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

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
    
    // 如果目标时间在队列范围之后
    if (it_upper == encoder_queue.end()) {
        double time_diff = target_timestamp - encoder_queue.back().timestamp;
        if (time_diff > MAX_INTERPOLATION_TIME) {
            ROS_WARN("Target timestamp %.6f is too new (%.3fs after last data), using last available value", 
                     target_timestamp, time_diff);
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

void vins_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    p_lidar_body = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    q_w_lidar = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    
    // 使用消息的时间戳进行编码器值插值
    double msg_timestamp = msg->header.stamp.toSec();
    double interpolated_encoder_value = interpolateEncoderValue(msg_timestamp);
    
    q_rotor_rotorframe = Eigen::AngleAxisd(-65 *M_PI/180.0, Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd(interpolated_encoder_value *M_PI/180.0, Eigen::Vector3d::UnitZ());
    q_rotorframe_lidar = Eigen::AngleAxisd(-60 *M_PI/180.0, Eigen::Vector3d::UnitY());
    q_mav_rotor = Eigen::AngleAxisd(90 *M_PI/180.0, Eigen::Vector3d::UnitY());
    q_mav = q_w_lidar * q_rotorframe_lidar.inverse() * q_rotor_rotorframe.inverse() *q_mav_rotor.inverse();
    
    ROS_DEBUG("Using interpolated encoder value %.3f for timestamp %.6f", 
              interpolated_encoder_value, msg_timestamp);
}



void rotor_encoder_callback(const fast_lio::StampedInt32::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(encoder_mutex);
    
    rotor_encoder_value = msg->data*360/65535.0;
    
    // 使用消息header中的时间戳
    double current_timestamp = msg->header.stamp.toSec();
    
    // 创建新的编码器数据
    EncoderData new_data;
    new_data.timestamp = current_timestamp;
    new_data.encoder_value = rotor_encoder_value;
    
    // 添加到队列
    encoder_queue.push_back(new_data);
    
    // 维护队列大小
    while (encoder_queue.size() > MAX_ENCODER_QUEUE_SIZE) {
        encoder_queue.pop_front();
    }
    
    // 清理过时数据（超过最大插值时间范围的数据）
    double cutoff_time = current_timestamp - MAX_INTERPOLATION_TIME;
    while (!encoder_queue.empty() && encoder_queue.front().timestamp < cutoff_time) {
        encoder_queue.pop_front();
    }
    // ROS_DEBUG("Stored encoder value %.3f at timestamp %.6f, queue size: %zu", 
    //           rotor_encoder_value, current_timestamp, encoder_queue.size());  
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_to_mavros");
    ros::NodeHandle nh("~");
 
    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("/lidar_slam/imu_propagate", 200, vins_callback);

    // Subscribe to /rotor_encoder
    ros::Subscriber rotor_encoder_sub = nh.subscribe<fast_lio::StampedInt32>("/rotor_encoder", 10, rotor_encoder_callback);
 
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    //ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/odometry/in", 10);
 
 
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
 
    ros::Time last_request = ros::Time::now();
    while(ros::ok()){
        // Directly transform to PX4 coordinate system
        geometry_msgs::PoseStamped vision;

        // PX4 ENU: x-forward, y-left, z-up
        // If your lidar_slam is already ENU, you may not need to transform.
        // If not, apply the necessary transformation here.
        // Example: If lidar_slam is NED, convert to ENU:
        // p_enu[0] = p_lidar_body[1];
        // p_enu[1] = p_lidar_body[0];
        // p_enu[2] = -p_lidar_body[2];
        // Otherwise, if already ENU:
        p_enu = p_lidar_body;

        vision.pose.position.x = p_enu[0];
        vision.pose.position.y = p_enu[1];
        vision.pose.position.z = p_enu[2];

        // Directly use q_mav for orientation (assuming it's ENU)
        vision.pose.orientation.x = q_mav.x();
        vision.pose.orientation.y = q_mav.y();
        vision.pose.orientation.z = q_mav.z();
        vision.pose.orientation.w = q_mav.w();

        vision.header.stamp = ros::Time::now();
        vision.header.frame_id = "map"; // or "odom", set as appropriate for your system
        static uint32_t seq = 0;
        vision.header.seq = seq++;

        vision_pub.publish(vision);

        ROS_INFO("\nposition in enu:\n   x: %.18f\n   y: %.18f\n   z: %.18f\norientation of lidar:\n   x: %.18f\n   y: %.18f\n   z: %.18f\n   w: %.18f", \
            p_enu[0],p_enu[1],p_enu[2],q_mav.x(),q_mav.y(),q_mav.z(),q_mav.w());

        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}
