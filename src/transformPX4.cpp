#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
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
ros::Publisher vision_pub;

// TF 参数全局变量
double rotorframe_to_lidar_rx = 0.0;  // 绕X轴旋转角度 (度)
double rotorframe_to_lidar_ry = -60.0;    // 绕Y轴旋转角度 (度) 
double rotorframe_to_lidar_rz = 0.0;    // 绕Z轴旋转角度 (度)
double rotorframe_to_lidar_tx = 0.0;    // X轴平移
double rotorframe_to_lidar_ty = 0.0;    // Y轴平移
double rotorframe_to_lidar_tz = 0.05;    // Z轴平移

double body_to_rotor_rx = 0.0;          // 绕X轴旋转角度 (度)
double body_to_rotor_ry = 90.0;         // 绕Y轴旋转角度 (度)
double body_to_rotor_rz = 0.0;          // 绕Z轴旋转角度 (度)
double body_to_rotor_tx = 0.1;          // X轴平移
double body_to_rotor_ty = 0.0;          // Y轴平移
double body_to_rotor_tz = 0.0;          // Z轴平移

double rotor_rotorframe_offset_deg = -65.0;  // rotor到rotorframe的固定偏移角度 (度)

// 静态变换矩阵 (全局变量)
Eigen::Matrix4d T_rotorframe_lidar = Eigen::Matrix4d::Identity();  // rotorframe -> lidar 静态变换
Eigen::Matrix4d T_body_rotor = Eigen::Matrix4d::Identity();        // body -> rotor 静态变换

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

// 从RPY角度(度)创建四元数
Eigen::Quaterniond quaternionFromRPY(double roll_deg, double pitch_deg, double yaw_deg)
{
    double roll = roll_deg * M_PI / 180.0;
    double pitch = pitch_deg * M_PI / 180.0;
    double yaw = yaw_deg * M_PI / 180.0;
    
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    
    return yawAngle * pitchAngle * rollAngle;
}

// 初始化静态变换矩阵
void initializeStaticTransforms()
{
    // 初始化 rotorframe -> lidar 变换
    Eigen::Quaterniond q_rotorframe_lidar_param = quaternionFromRPY(rotorframe_to_lidar_rx, rotorframe_to_lidar_ry, rotorframe_to_lidar_rz);
    T_rotorframe_lidar.block<3,3>(0,0) = q_rotorframe_lidar_param.toRotationMatrix();
    T_rotorframe_lidar(0,3) = rotorframe_to_lidar_tx;
    T_rotorframe_lidar(1,3) = rotorframe_to_lidar_ty;
    T_rotorframe_lidar(2,3) = rotorframe_to_lidar_tz;
    
    // 初始化 body -> rotor 变换
    Eigen::Quaterniond q_body_rotor_param = quaternionFromRPY(body_to_rotor_rx, body_to_rotor_ry, body_to_rotor_rz);
    T_body_rotor.block<3,3>(0,0) = q_body_rotor_param.toRotationMatrix();
    T_body_rotor(0,3) = body_to_rotor_tx;
    T_body_rotor(1,3) = body_to_rotor_ty;
    T_body_rotor(2,3) = body_to_rotor_tz;
    
    ROS_INFO("Static transforms initialized:");
    ROS_INFO("  T_rotorframe_lidar and T_body_rotor are ready for use");
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

void vins_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    {
        std::lock_guard<std::mutex> lock(encoder_mutex);
    
        if (encoder_queue.size() < 5) {
            ROS_WARN("Encoder queue is too small");
            return;
        }
    }
    p_lidar_body = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    q_w_lidar = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    
    // 使用消息的时间戳进行编码器值插值
    double msg_timestamp = msg->header.stamp.toSec();
    double interpolated_encoder_value = interpolateEncoderValue(msg_timestamp);
    
    // 计算完整的变换矩阵而非仅旋转
    
    // 1. 创建 rotor -> rotorframe 变换 (包含固定偏移 + 编码器动态旋转)
    Eigen::Matrix4d T_rotor_rotorframe = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q_offset(Eigen::AngleAxisd(rotor_rotorframe_offset_deg * M_PI/180.0, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q_encoder(Eigen::AngleAxisd(interpolated_encoder_value * M_PI/180.0, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q_rotor_rotorframe_combined = q_offset * q_encoder;
    T_rotor_rotorframe.block<3,3>(0,0) = q_rotor_rotorframe_combined.toRotationMatrix();
    // 无平移，保持 T_rotor_rotorframe(0:2,3) = [0,0,0]
    
    // 2. 使用全局静态变换 T_rotorframe_lidar (rotorframe -> lidar)
    // 3. 使用全局静态变换 T_body_rotor (body -> rotor)
    
    // 4. 从里程计获取 world -> lidar 变换
    Eigen::Matrix4d T_w_lidar = Eigen::Matrix4d::Identity();
    T_w_lidar.block<3,3>(0,0) = q_w_lidar.toRotationMatrix();
    T_w_lidar(0,3) = p_lidar_body.x();
    T_w_lidar(1,3) = p_lidar_body.y();
    T_w_lidar(2,3) = p_lidar_body.z();
    
    // 5. 计算复合变换: T_w_body = T_w_lidar * T_lidar_rotorframe * T_rotorframe_rotor * T_rotor_body
    // 注意：需要使用逆变换来得到正确的变换链
    Eigen::Matrix4d T_w_body = T_w_lidar * T_rotorframe_lidar.inverse() * T_rotor_rotorframe.inverse() * T_body_rotor.inverse();
    
    // 6. 从变换矩阵中提取旋转和平移
    Eigen::Matrix3d R_w_body = T_w_body.block<3,3>(0,0);
    Eigen::Vector3d t_w_body = T_w_body.block<3,1>(0,3);
    
    // 将旋转矩阵转换为四元数
    q_mav = Eigen::Quaterniond(R_w_body);
    p_enu = t_w_body;

    // q_rotor_rotorframe = Eigen::AngleAxisd(-65 *M_PI/180.0, Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd(interpolated_encoder_value *M_PI/180.0, Eigen::Vector3d::UnitZ());
    // q_rotorframe_lidar = Eigen::AngleAxisd(-60 *M_PI/180.0, Eigen::Vector3d::UnitY());
    // q_mav_rotor = Eigen::AngleAxisd(90 *M_PI/180.0, Eigen::Vector3d::UnitY());
    // q_mav = q_w_lidar * q_rotorframe_lidar.inverse() * q_rotor_rotorframe.inverse() *q_mav_rotor.inverse();
    
    ROS_DEBUG("Using interpolated encoder value %.3f for timestamp %.6f", 
              interpolated_encoder_value, msg_timestamp);


    geometry_msgs::PoseStamped vision;
    // p_enu 已经在上面的变换计算中设置了

    vision.pose.position.x = p_enu[0];
    vision.pose.position.y = p_enu[1];
    vision.pose.position.z = p_enu[2];

    // Directly use q_mav for orientation (assuming it's ENU)
    vision.pose.orientation.x = q_mav.x();
    vision.pose.orientation.y = q_mav.y();
    vision.pose.orientation.z = q_mav.z();
    vision.pose.orientation.w = q_mav.w();

    vision.header.stamp = msg->header.stamp;
    vision.header.frame_id = "map";
    static uint32_t seq = 0;
    vision.header.seq = seq++;
    vision_pub.publish(vision);

    // 发布 TF transforms
    static tf2_ros::TransformBroadcaster tf_broadcaster;
    std::vector<geometry_msgs::TransformStamped> transforms;
    
    // 1. 发布 map -> body (从 T_w_body)
    geometry_msgs::TransformStamped map_to_body;
    map_to_body.header.stamp = vision.header.stamp;
    map_to_body.header.frame_id = "map";
    map_to_body.child_frame_id = "body";
    map_to_body.transform.translation.x = T_w_body(0,3);
    map_to_body.transform.translation.y = T_w_body(1,3);
    map_to_body.transform.translation.z = T_w_body(2,3);
    Eigen::Quaterniond q_map_body(T_w_body.block<3,3>(0,0));
    map_to_body.transform.rotation.x = q_map_body.x();
    map_to_body.transform.rotation.y = q_map_body.y();
    map_to_body.transform.rotation.z = q_map_body.z();
    map_to_body.transform.rotation.w = q_map_body.w();
    transforms.push_back(map_to_body);
    
    // 2. 发布 body -> rotor (从 T_body_rotor)
    geometry_msgs::TransformStamped body_to_rotor_tf;
    body_to_rotor_tf.header.stamp = vision.header.stamp;
    body_to_rotor_tf.header.frame_id = "body";
    body_to_rotor_tf.child_frame_id = "rotor";
    body_to_rotor_tf.transform.translation.x = T_body_rotor(0,3);
    body_to_rotor_tf.transform.translation.y = T_body_rotor(1,3);
    body_to_rotor_tf.transform.translation.z = T_body_rotor(2,3);
    Eigen::Quaterniond q_body_rotor_tf(T_body_rotor.block<3,3>(0,0));
    body_to_rotor_tf.transform.rotation.x = q_body_rotor_tf.x();
    body_to_rotor_tf.transform.rotation.y = q_body_rotor_tf.y();
    body_to_rotor_tf.transform.rotation.z = q_body_rotor_tf.z();
    body_to_rotor_tf.transform.rotation.w = q_body_rotor_tf.w();
    transforms.push_back(body_to_rotor_tf);
    
    // 3. 发布 rotor -> rotorframe (从 T_rotor_rotorframe)
    geometry_msgs::TransformStamped rotor_to_rotorframe_tf;
    rotor_to_rotorframe_tf.header.stamp = vision.header.stamp;
    rotor_to_rotorframe_tf.header.frame_id = "rotor";
    rotor_to_rotorframe_tf.child_frame_id = "rotorframe";
    rotor_to_rotorframe_tf.transform.translation.x = T_rotor_rotorframe(0,3);
    rotor_to_rotorframe_tf.transform.translation.y = T_rotor_rotorframe(1,3);
    rotor_to_rotorframe_tf.transform.translation.z = T_rotor_rotorframe(2,3);
    Eigen::Quaterniond q_rotor_rotorframe_tf(T_rotor_rotorframe.block<3,3>(0,0));
    rotor_to_rotorframe_tf.transform.rotation.x = q_rotor_rotorframe_tf.x();
    rotor_to_rotorframe_tf.transform.rotation.y = q_rotor_rotorframe_tf.y();
    rotor_to_rotorframe_tf.transform.rotation.z = q_rotor_rotorframe_tf.z();
    rotor_to_rotorframe_tf.transform.rotation.w = q_rotor_rotorframe_tf.w();
    transforms.push_back(rotor_to_rotorframe_tf);
    
    // 4. 发布 rotorframe -> lidar (从 T_rotorframe_lidar)
    geometry_msgs::TransformStamped rotorframe_to_lidar_tf;
    rotorframe_to_lidar_tf.header.stamp = vision.header.stamp;
    rotorframe_to_lidar_tf.header.frame_id = "rotorframe";
    rotorframe_to_lidar_tf.child_frame_id = "lidar";
    rotorframe_to_lidar_tf.transform.translation.x = T_rotorframe_lidar(0,3);
    rotorframe_to_lidar_tf.transform.translation.y = T_rotorframe_lidar(1,3);
    rotorframe_to_lidar_tf.transform.translation.z = T_rotorframe_lidar(2,3);
    Eigen::Quaterniond q_rotorframe_lidar_tf(T_rotorframe_lidar.block<3,3>(0,0));
    rotorframe_to_lidar_tf.transform.rotation.x = q_rotorframe_lidar_tf.x();
    rotorframe_to_lidar_tf.transform.rotation.y = q_rotorframe_lidar_tf.y();
    rotorframe_to_lidar_tf.transform.rotation.z = q_rotorframe_lidar_tf.z();
    rotorframe_to_lidar_tf.transform.rotation.w = q_rotorframe_lidar_tf.w();
    transforms.push_back(rotorframe_to_lidar_tf);
    
    // 发布所有变换
    tf_broadcaster.sendTransform(transforms);

    ROS_INFO("\nposition in enu:\n   x: %.18f\n   y: %.18f\n   z: %.18f\norientation of lidar:\n   x: %.18f\n   y: %.18f\n   z: %.18f\n   w: %.18f", \
        p_enu[0],p_enu[1],p_enu[2],q_mav.x(),q_mav.y(),q_mav.z(),q_mav.w());
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

    // 读取TF参数
    nh.param("rotorframe_to_lidar_rx", rotorframe_to_lidar_rx, rotorframe_to_lidar_rx);
    nh.param("rotorframe_to_lidar_ry", rotorframe_to_lidar_ry, rotorframe_to_lidar_ry);
    nh.param("rotorframe_to_lidar_rz", rotorframe_to_lidar_rz, rotorframe_to_lidar_rz);
    nh.param("rotorframe_to_lidar_tx", rotorframe_to_lidar_tx, rotorframe_to_lidar_tx);
    nh.param("rotorframe_to_lidar_ty", rotorframe_to_lidar_ty, rotorframe_to_lidar_ty);
    nh.param("rotorframe_to_lidar_tz", rotorframe_to_lidar_tz, rotorframe_to_lidar_tz);

    nh.param("body_to_rotor_rx", body_to_rotor_rx, body_to_rotor_rx);
    nh.param("body_to_rotor_ry", body_to_rotor_ry, body_to_rotor_ry);
    nh.param("body_to_rotor_rz", body_to_rotor_rz, body_to_rotor_rz);
    nh.param("body_to_rotor_tx", body_to_rotor_tx, body_to_rotor_tx);
    nh.param("body_to_rotor_ty", body_to_rotor_ty, body_to_rotor_ty);
    nh.param("body_to_rotor_tz", body_to_rotor_tz, body_to_rotor_tz);

    nh.param("rotor_rotorframe_offset_deg", rotor_rotorframe_offset_deg, rotor_rotorframe_offset_deg);

    // 打印读取的参数
    ROS_INFO("TF Parameters:");
    ROS_INFO("  rotorframe->lidar: tx=%.2f, ty=%.2f, tz=%.2f, rx=%.2f°, ry=%.2f°, rz=%.2f°",
             rotorframe_to_lidar_tx, rotorframe_to_lidar_ty, rotorframe_to_lidar_tz,
             rotorframe_to_lidar_rx, rotorframe_to_lidar_ry, rotorframe_to_lidar_rz);
    ROS_INFO("  body->rotor: tx=%.2f, ty=%.2f, tz=%.2f, rx=%.2f°, ry=%.2f°, rz=%.2f°",
             body_to_rotor_tx, body_to_rotor_ty, body_to_rotor_tz,
             body_to_rotor_rx, body_to_rotor_ry, body_to_rotor_rz);
    ROS_INFO("  rotor->rotorframe offset: %.2f°", rotor_rotorframe_offset_deg);

    // 初始化静态变换矩阵
    initializeStaticTransforms();
 
    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("/lidar_slam/imu_propagate", 200, vins_callback);

    // Subscribe to /rotor_encoder
    ros::Subscriber rotor_encoder_sub = nh.subscribe<fast_lio::StampedInt32>("/rotor_encoder", 10, rotor_encoder_callback);
 
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    //ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/odometry/in", 10);

    ROS_INFO("Static transforms will be published dynamically as part of the transform chain");
    ros::Rate rate(20.0);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}
