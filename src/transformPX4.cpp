#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>
#include <queue>
#include <std_msgs/Int32.h> // Add this include

Eigen::Vector3d p_lidar_body, p_enu;
Eigen::Quaterniond q_w_lidar,q_rotorframe_lidar,q_rotor_rotorframe;
Eigen::Quaterniond q_mav;

double rotor_encoder_value = 0; // Store the latest encoder value

double fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

void vins_callback(const nav_msgs::Odometry::ConstPtr &msg)
{

    p_lidar_body = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    q_w_lidar = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    
    q_rotor_rotorframe = Eigen::AngleAxisd(rotor_encoder_value *M_PI/180.0, Eigen::Vector3d::UnitZ());
    q_rotorframe_lidar = Eigen::AngleAxisd(-60 *M_PI/180.0, Eigen::Vector3d::UnitY());
    q_mav = q_w_lidar * q_rotorframe_lidar.inverse() * q_rotor_rotorframe.inverse();
}



void rotor_encoder_callback(const std_msgs::Int32::ConstPtr& msg)
{
    rotor_encoder_value = msg->data*360/65535.0;
    ROS_INFO("rotor_encoder_value: %.3f", rotor_encoder_value);
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_to_mavros");
    ros::NodeHandle nh("~");
 
    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("/lidar_slam/imu_propagate", 200, vins_callback);

    // Subscribe to /rotor_encoder
    ros::Subscriber rotor_encoder_sub = nh.subscribe<std_msgs::Int32>("/rotor_encoder", 10, rotor_encoder_callback);
 
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
 
 
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(200.0);
 
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