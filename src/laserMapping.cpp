// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
// 参考地图模式读取任意 PCD 字段布局时需要先落到 PCLPointCloud2
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

// xinzhao
#include <geometry_msgs/QuaternionStamped.h>


#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
// kdtree_incremental_time为kdtree建立时间，kdtree_search_time为kdtree搜索时间，kdtree_delete_time为kdtree删除时间;
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
// T1为雷达初始时间戳，s_plot为整个流程耗时，s_plot2特征点数量,s_plot3为kdtree增量时间，s_plot4为kdtree搜索耗时，s_plot5为kdtree删除点数量
//，s_plot6为kdtree删除耗时，s_plot7为kdtree初始大小，s_plot8为kdtree结束大小,s_plot9为平均消耗时间，s_plot10为添加点数量，s_plot11为点云预处理的总时间
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
// 定义全局变量，用于记录时间,match_time为匹配时间，solve_time为求解时间，solve_const_H_time为求解H矩阵时间
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
// kdtree_size_st为ikd-tree获得的节点数，kdtree_size_end为ikd-tree结束时的节点数，add_point_size为添加点的数量，kdtree_delete_counter为删除点的数量
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
// runtime_pos_log运行时的log是否开启，pcd_save_en是否保存pcd文件，time_sync_en是否同步时间
bool   runtime_pos_log = false, pcd_save_en = false, trajectory_save_en = true, time_sync_en = false, extrinsic_est_en = true, path_en = true;
bool   pcd_save_each_frame_en = false;
string pcd_root_path;
string map_dir_path;
FILE* file_trajectory = nullptr;
/**************************/

float res_last[100000] = {0.0};           //残差，点到面距离平方和
float DET_RANGE = 300.0f;                 //设置的当前雷达系中心到各个地图边缘的距离
const float MOV_THRESHOLD = 1.5f;         //设置的当前雷达系中心到各个地图边缘的权重
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer,mtx_buffer_imu_prop;     // 互斥锁
condition_variable sig_buffer;            // 条件变量

string root_dir = ROOT_DIR;               //设置根目录
string map_file_path, lid_topic, imu_topic; //设置地图文件路径，雷达topic，imu topic

double res_mean_last = 0.05, total_residual = 0.0;                                                 //设置残差平均值，残差总和
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;                                        //设置雷达时间戳，imu时间戳
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;                       //设置imu的角速度协方差，加速度协方差，角速度协方差偏置，加速度协方差偏置
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0; //设置滤波器的最小尺寸，地图的最小尺寸，视野角度
//设置立方体长度，视野一半的角度，视野总角度，总距离，雷达结束时间，雷达初始时间
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
//设置有效特征点数，时间log计数器, scan_count：接收到的激光雷达Msg的总数，publish_count：接收到的IMU的Msg的总数
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
//设置迭代次数，下采样的点数，最大迭代次数，有效点数
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0}; // 是否为平面特征点
// lidar_pushed：用于判断激光雷达数据是否从缓存队列中拿到meas中的数据, flg_EKF_inited用于判断EKF是否初始化完成
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
//设置是否发布激光雷达数据，是否发布稠密数据，是否发布激光雷达数据的身体数据
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

// ==================== BEGIN: Reference Map Mode Globals ====================
// ==================== 参考地图定位模式全局参数 ====================
// 下面这组参数统一服务于三种模式：
//   1) no_ref             : 原始在线建图
//   2) ref_pub_grav_truth : 参考图定位 + 发布 G 系点云 + 发布固定 T^W_G
//   3) ref_pub_ref_truth  : 参考图定位 + 直接发布 W 系点云与位姿 + 发布单位阵
// 设计原则：
//   - 匹配链路尽量共用，避免复制一套 EKF/ICP 主流程
//   - 通过发布层分叉，显式区分 G 系评估输出和 W 系评估输出
// ref_mode: 参考地图工作模式，固定取值为
//   no_ref              : 完全沿用原始 FAST-LIO 在线建图与发布逻辑
//   ref_pub_grav_truth  : 使用 TLS 参考图定位，但继续发布重力对齐 G 系点云，同时额外发布固定 T^W_G
//   ref_pub_ref_truth   : 使用 TLS 参考图定位，并直接发布 W/TLS 系点云与里程计，同时发布单位阵
// ref_map_publish_en: 是否发布参考地图到 /ref_map 方便 RViz 观察
const string REF_MODE_NO_REF = "no_ref";
const string REF_MODE_PUB_GRAV_TRUTH = "ref_pub_grav_truth";
const string REF_MODE_PUB_REF_TRUTH = "ref_pub_ref_truth";
string ref_mode = REF_MODE_NO_REF;
bool   ref_map_publish_en = true;
// ref_map_path: TLS 参考地图 PCD 路径
// ref_frame_id: ref_ 系列话题发布时使用的 TLS 坐标系名称
string ref_map_path, ref_frame_id = "tls_map";
// ref_map_downsample_leaf: 参考地图建树前的体素降采样大小
// ref_scan_downsample_leaf: 新模式下当前帧 scan 用于匹配时的轻度降采样大小
double ref_map_downsample_leaf = 0.2, ref_scan_downsample_leaf = 0.2;
// ref_transform_wg_vec: yaml 中按行主序给出的 T_WG 4x4 矩阵
vector<double> ref_transform_wg_vec(16, 0.0);
// ref_T_W_G: G -> W 的外部初值变换，用于把定位结果发布到 TLS 系
// ref_T_G_W: 上式逆矩阵，用于把 TLS 地图先转换回内部匹配坐标链
Matrix4d ref_T_W_G = Matrix4d::Identity();
Matrix4d ref_T_G_W = Matrix4d::Identity();
// ref_map_raw_loaded: TLS 参考图是否已经从磁盘成功加载
// ref_map_tree_initialized: 内部静态参考树是否已经构建完成
bool   ref_map_raw_loaded = false, ref_map_tree_initialized = false;
// ref_map_cloud_w: 原始 TLS 坐标系 W 下的参考地图
// ref_map_cloud_g: 经过 T_GW 变换后的重力对齐 G 系参考地图，用于调试查看
// ref_map_cloud_internal: 进一步去掉 q_Grav_w 后，实际用于内部匹配建树的参考地图
PointCloudXYZI::Ptr ref_map_cloud_w(new PointCloudXYZI());
PointCloudXYZI::Ptr ref_map_cloud_g(new PointCloudXYZI());
PointCloudXYZI::Ptr ref_map_cloud_internal(new PointCloudXYZI());
// ===================== END: Reference Map Mode Globals =====================

// 前向声明：下面的参考图/保存辅助函数需要复用这些接口与缓存。
void RGBpointBodyToWorld(PointType const * const pi, PointType * const po);
extern PointCloudXYZI::Ptr pcl_wait_save;

// 模式判断辅助函数，避免后续分支到处写字符串常量
inline bool is_valid_ref_mode(const string &mode)
{
    return mode == REF_MODE_NO_REF ||
           mode == REF_MODE_PUB_GRAV_TRUTH ||
           mode == REF_MODE_PUB_REF_TRUTH;
}

inline bool is_ref_mode_enabled()
{
    return ref_mode != REF_MODE_NO_REF;
}

inline bool is_ref_pub_grav_truth_mode()
{
    return ref_mode == REF_MODE_PUB_GRAV_TRUTH;
}

inline bool is_ref_pub_ref_truth_mode()
{
    return ref_mode == REF_MODE_PUB_REF_TRUTH;
}

vector<vector<int>>  pointSearchInd_surf;      //每个点的索引,暂时没用到
vector<BoxPointType> cub_needrm;              // ikd-tree中，地图需要移除的包围盒序列
vector<PointVector>  Nearest_Points;           //每个点的最近点序列
vector<double>       extrinT(3, 0.0);               //雷达相对于IMU的外参T
vector<double>       extrinR(9, 0.0);               //雷达相对于IMU的外参R
deque<double>                     time_buffer;                    // 激光雷达数据时间戳缓存队列
deque<PointCloudXYZI::Ptr>        lidar_buffer;      //记录特征提取或间隔采样后的lidar（特征）数据
deque<sensor_msgs::Imu::ConstPtr> imu_buffer; // IMU数据缓存队列

//一些点云变量
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());           //提取地图中的特征点，IKD-tree获得
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());        //去畸变的特征
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());        //畸变纠正后降采样的单帧点云，lidar系
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());       //畸变纠正后降采样的单帧点云，w系
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));       //特征点在地图中对应点的，局部平面参数,w系
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1)); // laserCloudOri是畸变纠正后降采样的单帧点云，body系
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1)); //对应点法相量
PointCloudXYZI::Ptr _featsArray;                                  // ikd-tree中，map需要移除的点云序列

//下采样的体素点云
pcl::VoxelGrid<PointType> downSizeFilterSurf; //单帧内降采样使用voxel grid
pcl::VoxelGrid<PointType> downSizeFilterMap;  //未使用

KD_TREE<PointType> ikdtree;  // ikd-tree类

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);  //雷达相对于body系的X轴方向的点
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0); //雷达相对于world系的X轴方向的点
V3D euler_cur;                                //当前的欧拉角
V3D position_last(Zero3d);                    //上一帧的位置
V3D Lidar_T_wrt_IMU(Zero3d);                  // T lidar to imu (imu = r * lidar + t)
M3D Lidar_R_wrt_IMU(Eye3d);                   // R lidar to imu (imu = r * lidar + t)

/*** EKF inputs and output ***/
// ESEKF操作
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf; // 状态，噪声维度（角速度(3),加速度(3),角速度偏置(3),加速度偏置(3)），输入
state_ikfom state_point;                         // 状态
vect3 pos_lid;                                   // world系下lidar坐标

bool b_q_Grav_w_caclulated = false;
Eigen::Quaterniond q_Grav_w; // transform from world to gravity_world
V3D grav_direction;  //重力参考方向

//输出的路径参数
nav_msgs::Path path;                      //包含了一系列位姿
nav_msgs::Odometry odomAftMapped;         //只包含了一个位姿
nav_msgs::Odometry truthOdomAftMapped;    //新增：TLS/W 系下发布的真值/参考位姿里程计缓存
geometry_msgs::Quaternion geoQuat;        //四元数
geometry_msgs::PoseStamped msg_body_pose; //位姿

//激光和imu处理操作
shared_ptr<Preprocess> p_pre(new Preprocess()); // 定义指向激光雷达数据的预处理类Preprocess的智能指针
shared_ptr<ImuProcess> p_imu(new ImuProcess()); // 定义指向IMU数据预处理类ImuProcess的智能指针

Matrix4d G_T_I0;
V3D unbiased_gyr;

//IMU propagation Parameters
StatesGroup imu_propagate, latest_ekf_state;
bool new_imu{false}, state_update_flg{false}, imu_prop_enable{true}, ekf_finish_once{false};
deque<sensor_msgs::Imu> prop_imu_buffer;
sensor_msgs::Imu newest_imu;
double latest_ekf_time;
nav_msgs::Odometry imu_prop_odom;
ros::Publisher pubImuPropOdom;
string imu_prop_topic;

// ==================== BEGIN: Reference Map Mode Helpers ====================
// 检查 PCD 中是否存在指定字段，用于兼容带/不带 intensity 的参考地图文件
bool hasField(const pcl::PCLPointCloud2 &cloud, const std::string &field_name)
{
    for (const auto &field : cloud.fields)
    {
        if (field.name == field_name)
        {
            return true;
        }
    }
    return false;
}

// 加载 TLS 参考地图到 W 系点云，并根据配置做一次体素降采样
bool load_reference_map_cloud()
{
    if (ref_map_path.empty())
    {
        ROS_ERROR("reference-map mode is enabled but ref_map/pcd_path is empty");
        return false;
    }

    pcl::PCLPointCloud2 cloud_blob;
    if (pcl::io::loadPCDFile(ref_map_path, cloud_blob) < 0)
    {
        ROS_ERROR("Failed to load reference map: %s", ref_map_path.c_str());
        return false;
    }

    PointCloudXYZI::Ptr loaded_cloud(new PointCloudXYZI());
    if (hasField(cloud_blob, "intensity"))
    {
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
        pcl::fromPCLPointCloud2(cloud_blob, pcl_cloud);
        loaded_cloud->reserve(pcl_cloud.size());
        for (const auto &point : pcl_cloud.points)
        {
            PointType point_out;
            point_out.x = point.x;
            point_out.y = point.y;
            point_out.z = point.z;
            point_out.intensity = point.intensity;
            point_out.normal_x = 0.0f;
            point_out.normal_y = 0.0f;
            point_out.normal_z = 0.0f;
            point_out.curvature = 0.0f;
            loaded_cloud->push_back(point_out);
        }
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromPCLPointCloud2(cloud_blob, pcl_cloud);
        loaded_cloud->reserve(pcl_cloud.size());
        for (const auto &point : pcl_cloud.points)
        {
            PointType point_out;
            point_out.x = point.x;
            point_out.y = point.y;
            point_out.z = point.z;
            point_out.intensity = 0.0f;
            point_out.normal_x = 0.0f;
            point_out.normal_y = 0.0f;
            point_out.normal_z = 0.0f;
            point_out.curvature = 0.0f;
            loaded_cloud->push_back(point_out);
        }
    }

    if (ref_map_downsample_leaf > 0.0)
    {
        pcl::VoxelGrid<PointType> ref_map_filter;
        ref_map_filter.setLeafSize(ref_map_downsample_leaf, ref_map_downsample_leaf, ref_map_downsample_leaf);
        ref_map_filter.setInputCloud(loaded_cloud);
        ref_map_filter.filter(*ref_map_cloud_w);
    }
    else
    {
        *ref_map_cloud_w = *loaded_cloud;
    }

    ref_map_raw_loaded = !ref_map_cloud_w->empty();
    ROS_INFO("Loaded reference map with %zu points from %s", ref_map_cloud_w->size(), ref_map_path.c_str());
    return ref_map_raw_loaded;
}

// 解析 yaml 中按行主序给出的 T_WG，并同步计算逆矩阵 T_GW
void parse_reference_transform()
{
    if (ref_transform_wg_vec.size() != 16)
    {
        ROS_WARN("ref_map/transform_wg must contain 16 values, fallback to identity");
        ref_T_W_G = Matrix4d::Identity();
    }
    else
    {
        for (int row = 0; row < 4; ++row)
        {
            for (int col = 0; col < 4; ++col)
            {
                ref_T_W_G(row, col) = ref_transform_wg_vec[row * 4 + col];
            }
        }
    }
    ref_T_G_W = ref_T_W_G.inverse();
}

// 将内部匹配使用的 world 系点变换到 TLS 参考地图坐标系 W
V3D transform_internal_world_to_ref_world(const V3D &point_internal)
{
    V3D point_gravity = q_Grav_w * point_internal;
    return ref_T_W_G.block<3, 3>(0, 0) * point_gravity + ref_T_W_G.block<3, 1>(0, 3);
}

// 将 TLS 参考地图坐标系 W 的点变换回内部匹配使用的 world 系
V3D transform_ref_world_to_internal_world(const V3D &point_ref)
{
    V3D point_gravity = ref_T_G_W.block<3, 3>(0, 0) * point_ref + ref_T_G_W.block<3, 1>(0, 3);
    return q_Grav_w.inverse() * point_gravity;
}

// 在重力对齐矩阵可用后，将 TLS 地图从 W 变换到内部匹配坐标系并建立静态 ikdtree
bool initialize_reference_map_tree()
{
    if (!is_ref_mode_enabled() || ref_map_tree_initialized || !ref_map_raw_loaded || !b_q_Grav_w_caclulated)
    {
        return ref_map_tree_initialized;
    }

    ref_map_cloud_g->clear();
    ref_map_cloud_internal->clear();
    ref_map_cloud_g->reserve(ref_map_cloud_w->size());
    ref_map_cloud_internal->reserve(ref_map_cloud_w->size());
    PointVector ref_points_internal;
    ref_points_internal.reserve(ref_map_cloud_w->size());

    for (const auto &point_w : ref_map_cloud_w->points)
    {
        // W -> G -> internal world：先应用外部矩阵逆，再去掉当前实现里的重力对齐旋转
        V3D point_ref(point_w.x, point_w.y, point_w.z);
        V3D point_gravity = ref_T_G_W.block<3, 3>(0, 0) * point_ref + ref_T_G_W.block<3, 1>(0, 3);
        V3D point_internal = q_Grav_w.inverse() * point_gravity;

        PointType point_g = point_w;
        point_g.x = point_gravity.x();
        point_g.y = point_gravity.y();
        point_g.z = point_gravity.z();
        ref_map_cloud_g->push_back(point_g);

        PointType point_internal_type = point_w;
        point_internal_type.x = point_internal.x();
        point_internal_type.y = point_internal.y();
        point_internal_type.z = point_internal.z();
        ref_map_cloud_internal->push_back(point_internal_type);
        ref_points_internal.push_back(point_internal_type);
    }

    ikdtree.Build(ref_points_internal);
    ref_map_tree_initialized = true;
    ROS_INFO("Reference map tree initialized with %zu points", ref_points_internal.size());
    return true;
}

// 将当前帧点云从 body 系直接变换到 TLS 参考地图坐标系 W，用于 ref 点云发布
void pointBodyToRefWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_internal(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);
    V3D p_ref = transform_internal_world_to_ref_world(p_internal);

    po->x = p_ref(0);
    po->y = p_ref(1);
    po->z = p_ref(2);
    po->intensity = pi->intensity;
}

template<typename T>
// 将任意 4x4 齐次变换写入 ROS pose 字段，便于在多个发布接口间复用。
void set_pose_from_matrix(const Matrix4d &transform, T &out)
{
    Eigen::Quaterniond q_out(transform.block<3, 3>(0, 0));
    q_out.normalize();

    out.position.x = transform(0, 3);
    out.position.y = transform(1, 3);
    out.position.z = transform(2, 3);
    out.orientation.x = q_out.x();
    out.orientation.y = q_out.y();
    out.orientation.z = q_out.z();
    out.orientation.w = q_out.w();
}

template<typename T>
// 将当前 body 位姿从内部 world 系经 G 系变换后输出到 TLS/W 坐标系。
// 该位姿只服务于 body 在 W 系下的发布，例如 ref_truth 里的 /ref_truth_Odometry。
void set_posestamp_truth_in_ref_world(T &out)
{
    Matrix4d T_G_B = Matrix4d::Identity();
    T_G_B.block<3, 3>(0, 0) = q_Grav_w.toRotationMatrix() * state_point.rot.toRotationMatrix();
    T_G_B.block<3, 1>(0, 3) = q_Grav_w * Eigen::Vector3d(state_point.pos(0), state_point.pos(1), state_point.pos(2));

    Matrix4d T_W_B = ref_T_W_G * T_G_B;
    set_pose_from_matrix(T_W_B, out.pose);
}

// 发布 TLS/W 坐标系下的整帧点云。
// 该输出专门服务于 ref_pub_ref_truth 模式，对应 /ref_truth_cloud_registered。
void publish_ref_truth_cloud(const ros::Publisher &pubRefTruthCloud)
{
    if (!scan_pub_en)
    {
        return;
    }

    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudRef(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        pointBodyToRefWorld(&laserCloudFullRes->points[i], &laserCloudRef->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudRef, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = ref_frame_id;
    pubRefTruthCloud.publish(laserCloudmsg);
}

// 直接发布一帧已经构建好的点云，避免主循环里重复做坐标变换。
void publish_prebuilt_cloud(const ros::Publisher &publisher,
                            const PointCloudXYZI::Ptr &cloud,
                            const string &frame_id)
{
    if (!scan_pub_en || cloud->empty())
    {
        return;
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*cloud, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = frame_id;
    publisher.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

// 构建一帧重力对齐 G 系点云。
// no_ref 与 grav_truth 的最终 map 都基于这个坐标系累计。
PointCloudXYZI::Ptr build_grav_aligned_cloud(const PointCloudXYZI::Ptr &source_cloud)
{
    int size = source_cloud->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&source_cloud->points[i], &laserCloudWorld->points[i]);
    }

    return laserCloudWorld;
}

// 构建一帧 TLS/W 系点云。
// ref_truth 的最终 map 直接基于这个坐标系累计。
PointCloudXYZI::Ptr build_ref_aligned_cloud(const PointCloudXYZI::Ptr &source_cloud)
{
    int size = source_cloud->points.size();
    PointCloudXYZI::Ptr laserCloudRef(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        pointBodyToRefWorld(&source_cloud->points[i], &laserCloudRef->points[i]);
    }

    return laserCloudRef;
}

// 仅负责累计最终整图，节点退出时统一写到 map_dir_path。
void accumulate_cloud_to_final_map(const PointCloudXYZI::Ptr &cloud)
{
    if (!pcd_save_en || cloud->empty())
    {
        return;
    }

    *pcl_wait_save += *cloud;
}

// 仅负责单帧落盘，与最终整图保存解耦。
// 参考版本里，这两类保存逻辑原本都耦合在 publish_frame_world() 内部，核心写法如下：
//
//   if (pcd_save_en)
//   {
//       int size = feats_undistort->points.size();
//       PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
//       for (int i = 0; i < size; i++)
//       {
//           RGBpointBodyToWorld(&feats_undistort->points[i], &laserCloudWorld->points[i]);
//       }
//       *pcl_wait_save += *laserCloudWorld;
//   }
//
//   // jianping: save data for postprocessing
//   if (pcd_save_en)
//   {
//       double current_frame_time = Measures.lidar_end_time;
//       char filename[256];
//       sprintf(filename, "%s/%.3lf.pcd", pcd_root_path.c_str(), current_frame_time);
//       pcl::io::savePCDFileBinary<PointType>(filename, *laserCloudWorld);
//   }
//
// 当前实现将“最终 map 累计”和“单帧 pcd 落盘”拆开，避免 publish_frame_world() 被重复调用时
// 无意中把同一帧重复写进最终 map。
void save_single_frame_cloud(const PointCloudXYZI::Ptr &cloud)
{
    if (!pcd_save_each_frame_en || cloud->empty())
    {
        return;
    }

    double current_frame_time = Measures.lidar_end_time;
    char filename[256];
    sprintf(filename, "%s/%.3lf.pcd", pcd_root_path.c_str(), current_frame_time);
    pcl::io::savePCDFileBinary<PointType>(filename, *cloud);
}

// 发布 TLS/W 坐标系下的 body 里程计。
// 该函数用于发布 body 在 W 系下的位姿输出。
void publish_truth_odometry_in_frame(const ros::Publisher &pubTruthOdom,
                                     const string &frame_id,
                                     const string &child_frame_id)
{
    truthOdomAftMapped.header.frame_id = frame_id;
    truthOdomAftMapped.child_frame_id = child_frame_id;
    truthOdomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    set_posestamp_truth_in_ref_world(truthOdomAftMapped.pose);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        truthOdomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        truthOdomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        truthOdomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        truthOdomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        truthOdomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        truthOdomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }
    pubTruthOdom.publish(truthOdomAftMapped);
}

void publish_truth_odometry_in_ref_world(const ros::Publisher &pubTruthOdom)
{
    publish_truth_odometry_in_frame(pubTruthOdom, ref_frame_id, "body");
}

// 发布与当前点云对应的固定系间关系：
//   - grav_truth: 发布固定 T^W_G
//   - ref_truth : 发布单位阵，表示当前点云已经直接在 W 系表达
// 这里使用 PoseStamped 仅仅是为了携带一个带时间戳的 4x4 位姿矩阵。
// 由于 PoseStamped 没有 child_frame_id，矩阵“从哪个源坐标系到 header.frame_id”
// 由话题名本身约定：
//   - /grav_truth_TWG 表示 G -> W
//   - /ref_truth_TWW  表示 W -> W
void publish_ref_relation_pose(const ros::Publisher &pubRelationPose, const Matrix4d &transform)
{
    geometry_msgs::PoseStamped relation_pose;
    relation_pose.header.frame_id = ref_frame_id;
    relation_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    set_pose_from_matrix(transform, relation_pose.pose);
    pubRelationPose.publish(relation_pose);
}

// 发布原始 TLS 参考地图，便于在 RViz 中直接观察 ref 配准效果
void publish_ref_map(const ros::Publisher &pubRefMap)
{
    if (!ref_map_publish_en || ref_map_cloud_w->empty())
    {
        return;
    }

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*ref_map_cloud_w, map_msg);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = ref_frame_id;
    pubRefMap.publish(map_msg);
}
// ===================== END: Reference Map Mode Helpers =====================

void ConvertTwistToGravFrame(V3D &L0_vel_L) {
    M3D omg_hat;
    omg_hat << SKEW_SYM_MATRX(unbiased_gyr);
    L0_vel_L = G_T_I0.block<3,3>(0,0) * L0_vel_L;
}

void ConvertPoseToGravFrame(V3D &pos, Quaterniond &q) {
    M3D rot = q.toRotationMatrix();
    Matrix4d I0_T_I = Matrix4d::Identity();
    I0_T_I.block<3, 3>(0, 0) = rot;
    I0_T_I.col(3).head(3) = pos;
    Matrix4d T_out = G_T_I0 * I0_T_I;
    pos = T_out.col(3).head(3);
    rot = T_out.block<3, 3>(0, 0);
    q = Quaterniond(rot);
}


// IMU单步状态传播函数
// 根据IMU测量值和时间增量对系统状态进行一步离散积分
// 参数说明：
//   imu_prop_state: 系统状态（含位置、速度、旋转、重力、偏置）
//   dt: 时间增量(秒)，典型值为 0.005s (200Hz IMU)
//   acc_avr: body系中的平均加速度测量值(m/s²)
//   angvel_avr: body系中的平均角速度测量值(rad/s)
void prop_imu_once(StatesGroup & imu_prop_state,
                   const double dt,
                   V3D acc_avr,
                   V3D angvel_avr) {
    // ===== 第一步：IMU测量值校准 =====
    // 补偿IMU硬件的标度因子误差和零偏
    double mean_acc_norm = p_imu->IMU_mean_acc_norm;  // 初始化时测得的"重力"大小
    // 加速度标度因子补偿：实测值 * (标准重力/实际测得重力) - 加速度零偏
    acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
    // 角速度偏置补偿：移除陀螺仪的零点漂移
    angvel_avr -= imu_prop_state.bias_g;
    unbiased_gyr = angvel_avr;  // 保存无偏差角速度，用于后续重力坐标系转换
    
    // ===== 第二步：旋转状态更新（使用罗德里格斯公式） =====
    // 根据角速度计算旋转矩阵增量Δt时间内的旋转变化
    // Exp(ω, dt) = I + sin(θ)K + (1-cos(θ))K²，其中θ = |ω|*dt，K为ω的斜对称矩阵
    M3D Exp_f = Exp(angvel_avr, dt);
    // 当前时刻的姿态 = 前一时刻姿态 × 旋转增量
    // 注意：这里使用更新后的旋转矩阵计算加速度，存在O(dt)量级误差，但EKF会修正
    imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

    // ===== 第三步：加速度从body系转换到全局坐标系 =====
    // 公式：a_global = R * a_body + g
    // 原因：IMU加速度计测得的是"视觉加速度" = 真实加速度 - 重力
    // 所以：真实加速度 = IMU测值 + 重力加速度
    V3D acc_imu = imu_prop_state.rot_end * acc_avr +  // 将body系加速度转到全局系
                  V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);  // 加上重力加速度

    // ===== 第四步：位置更新（运动学积分） =====
    // 公式：p_new = p_old + v_old*dt + 0.5*a*dt²
    // 三项含义：
    //   imu_prop_state.pos_end: 上一时刻的位置
    //   + imu_prop_state.vel_end * dt: 匀速运动的位移
    //   + 0.5 * acc_imu * dt * dt: 加速度引起的额外位移
    imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

    // ===== 第五步：速度更新 =====
    // 公式：v_new = v_old + a*dt
    // 将加速度对时间积分得到速度变化
    imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

// ==================== 里程计方式1：IMU高频前馈积分 ====================
// 【目的】同样是发布里程计信息供下游使用
// 【特点】基于IMU单传感器的纯数学积分，实时性强但精度受漂移限制
// 【频率】250Hz（由定时器驱动，4ms周期触发）
// 【精度】中等 - 会随时间累积IMU积分误差（INS漂移）
// 【延迟】极低，约<5ms（仅需简单加速度/角速度积分）
// 【约束】无外部观测约束，纯动力学模型
// 
// 与publish_odometry()的对比：
//   本函数(IMU): 250Hz + 低精度 + 超低延迟 → 实时控制反馈、可视化预览
//   另一函数(EKF): 10Hz + 高精度 + 正常延迟 → 精确定位、位姿图优化
void imu_prop_callback(const ros::TimerEvent &e) {
    // ===== 系统就绪检查 =====
    // 等待IMU初始化完成、新IMU数据到达、EKF至少更新过一次
    if (p_imu->imu_need_init_ || !new_imu || !ekf_finish_once) {
        return;
    }
    
    mtx_buffer_imu_prop.lock();
    new_imu = false; //控制propagate频率和IMU频率一致
    if (imu_prop_enable && !prop_imu_buffer.empty()) {
        static double last_t_from_lidar_end_time = 0;  // 上一次IMU的时间戳（从LiDAR完成时刻算起）
        
        // ===== 工作模式1：EKF刚完成更新（state_update_flg==true） =====
        // 在EKF完成LiDAR点云匹配和状态更新后被触发
        // 目的：从LiDAR时刻一路向前积分到当前时刻
        if (state_update_flg) {
            imu_propagate = latest_ekf_state;  // 使用最新的EKF状态作为积分起点
            
            // 删除缓冲中所有时间早于EKF更新时刻的IMU数据（过期数据）
            while ((!prop_imu_buffer.empty() && prop_imu_buffer.front().header.stamp.toSec() < latest_ekf_time)) {
                prop_imu_buffer.pop_front();
            }
            
            last_t_from_lidar_end_time = 0;  // 重置时间基准
            
            // 批量处理缓冲中所有有效IMU，逐步积分
            for (int i = 0; i < prop_imu_buffer.size(); i++) {
                // 当前IMU相对于LiDAR完成时刻的时间偏移
                double t_from_lidar_end_time = prop_imu_buffer[i].header.stamp.toSec() - latest_ekf_time;
                // 两条IMU之间的时间间隔（5ms@200Hz）
                double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
                //cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
                // 提取加速度测量值（body系）
                V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x,
                            prop_imu_buffer[i].linear_acceleration.y,
                            prop_imu_buffer[i].linear_acceleration.z);
                // 提取角速度测量值（body系）
                V3D omg_imu(prop_imu_buffer[i].angular_velocity.x,
                            prop_imu_buffer[i].angular_velocity.y,
                            prop_imu_buffer[i].angular_velocity.z);
                
                // 执行单步状态传播（位置、速度、姿态）
                prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
                last_t_from_lidar_end_time = t_from_lidar_end_time;
            }
            state_update_flg = false;  // 复位标志，等待下一次EKF更新
        }
        // ===== 工作模式2：两次LiDAR更新之间（state_update_flg==false） =====
        // 在EKF更新完成后、下一次LiDAR到达前连续调用
        // 目的：快速响应，实时更新里程计，无需等待LiDAR
        else {
            // 提取最新到达的一条IMU数据的加速度（body系）
            V3D acc_imu(newest_imu.linear_acceleration.x,
                        newest_imu.linear_acceleration.y,
                        newest_imu.linear_acceleration.z);
            // 提取最新到达的一条IMU数据的角速度（body系）
            V3D omg_imu(newest_imu.angular_velocity.x,
                        newest_imu.angular_velocity.y,
                        newest_imu.angular_velocity.z);
            
            // 当前IMU相对于LiDAR完成时刻的时间偏移
            double t_from_lidar_end_time = newest_imu.header.stamp.toSec() - latest_ekf_time;
            // 与上次IMU之间的时间增量
            double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
            
            // 执行单步状态传播
            prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
            last_t_from_lidar_end_time = t_from_lidar_end_time;
        }

        // ===== 发布里程计消息 =====
        V3D posi, vel_i;
        Eigen::Quaterniond q;
        
        // 提取传播后的状态
        posi = imu_propagate.pos_end;      // 位置（世界坐标系）
        vel_i = imu_propagate.vel_end;     // 速度（世界坐标系）
        q = Eigen::Quaterniond(imu_propagate.rot_end);  // 姿态（四元数）
        
        // 坐标系转换：普通世界坐标系 → 重力对齐的世界系
        // （考虑重力方向，使Z轴竖直向上）
        ConvertPoseToGravFrame(posi, q);
        ConvertTwistToGravFrame(vel_i);
        
        // 填充ROS Odometry消息
        imu_prop_odom.header.frame_id = "world";
        imu_prop_odom.header.stamp = newest_imu.header.stamp;
        
        // 填充位置信息
        imu_prop_odom.pose.pose.position.x = posi.x();
        imu_prop_odom.pose.pose.position.y = posi.y();
        imu_prop_odom.pose.pose.position.z = posi.z();
        
        // 填充姿态信息（四元数）
        imu_prop_odom.pose.pose.orientation.w = q.w();
        imu_prop_odom.pose.pose.orientation.x = q.x();
        imu_prop_odom.pose.pose.orientation.y = q.y();
        imu_prop_odom.pose.pose.orientation.z = q.z();
        
        // 填充速度信息（线速度）
        imu_prop_odom.twist.twist.linear.x = vel_i.x();
        imu_prop_odom.twist.twist.linear.y = vel_i.y();
        imu_prop_odom.twist.twist.linear.z = vel_i.z();
        
        // 发布到ROS话题 /imu_prop_odom（200Hz频率）
        pubImuPropOdom.publish(imu_prop_odom);

        // ===== 可选：发布TF变换（用于RViz可视化） =====
        // （当前代码被注释，若需启用请反注释下列代码块）
        // 作用：将IMU位姿广播为TF变换，使RViz能显示坐标系
        // 话题：指向无人机坐标系的TF变换
        // static tf::TransformBroadcaster br1;
        // tf::Transform transform;
        // tf::Quaternion q1;
        // transform.setOrigin(tf::Vector3(imu_prop_odom.pose.pose.position.x, \
        //                         imu_prop_odom.pose.pose.position.y, \
        //                         imu_prop_odom.pose.pose.position.z));
        // q1.setW(imu_prop_odom.pose.pose.orientation.w);
        // q1.setX(imu_prop_odom.pose.pose.orientation.x);
        // q1.setY(imu_prop_odom.pose.pose.orientation.y);
        // q1.setZ(imu_prop_odom.pose.pose.orientation.z);
        // transform.setRotation(q1);
        // br1.sendTransform(
        //         tf::StampedTransform(transform, imu_prop_odom.header.stamp, topic_name_prefix + "world",
        //                              "quad" + SetString(drone_id) + "_imu_propagation"));
    }
    mtx_buffer_imu_prop.unlock();  // 释放互斥锁，允许其他线程访问IMU传播变量
}

//按下ctrl+c后唤醒所有线程
void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();   //  会唤醒所有等待队列中阻塞的线程 线程被唤醒后，会通过轮询方式获得锁，获得锁前也一直处理运行状态，不会被再次阻塞。
}
//将fast lio2信息打印到log中
inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    //下面式子最里面的括弧是从雷达到IMU坐标系 然后从設转换到世界坐标系
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
//把点从身体系转到world系
void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
//把点从身体系转到world系
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}
// 含有RGB的点云从身体系转到world系
void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    //jianping
    p_global = q_Grav_w*p_global;

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
// 含有RGB的点云从Lidar系转到IMU系
void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

// ikd-tree中,局部地图的包围盒角点
BoxPointType LocalMap_Points;
// 局部地图是否初始化
bool Localmap_Initialized = false;
// 在拿到eskf前馈结果后，动态调整地图区域，防止地图过大而内存溢出，类似LOAM中提取局部地图的方法
void lasermap_fov_segment()
{
    // 参考地图模式下不维护局部在线地图，因此直接跳过删图逻辑
    if (is_ref_mode_enabled())
    {
        return;
    }
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    // global系下lidar位置
    V3D pos_LiD = pos_lid;
    //初始化局部地图包围盒角点，以为w系下lidar位置为中心,得到长宽高200*200*200的局部地图
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    // 各个方向上Lidar与局部地图边界的距离，或者说是lidar与立方体盒子六个面的距离
    float dist_to_map_edge[3][2];
    bool need_move = false;
    // 当前雷达系中心到各个地图边缘的距离
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        // 与某个方向上的边界距离（例如1.5*300m）太小，标记需要移除need_move，参考论文Fig3
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    // 不需要挪动就直接退回了
    if (!need_move) return;
    // 否则需要计算移动的距离
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    // 新的局部地图盒子边界点
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        //与包围盒最小值边界点距离
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    // 使用Boxs删除指定盒内的点
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}
// 除了AVIA类型之外的雷达点云回调函数，将数据引入到buffer当中
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock(); //加锁
    scan_count ++;
    double preprocess_start_time = omp_get_wtime(); //记录时间
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);                                       //点云预处理
    lidar_buffer.push_back(ptr);                                    //将点云放入缓冲区
    time_buffer.push_back(msg->header.stamp.toSec());               //将时间放入缓冲区
    last_timestamp_lidar = msg->header.stamp.toSec();               //记录最后一个时间
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time; //预处理时间
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
//Livox雷达点云回调函数，时间戳在msg中，而不是header中
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    //检测时间戳是否回溯
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();
    
    //检测IMU和LiDAR是否同步
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }
    
    //自适应计算IMU和LiDAR的时间差
    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

// IMU数据回调函数
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    //根据时间差调整IMU时间戳
    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
    //如果已经设置了时间同步，则使用自适应计算的时间差
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    //检测时间戳是否回溯
    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    //将IMU数据放入缓冲区
    imu_buffer.push_back(msg);
    mtx_buffer.unlock();

    //IMU前馈传播
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init_) {
        prop_imu_buffer.push_back(*msg);
    }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();

    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
// 同步LiDAR和IMU数据，组合成MeasureGroup供处理
// 同步原则：以LiDAR扫描周期为基准，确保每个LiDAR扫描（一个msg）都能找到覆盖的IMU数据
bool sync_packages(MeasureGroup &meas)
{
    //检查缓冲区是否有数据
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        //如果点云太少，使用平均扫描时间
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        //如果点云最后一个点的时间戳太小，使用平均扫描时间
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            //计算实际的扫描结束时间
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            //更新平均扫描时间
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    //检查是否有足够的IMU数据覆盖该扫描周期
    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
// 向ikd-tree中增量添加点云，是点云地图的构建过程
void map_incremental()
{
    // 参考地图模式下只做定位，不向地图中增量写入当前 scan
    if (is_ref_mode_enabled())
    {
        return;
    }
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    //遍历每个下采样后的点
    for (int i = 0; i < feats_down_size; i++)
    {
        /* 将点从body系转到world系 */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* 决定是否需要加入地图 */
        //首先检查该点是否有最近邻点，并且EKF是否初始化
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            //计算该点所在体素的中心坐标
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            //计算该点与滤波器大小最小中心点的距离
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            //如果最近的邻近点距离该点所在体素中心距离太远，则不需要下采样
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            //根据点与所在包围盒中心点的距离，分类是否需要降采样
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    //向ikd-tree中添加下采样后的点
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    //向ikd-tree中添加不需要下采样的点
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
// 发布world系下的点云数据，带重力标定转换
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{
    // 历史说明：参考版本的 publish_frame_world() 除了发布 /cloud_registered 之外，
    // 还曾直接承担最终 map 累计与单帧 pcd 落盘职责；当前版本保留此函数仅用于旧的
    // G 系发布/轨迹输出辅助路径，保存职责已经迁移到主循环中的独立函数。
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        //将点从身体系转到重力对齐的world系
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "map";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }
    if(trajectory_save_en)
    {
        double current_frame_time = Measures.lidar_beg_time;
        Eigen::Quaterniond q(q_Grav_w*state_point.rot.toRotationMatrix());
        Eigen::Vector3d t = q_Grav_w*state_point.pos;
        //body frame in TUM format
        fprintf(file_trajectory,"%10.3f %10.3f %10.3f %10.3f %10.3f %10.3f %10.3f %10.3f\n",
        current_frame_time,
        t(0),t(1),t(2),
        q.x(),q.y(),q.z(),q.w());
        fflush(file_trajectory);
    }

}

// 发布IMU body系下的点云数据，从Lidar系转换到IMU系
void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    //将点从Lidar系转到IMU系
    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "lidar";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

// 发布有效点云（即EKF匹配后的点）在world系中，带重力标定转换
void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    //将有效的原始点云从body系转到world系
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "map";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

// 发布ikd-tree地图中的点云
void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "map";
    pubLaserCloudMap.publish(laserCloudMap);
}

// 将位置和姿态信息设置到输出消息中，应用重力标定转换
template<typename T>
void set_posestamp(T & out)
{
    //jianping:应用重力标定，将位置和姿态转换到重力对齐的坐标系
    
    Eigen::Vector3d pos_grav = q_Grav_w*Eigen::Vector3d(state_point.pos(0),state_point.pos(1),state_point.pos(2));
    Eigen::Quaterniond q_grav(q_Grav_w*state_point.rot.toRotationMatrix());
    q_grav.normalize();

    out.pose.position.x = pos_grav(0);
    out.pose.position.y = pos_grav(1);
    out.pose.position.z = pos_grav(2);
    out.pose.orientation.x = q_grav.x();
    out.pose.orientation.y = q_grav.y();
    out.pose.orientation.z = q_grav.z();
    out.pose.orientation.w = q_grav.w();

    // out.pose.position.x = state_point.pos(0);
    // out.pose.position.y = state_point.pos(1);
    // out.pose.position.z = state_point.pos(2);
    // out.pose.orientation.x = geoQuat.x;
    // out.pose.orientation.y = geoQuat.y;
    // out.pose.orientation.z = geoQuat.z;
    // out.pose.orientation.w = geoQuat.w;
    
}

// ==================== 里程计方式2：EKF滤波后的精准定位 ====================
// 【目的】同样是发布里程计信息供下游使用
// 【特点】基于IMU+LiDAR的多传感器融合EKF，精度高但更新频率受LiDAR限制
// 【频率】~10Hz - 与LiDAR扫描频率一致，每次LiDAR完成点云到地图匹配后发布一次
// 【约束】受点云-地图匹配结果约束，误差被LiDAR观测修正，长期漂移有界
// 【延迟】较大，约100-200ms（需要完整的LiDAR扫描处理）
// 【协方差】包含EKF估计的6×6位姿协方差矩阵（表示定位不确定性）
// 
// 两种方式对比：
//   方式1(IMU): 高频 + 低精度 + 快速反馈 → 用于实时控制、可视化预览
//   方式2(EKF): 低频 + 高精度 + 缓慢反馈 → 用于导航定位、位姿图优化
void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "map";
    odomAftMapped.child_frame_id = "lidar";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    //设置重力对齐后的位置和姿态信息
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    //从EKF中获取协方差矩阵，填充到里程计消息中
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    // static tf::TransformBroadcaster br;
    // tf::Transform                   transform;
    // tf::Quaternion                  q;
    // transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
    //                                 odomAftMapped.pose.pose.position.y, \
    //                                 odomAftMapped.pose.pose.position.z));
    // q.setW(odomAftMapped.pose.pose.orientation.w);
    // q.setX(odomAftMapped.pose.pose.orientation.x);
    // q.setY(odomAftMapped.pose.pose.orientation.y);
    // q.setZ(odomAftMapped.pose.pose.orientation.z);
    // transform.setRotation( q );
    // br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "map", "lidar" ) );
}

// 发布轨迹路径（每10个pose发布一次）
void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "map";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

// EKF测量模型：计算点到平面的残差和Jacobian矩阵，是FAST-LIO最核心的算法函数
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** 最近平面搜索和残差计算 **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        //多线程并行处理每个点
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* 将点从body系转到world系 */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        //如果EKF已收敛，则从ikd-tree中搜索最近邻点
        if (ekfom_data.converge)
        {
            /** 在地图中查找最近的表面点 **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            //检查是否有足够的邻近点且距离合理
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        //拟合包含最近邻点的平面
        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            //计算点到平面的距离（残差）
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            //根据残差和点的距离计算权重
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            //如果权重足够大，则该点被选中用于EKF更新
            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    //统计有效特征点数
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    //如果没有有效点则EKF不进行更新
    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    //为每个有效点计算Jacobian矩阵
    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** 获取最近表面/角点的法向量 ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        //如果估计外参，则添加外参相关的Jacobian项
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** 测量值：点到最近表面/角点的距离 ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

// 主函数：ROS节点初始化、参数加载、消息订阅和发布、线程启动
int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    //加载发布配置参数
    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    //加载EKF迭代次数
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    //加载地图文件路径
    nh.param<string>("map_file_path",map_file_path,"");
    //加载话题名称
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    //时间同步相关参数
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    //点云滤波器大小
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);
    //局部地图立方体边长
    nh.param<double>("cube_side_length",cube_len,200);
    //检测范围
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);
    //视场角度
    nh.param<double>("mapping/fov_degree",fov_deg,180);
    //IMU噪声参数
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);
    //IMU偏置噪声参数
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    //点云预处理参数
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    //特征提取使能
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    //运行时位置日志
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    //外参估计使能
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    //PCD保存和轨迹保存参数
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<bool>("pcd_save/pcd_save_each_frame_en", pcd_save_each_frame_en, false);
    nh.param<bool>("pcd_save/trajectory_save_en",trajectory_save_en,false);
    nh.param<string>("pcd_save/pcd_root_path", pcd_root_path, "/home/workspace/data/pcds");
    nh.param<string>("pcd_save/map_dir_path", map_dir_path, "/home/workspace/data/map.pcd");
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    //外参矩阵
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    // 参考地图定位模式参数：用固定字符串显式选择模式，避免多个布尔开关交叉组合
    nh.param<string>("ref_map/mode", ref_mode, REF_MODE_NO_REF);
    nh.param<string>("ref_map/pcd_path", ref_map_path, "");
    nh.param<double>("ref_map/map_downsample_leaf", ref_map_downsample_leaf, 0.2);
    nh.param<double>("ref_map/scan_downsample_leaf", ref_scan_downsample_leaf, 0.2);
    nh.param<bool>("ref_map/publish_map", ref_map_publish_en, true);
    nh.param<string>("ref_map/frame_id", ref_frame_id, string("tls_map"));
    nh.param<vector<double>>("ref_map/transform_wg", ref_transform_wg_vec, vector<double>());

    // 加载重力参考方向参数（默认为Z轴负方向）
    vector<double> grav_dir;
    nh.param<vector<double>>("mapping/grav_direction", grav_dir, 
                            vector<double>{0.0, 0.0, -1.0});
    grav_direction << grav_dir[0], grav_dir[1], grav_dir[2];
    ROS_INFO("Gravity reference direction: [%.5f, %.5f, %.5f]", 
             grav_direction[0], grav_direction[1], grav_direction[2]);

    if (!is_valid_ref_mode(ref_mode))
    {
        ROS_WARN("Unsupported ref_map/mode: %s, fallback to no_ref", ref_mode.c_str());
        ref_mode = REF_MODE_NO_REF;
    }
    ROS_INFO("Reference-map mode: %s", ref_mode.c_str());

    parse_reference_transform();

    if (is_ref_mode_enabled())
    {
        // 参考图模式下允许继续累计最终 map，但不再输出旧轨迹文件。
        trajectory_save_en = false;
        path_en = false;
        if (!load_reference_map_cloud())
        {
            ROS_ERROR("Reference-map mode %s is enabled but reference map could not be loaded", ref_mode.c_str());
            return -1;
        }
        ROS_INFO("Reference-map mode %s enabled. Final map saving remains available.", ref_mode.c_str());
    }

    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;

    if(pcd_save_en)
    {
        cout<<"pcd_root_path: "<< pcd_root_path<<"\n";
        cout<<"map_dir_path: "<< map_dir_path<<"\n";
    }
    if(pcd_save_each_frame_en)
    {
        cout<<"single-frame pcd output is enabled under: "<< pcd_root_path<<"\n";
    }

    if(trajectory_save_en)
    {
        std::stringstream ss;
        ss << pcd_root_path << "/trajectory_" << ros::Time::now().toNSec()<<".txt";
        file_trajectory = fopen(ss.str().c_str(),"w");
        //cout<<"pcd_root_path: "<< pcd_root_path<<"\n";
        cout<<"ready to write trajectory to : "<< pcd_root_path<<"\n";
    }

    // ==================== 里程计方式1：IMU高频前馈积分 ====================
    // 【目的】同样是发布里程计信息供下游使用
    // 【特点】基于IMU单传感器数据的纯积分，实时性强但精度不高
    // 【频率】250Hz - 远高于LiDAR（~10Hz），在LiDAR帧之间提供密集的位姿更新
    // 【约束】无外部观测约束，会随时间累积积分误差（INS漂移）
    // 【延迟】极低，约<5ms（适合实时控制反馈）
    pubImuPropOdom = nh.advertise<nav_msgs::Odometry> ("/lidar_slam/imu_propagate", 1000);
    
    // 创建定时器以4ms周期（250Hz）触发IMU传播回调函数
    // 每条新IMU到达时就推进一次状态，在两次LiDAR更新间进行持续的高频积分
    ros::Timer imu_prop_timer = nh.createTimer(ros::Duration(0.004), imu_prop_callback);

    
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="map";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    double scan_leaf_size = is_ref_mode_enabled() ? ref_scan_downsample_leaf : filter_size_surf_min;
    downSizeFilterSurf.setLeafSize(scan_leaf_size, scan_leaf_size, scan_leaf_size);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    //设置IMU处理器的外参和噪声参数
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    //初始化EKF迭代器
    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    /*** ROS消息订阅和发布初始化 ***/
    //根据雷达类型选择对应的回调函数
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    //发布处理后的点云和里程计消息
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_copy = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_copy", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/Odometry", 100000);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", 100000);
        // 将两类真值输出在话题名上彻底拆开，避免 grav_truth 与 ref_truth 共用同名输出造成歧义。
        // 这里的约定是：
        //   - grav_truth 仍保留原始 /cloud_registered 与 /Odometry 这一套 G 系输出
        //   - ref_truth  只发布 W/TLS 系输出，不再复用 G 系真值话题名
        // /grav_truth_Odometry: G 系点云对应的 body 在 W 系下位姿
        // /grav_truth_TWG     : G 系点云对应的固定 T^W_G
        // /ref_truth_Odometry : TLS/W 系点云对应的 body 在 W 系位姿
        // /ref_truth_TWW      : ref_truth 下恒为单位阵，即固定 T^W_W
        // /ref_truth_cloud_registered: 直接发布在 TLS/W 系下的整帧点云
        ros::Publisher pubRefTruthLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
            ("/ref_truth_cloud_registered", 100000);
        ros::Publisher pubGravTruthOdom = nh.advertise<nav_msgs::Odometry>
            ("/grav_truth_Odometry", 100000);
        ros::Publisher pubGravTruthTWG = nh.advertise<geometry_msgs::PoseStamped>
            ("/grav_truth_TWG", 100000);
        ros::Publisher pubRefTruthOdom = nh.advertise<nav_msgs::Odometry>
            ("/ref_truth_Odometry", 100000);
        ros::Publisher pubRefTruthTWW = nh.advertise<geometry_msgs::PoseStamped>
            ("/ref_truth_TWW", 100000);
        ros::Publisher pubRefMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/ref_map", 1, true);

    // xinzhao:发布重力对齐结果
    ros::Publisher pubGravAlign = nh.advertise<geometry_msgs::QuaternionStamped>("/grav_alignment", 10);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    if (is_ref_mode_enabled())
    {
        // 参考地图为 latched topic，RViz 后启动也能立即看到地图
        publish_ref_map(pubRefMap);
    }
    ros::Rate rate(5000);
    bool status = ros::ok();
    //主循环
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();
        //同步LiDAR和IMU数据，形成一个measure
        if(sync_packages(Measures)) 
        {
            //第一帧扫描，记录初始时间戳
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            //IMU预处理和状态前馈
            p_imu->Process(Measures, kf, feats_undistort);  //IMU的初始化；点云去畸变
            state_point = kf.get_x();  // get_x() = get state_ikfom of esekfom::esekf<state_ikfom, 12, input_ikfom>
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;          

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            // xinzhao 保证IMU初始化后再计算重力对齐旋转矩阵q_Grav_w
            // jianping, calculate q_Grav_w
            if(!b_q_Grav_w_caclulated)
            {
                Eigen::Vector3d gravVec(state_point.grav[0],state_point.grav[1],state_point.grav[2]);  //经过IMU初始化后的重力向量
                

                G_T_I0.setIdentity();
                V3D euler_ = RotMtoEuler(Quaterniond::FromTwoVectors(gravVec.normalized(), grav_direction.normalized()).toRotationMatrix());
                euler_.z() = 0.0;
                G_T_I0.block<3,3>(0,0) = EulerToRotM(euler_);

                q_Grav_w = G_T_I0.block<3,3>(0,0);

                // xinzhao, publish grav alignment result
                Eigen::Quaterniond q_grav_w_norm = q_Grav_w.normalized();
                geometry_msgs::QuaternionStamped grav_msg;
                grav_msg.header.stamp = ros::Time::now();
                grav_msg.header.frame_id = "map";           // 重力对齐后的世界系
                grav_msg.quaternion.w = q_grav_w_norm.w();
                grav_msg.quaternion.x = q_grav_w_norm.x();
                grav_msg.quaternion.y = q_grav_w_norm.y();
                grav_msg.quaternion.z = q_grav_w_norm.z();
                pubGravAlign.publish(grav_msg);

                std::cout<<"q_Grav_w: " << q_Grav_w.coeffs()<<"\n";
                ROS_WARN("gravVec: %f, %f, %f", state_point.grav[0],state_point.grav[1],state_point.grav[2]);
                b_q_Grav_w_caclulated = true;
            }

            // ==================== BEGIN: Reference Map Mode Runtime ====================
            // 参考图模式下，静态 TLS 地图必须等到 q_Grav_w 可用后才能正确投回内部匹配链。
            // 因此这里在第一批有效 IMU 初始化完成后再延迟建树，而不是在节点启动时立即建树。
            if (is_ref_mode_enabled() && !ref_map_tree_initialized)
            {
                // q_Grav_w 只有在 IMU 初始化后可用，因此参考图建树要延后到这里执行
                initialize_reference_map_tree();
                if (!ref_map_tree_initialized)
                {
                    ROS_WARN("Reference map tree is not ready, skip this scan\n");
                    continue;
                }
            }
            // ===================== END: Reference Map Mode Runtime =====================

            //检查EKF是否初始化（需要等待一定的初始化时间）
            // 当前帧与第一帧的时间差 < INIT_TIME(通常0.1s) 时，EKF未初始化
            // IMU在静止100ms内完成重力向量和偏差初始化，之后才能开始SLAM定位建图
            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            // 根据当前LiDAR姿态和视场角，动态调整局部地图范围
            // 防止地图无限增长，节省内存和计算资源
            lasermap_fov_segment();

            /*** 对一帧扫描中的特征点进行下采样 ***/
            // 使用体素滤波器对去畸变后的点云下采样，设置叶子大小filter_size_surf_min(通常0.5m)
            // 下采样可以减少50%-70%的点数，提高ICP匹配和EKF更新的速度
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            // 记录下采样后的点云大小，用于后续有效性检查
            feats_down_size = feats_down_body->points.size();
            /*** 初始化地图kdtree ***/
            // 检查ikd-tree是否为空（Root_Node == nullptr表示未初始化）
            if(ikdtree.Root_Node == nullptr)
            {
                if (is_ref_mode_enabled())
                {
                    // 新模式下不会用首帧 scan 建图，若参考图树未就绪则当前帧直接跳过
                    ROS_WARN("Reference map mode is enabled but map tree is empty, skip this scan\n");
                    continue;
                }
                // 第一帧点云数量必须足够（>5个点）才能构建有效的kdtree
                if(feats_down_size > 5)
                {
                    // 设置地图kdtree的下采样参数（控制点云地图的密度）
                    ikdtree.set_downsample_param(filter_size_map_min);
                    // 预分配world系点云空间
                    feats_down_world->resize(feats_down_size);
                    //将第一帧下采样点从body系(LiDAR指标系)转到world系(全局坐标系)
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    //用第一帧点云构建ikd-tree作为初始参考地图
                    ikdtree.Build(feats_down_world->points);
                }
                // 第一帧只完成地图初始化，不进行EKF更新，直接跳过后续处理
                continue;
            }
            // 获取地图中的有效点数和kdtree大小（用于性能统计）
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP和迭代卡尔曼滤波器更新 ***/
            // 检查下采样后的点数是否足够（至少需要5个点才能进行有效的SLAM处理）
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                // 当前帧点数不足，跳过此帧不进行处理
                continue;
            }
            
            // 预分配法向量和world系点云的内存空间，避免动态扩展导致的性能下降
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            // 获取LiDAR与IMU之间的外参旋转关系（经欧拉角表示）
            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            // 将状态预测值记录到调试日志：时间戳、欧拉角、位置、外参、速度、IMU偏差、重力向量
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            // 调试开关：改为if(1)可将ikd-tree中的所有地图点导出用于可视化调试
            // 正常运行时保持false以减少内存消耗和计算开销
            if(0) // If you need to see map point, change to "if(1)"
            {
                // 将ikd-tree中的所有点平展到PCL_Storage中用于导出或可视化
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            // 预分配最近邻搜索的结果容器空间
            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            // 用于统计重匹配的次数和开启最近邻搜索标志
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** 迭代状态估计 ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            //执行EKF迭代更新
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            //更新最新的EKF状态 // jianping
            ekf_finish_once = true;
            latest_ekf_state.pos_end = state_point.pos;
            latest_ekf_state.rot_end = state_point.rot;
            latest_ekf_state.vel_end = state_point.vel;
            latest_ekf_state.bias_g = state_point.bg;
            latest_ekf_state.bias_a = state_point.ba;
            latest_ekf_state.gravity = state_point.grav;
            latest_ekf_time = lidar_end_time;
            state_update_flg = true;

            /******* Publish odometry *******/
            // 三种模式下位姿输出语义如下：
            //   no_ref:
            //     - /Odometry = G 系估计位姿
            //   ref_pub_grav_truth:
            //     - /Odometry           = G 系估计位姿
            //     - /grav_truth_Odometry = 同一时刻 body 在 W 系下的位姿
            //     - /grav_truth_TWG      = 与 G 系点云对应的固定 T^W_G
            //   ref_pub_ref_truth:
            //     - /ref_truth_Odometry = 直接发布在 W/TLS 系下的位姿
            //     - /ref_truth_TWW      = 单位阵，即固定 T^W_W
            if (is_ref_pub_grav_truth_mode())
            {
                // grav_truth: 保留 G 系 /Odometry，同时发布 body 在 W 系下位姿与固定 T^W_G
                publish_odometry(pubOdomAftMapped);
                publish_truth_odometry_in_ref_world(pubGravTruthOdom);
                publish_ref_relation_pose(pubGravTruthTWG, ref_T_W_G);
            }
            else if (is_ref_pub_ref_truth_mode())
            {
                // ref_truth: 直接发布 W/TLS 系 /ref_truth_Odometry，同时补充单位阵关系
                publish_truth_odometry_in_ref_world(pubRefTruthOdom);
                // RViz 兼容输出：沿用现有固定配置里的 /Odometry + map，
                // 仅作为 ref_truth 的可视化镜像，不改变 ref_truth_* 正式语义。
                publish_truth_odometry_in_frame(pubOdomAftMapped, "map", "body");
                publish_ref_relation_pose(pubRefTruthTWW, Matrix4d::Identity());
            }
            else
            {
                publish_odometry(pubOdomAftMapped);
            }

            /*** 将特征点添加到地图kdtree中 ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            // ==================== BEGIN: Reference Map Mode Publish Branch ====================
            // 三种模式下点云输出语义如下：
            //   no_ref:
            //     - /cloud_registered      = G 系点云
            //     - /cloud_registered_body = body 系点云
            //   ref_pub_grav_truth:
            //     - /cloud_registered      = G 系点云（用于相对位姿评估）
            //     - /cloud_registered_body = body 系点云（保留原始局部表达）
            //   ref_pub_ref_truth:
            //     - /ref_truth_cloud_registered = W/TLS 系点云（用于单位真值评估）
            if (ref_mode == REF_MODE_NO_REF)
            {
                if (path_en)                         publish_path(pubPath);
                if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);

                // 参考版本主循环中，这里曾直接写成：
                //   if (scan_pub_en || pcd_save_en) publish_frame_world(pubLaserCloudFull);
                //   publish_frame_world(pubLaserCloudFull_copy);
                // 再叠加当时 publish_frame_world() 内部自带保存逻辑，容易让“发布次数”影响
                // “最终 map 被累计多少次”。当前版本把发布与保存拆开后，不再保留这种耦合语义。

                PointCloudXYZI::Ptr grav_publish_cloud;
                PointCloudXYZI::Ptr grav_save_cloud;

                if (scan_pub_en)
                {
                    grav_publish_cloud = build_grav_aligned_cloud(dense_pub_en ? feats_undistort : feats_down_body);
                    publish_prebuilt_cloud(pubLaserCloudFull, grav_publish_cloud, "map");
                    publish_prebuilt_cloud(pubLaserCloudFull_copy, grav_publish_cloud, "map");
                }

                // no_ref 最终保存的 map 位于重力对齐 G 系。
                // 与旧版保持一致：最终 map / 单帧 pcd 都基于 feats_undistort 保存，
                // 不跟随 dense_publish_en 切到 feats_down_body。
                if (pcd_save_en || pcd_save_each_frame_en)
                {
                    if (scan_pub_en && dense_pub_en && grav_publish_cloud)
                    {
                        grav_save_cloud = grav_publish_cloud;
                    }
                    else
                    {
                        grav_save_cloud = build_grav_aligned_cloud(feats_undistort);
                    }

                    accumulate_cloud_to_final_map(grav_save_cloud);
                    save_single_frame_cloud(grav_save_cloud);
                }
            }
            else if (is_ref_pub_grav_truth_mode())
            {
                // grav_truth: 点云保持在重力对齐 G 系，便于后续做相对位姿评估
                // grav_truth 最终保存的 map 与 no_ref 一样，也位于 G 系。
                PointCloudXYZI::Ptr grav_publish_cloud;
                PointCloudXYZI::Ptr grav_save_cloud;

                if (scan_pub_en)
                {
                    grav_publish_cloud = build_grav_aligned_cloud(dense_pub_en ? feats_undistort : feats_down_body);
                    publish_prebuilt_cloud(pubLaserCloudFull, grav_publish_cloud, "map");
                }

                if (pcd_save_en || pcd_save_each_frame_en)
                {
                    if (scan_pub_en && dense_pub_en && grav_publish_cloud)
                    {
                        grav_save_cloud = grav_publish_cloud;
                    }
                    else
                    {
                        grav_save_cloud = build_grav_aligned_cloud(feats_undistort);
                    }

                    accumulate_cloud_to_final_map(grav_save_cloud);
                    save_single_frame_cloud(grav_save_cloud);
                }
            }
            else
            {
                // ref_truth: 点云直接发布在 TLS/W 系，后续块间真值可视作单位变换
                // ref_truth 最终保存的 map 直接位于 TLS/W 系。
                PointCloudXYZI::Ptr ref_publish_cloud;
                PointCloudXYZI::Ptr ref_save_cloud;

                if (scan_pub_en)
                {
                    ref_publish_cloud = build_ref_aligned_cloud(dense_pub_en ? feats_undistort : feats_down_body);
                    publish_prebuilt_cloud(pubRefTruthLaserCloud, ref_publish_cloud, ref_frame_id);
                    // RViz 兼容输出：将 TLS/W 系点云镜像到 /cloud_registered，
                    // 并把 W 系临时映射到 RViz 默认 fixed frame=map，便于直接观察。
                    publish_prebuilt_cloud(pubLaserCloudFull, ref_publish_cloud, "map");
                }

                if (pcd_save_en || pcd_save_each_frame_en)
                {
                    if (scan_pub_en && dense_pub_en && ref_publish_cloud)
                    {
                        ref_save_cloud = ref_publish_cloud;
                    }
                    else
                    {
                        ref_save_cloud = build_ref_aligned_cloud(feats_undistort);
                    }

                    accumulate_cloud_to_final_map(ref_save_cloud);
                    save_single_frame_cloud(ref_save_cloud);
                }
            }
            // ===================== END: Reference Map Mode Publish Branch =====================
            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                //计算平均时间消耗
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                //记录性能数据用于后续分析
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string all_points_dir = map_dir_path.c_str();
        //string file_name = string("scans.pcd");
        //string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to " << all_points_dir <<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    //保存运行时性能日志
    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    if (file_trajectory != nullptr)
    {
        fclose(file_trajectory);
    }

    return 0;
}
