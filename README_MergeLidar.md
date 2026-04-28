# MergeLidar 多雷达实时融合系统

## 概述
MergeLidar 是一个支持多种雷达类型的实时点云融合系统，可以自动检测并处理 PointCloud2 格式（如 Ouster、Velodyne）和 Livox CustomMsg 格式的点云数据。

## 支持的雷达类型
- **PointCloud2**: 标准点云格式（Ouster、Velodyne等）
- **Livox CustomMsg**: Livox雷达专用格式

## 新特性：自动消息类型检测
- ✅ **无需手动配置雷达类型**
- ✅ **自动检测PointCloud2和Livox CustomMsg**
- ✅ **使用PCL标准点类型**
- ✅ **保留intensity和时间信息**

## 配置参数

### 基本参数
- `lidar_topic`: 雷达话题列表
- `lidar_extr`: 雷达外参矩阵（4x4变换矩阵）

### 配置示例

#### 混合雷达配置（自动检测类型）
```xml
<rosparam param="lidar_topic">[
    "/os_cloud_node/points",    # 系统自动检测：Ouster雷达
    "/livox/lidar",             # 系统自动检测：Livox雷达
    "/velodyne_points"          # 系统自动检测：Velodyne雷达
]</rosparam>
```

#### 单个Livox雷达配置
```xml
<rosparam param="lidar_topic">["/livox/lidar"]</rosparam>
```

## 启动方式

### 1. 混合多雷达融合
```bash
roslaunch fast_lio merge_lidar.launch
```

### 2. 单个Livox雷达
```bash
roslaunch fast_lio merge_lidar_livox.launch
```

### 3. 带可视化启动
```bash
roslaunch fast_lio merge_lidar.launch rviz:=true
```

## 输出话题
- `/merged_pointcloud`: 融合后的点云数据（sensor_msgs/PointCloud2）

## 性能特性
- **实时同步**: 20Hz 目标同步频率
- **多线程处理**: 支持并行点云变换
- **缓冲区管理**: 自动防止内存溢出
- **性能监控**: 实时显示处理统计信息

## 监控工具
使用测试脚本监控融合性能：
```bash
rosrun fast_lio test_merge_lidar.py
```

## 点云格式转换

### 自动类型检测和转换
系统会自动检测消息类型并进行相应转换：

#### Livox CustomMsg 到 PCL PointOuster 映射
| Livox CustomMsg | PCL PointOuster   | 说明                    |
|----------------|-------------------|------------------------|
| x, y, z        | x, y, z          | 3D坐标                 |
| reflectivity   | intensity        | 反射强度               |
| reflectivity   | reflectivity     | 反射率（保持原值）        |
| offset_time    | t                | 时间偏移（纳秒）         |
| line           | ring             | 激光线号               |
| tag            | -                | 标签信息（未使用）        |

#### PointCloud2 直接处理
- 直接使用PCL标准转换
- 保持原有字段映射
- 无需额外转换步骤

## 外参标定
雷达外参矩阵格式为4x4齐次变换矩阵：
```
[R11, R12, R13, tx]
[R21, R22, R23, ty]
[R31, R32, R33, tz]
[0,   0,   0,   1 ]
```

其中：
- R: 3x3旋转矩阵
- t: 3x1平移向量

## 编译要求
- ROS Melodic/Noetic
- PCL 1.8+
- Eigen3
- livox_ros_driver
- OpenMP（可选，用于加速）

## 故障排除

### 常见问题
1. **编译错误**: 确保安装了 livox_ros_driver
2. **点云为空**: 检查话题名称是否正确
3. **性能低下**: 调整缓冲区大小和同步频率
4. **外参错误**: 验证变换矩阵的正确性
5. **消息类型问题**: 系统会自动检测，无需手动配置

### 调试命令
```bash
# 查看话题列表
rostopic list | grep lidar

# 检查点云数据
rostopic echo /livox/lidar -n 1

# 监控融合结果
rostopic hz /merged_pointcloud
```

## 性能优化建议
1. 根据实际需求调整同步频率
2. 优化雷达外参标定精度
3. 合理设置缓冲区大小
4. 使用多线程处理提高效率
