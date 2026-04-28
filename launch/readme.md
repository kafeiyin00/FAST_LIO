# 1.离线帧生成及帧转换关系的真值计算

真实数据和仿真数据都统一使用 run_helmet_mid.launch，差别只在于 launch 中加载的 YAML 配置不同：
1. 仿真数据加载 FAST_LIO/config/marsim.yaml
2. 真实数据加载 FAST_LIO/config/mid360.yaml

下面给出两套完整中文命令流程：
1. 仿真数据：FAST-LIO 建图（no_ref，marsim.yaml） -> generate_block -> 计算任意两个 block 的 odom 真值
2. 真实数据：FAST-LIO 建图（grav_truth 或 ref_truth，mid360.yaml） -> generate_block -> 计算任意两个 block 的 odom 真值

## 1.1前置准备

在工作空间根目录执行：

```bash
cd /home/workspace/code/simulation
catkin_make --pkg fast_lio -j2
source devel/setup.bash
```

说明：
1. odom_transformer 的输入是一组 .odom 文件，每个文件里存一个 4x4 位姿矩阵。
2. 只要这些 .odom 统一表达为同一公共世界系下的 T^W_A，就可以直接计算任意两个 odom 之间的相对真值。
3. generate_block 输出的 .odom 默认保存在 dataFolder 目录下。

run_helmet_mid.launch 配置切换模板

仿真数据版本：

```xml
<rosparam command="load" file="$(find fast_lio)/config/marsim.yaml" />
<!-- <rosparam command="load" file="$(find fast_lio)/config/mid360.yaml" /> -->
```

真实数据版本：

```xml
<rosparam command="load" file="$(find fast_lio)/config/mid360.yaml" />
<!-- <rosparam command="load" file="$(find fast_lio)/config/marsim.yaml" /> -->
```

建议：
1. 平时只保留一个启用的 rosparam load，避免误读两套 YAML。
2. 若希望与当前 launch 文件风格一致，generate_block 的输出目录建议显式带上模式后缀，例如：
	- 仿真 no_ref: 某个数据前缀 + _no_ref_frames / _no_ref_points
	- 真实 grav_truth: bag_file + _grav_truth_frames / _grav_truth_points
	- 真实 ref_truth: bag_file + _ref_truth_frames / _ref_truth_points

## 1.2仿真数据完整流程

适用条件：
1. 在 run_helmet_mid.launch 中加载 FAST_LIO/config/marsim.yaml。
2. ref_map.mode 设为 no_ref。

建议配置：
1. 在 FAST_LIO/config/marsim.yaml 中确认 ref_map.mode: "no_ref"。
2. 在 FAST_LIO/config/generate_block.yaml 中：
	 - 若保持旧 no_ref 方式，设 save_registered_data: false。
	 - 若希望直接保存已注册整帧点云，设 save_registered_data: true。
	 - 若希望将 block 的 odom 额外写到仿真全局系，可设 apply_global_shift: true，并正确填写 global_shift。


### 终端 1：启动 FAST-LIO 仿真建图 + 启动 generate_block （注意launch中的enable_generate_block:=true）
roslaunch fast_lio run_helmet_mid.launch


说明：
1. 当前 no_ref 模式下，generate_block 会自动读取参数服务器中的 ref_map.mode=no_ref。
2. 输出的 block 位姿文件：.odom 文件，下游 odom_transformer 直接读取这里的 .odom 文件。
3. 如果 apply_global_shift: true，generate_block 还会额外输出 global_*.odom；若你要严格使用仿真公共全局系真值，推荐把这些 global_*.odom 单独整理到一个文件夹再做变换计算。

### 终端 2：计算任意两个 block 的 odom 真值

roslaunch fast_lio odom_transformer.launch

## 1.3真实数据完整流程

适用条件：
1. 在 run_helmet_mid.launch 中加载 FAST_LIO/config/mid360.yaml。
2. ref_map.mode 设为 ref_pub_grav_truth 或 ref_pub_ref_truth。
3. 使用 rosbag 播放真实数据。

建议配置：
1. 在 FAST_LIO/config/mid360.yaml 中选择模式：
	 - ref_map.mode: "ref_pub_grav_truth"
	 - 或 ref_map.mode: "ref_pub_ref_truth"
2. 同时确认：
	 - ref_map.pcd_path 指向 TLS 参考地图
	 - ref_map.transform_wg 填写对应线路的 T^W_G
3. 在 FAST_LIO/config/generate_block.yaml 中保持 save_registered_data: true。
	 - grav_truth 模式下，generate_block 会自动订阅 /cloud_registered 和 /grav_truth_Odometry。
	 - ref_truth 模式下，generate_block 会自动订阅 /ref_truth_cloud_registered 和 /ref_truth_Odometry。

### 终端 1：启动 FAST-LIO 真实数据建图 + 启动 generate_block

先将 run_helmet_mid.launch 切换到真实数据版本，即只加载 mid360.yaml。
roslaunch fast_lio run_helmet_mid.launch


说明：
1. 若使用 grav_truth，则最终 block odom 语义是统一 W 系下的 T^W_G。
2. 若使用 ref_truth，则最终 block odom 语义是统一 W 系下的 T^W_W 或更一般的 T^W_A；对 odom_transformer 来说只要全都在同一公共 W 系即可直接计算。


### 终端 2：计算任意两个 block 的 odom 真值

roslaunch fast_lio odom_transformer.launch


补充说明

1. odom_transformer 当前直接读取 dataFolder 下的 .odom 文件，不读取 pointsFolder。
2. 若只想计算若干指定配对，可先编辑 FAST_LIO/config/odom_transformer.yaml 中的 indices，再执行 roslaunch fast_lio odom_transformer.launch。


# 2.文件批量重命名
如果需要把文件按 0 开始重命名，可在对应目录执行：

重命名当前文件夹下的 pcd 文件（从 0 开始）：

```bash
ls -v *.pcd | awk -v start=0 '{printf "mv -n \"%s\" \"%d.pcd\"\n", $0, NR+start-1}' | bash
```

重命名当前文件夹下的 odom 文件（从 0 开始）：

```bash
ls -v *.odom | awk -v start=0 '{printf "mv -n \"%s\" \"%d.odom\"\n", $0, NR+start-1}' | bash
```