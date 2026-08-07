# FAST_LIO AI 辅助资料

本目录保存 WHU-TLS 专用验证入口和数据整理工具，不替代 FAST_LIO 的正式源码、通用 launch 或 config。

## 文件分类

- `validation/run_whu_validation.launch`：隔离的 WHU 仿真 bag 验证入口，固定历史可比参数，所有输出路径及航线平移由调用者显式传入。
- `data_tools/aggregate_whu_validation.py`：按 Route 1-5 和时间戳顺序聚合 submap/odom、生成连续 index，并为 `all_transforms.txt` 添加航线范围注释。
- 正式算法与兼容性修复仍在 `src/laserMapping.cpp`、`config/*.yaml` 及原有 launch 中。

验证 launch 使用绝对路径启动：

```bash
source /home/workspace/code/simulation/devel/setup.bash
roslaunch /home/workspace/code/simulation/src/rl_motor_lidar/FAST_LIO/AI_prompt/validation/run_whu_validation.launch \
  data_folder:=/home/workspace/work/whu/route_1/frames \
  points_folder:=/home/workspace/work/whu/route_1/points \
  shift_x:=21.195669 shift_y:=7.421676 shift_z:=2.797266 \
  rviz:=false
```

详细批处理顺序见 `launch/readme.md`。

## WHU-TLS 验证结论

五航线最终得到 35 个完整 submap，连续 index 为 `0..34`；全部 595 个无序 pair 和 1190 个方向矩阵都已复算检查。各路线位置 RMSE 为 `0.0519/0.0909/0.0643/0.0493/0.0315 m`，均优于对应现有结果。

新旧结果的坐标语义、80 帧分块方式和过滤配置一致，算法精度正常；但新 bag 的帧数和每帧点数不同，因此 submap 数量、点数和文件内容不会逐字节相同。完整量化报告位于：

```text
/home/workspace/data/pipeline_validation/WHU_TLS_Forest_20260806_145507/comparison/VALIDATION_REPORT.md
```

## 2026-08-07 上游 Marsim 变更对 FAST_LIO 的影响

FAST_LIO 源码和本目录验证工具在本次 Marsim 参数化中没有算法修改，但后续比较新仿真 bag 时必须记录所用 Marsim profile：

- RL 的普通和旋转 GPU renderer 已统一为同一参数化实现；`near_clip`、`rotor_enabled`、外参、下采样和启动策略都会改变点云帧数、点数或起始时间。
- forest 推荐配置使用 `rotor_enabled=false`、`near_clip=5 m`、地图/点云下采样 `0.01 m`，并等待 `/points_loaded=true` 后启动；`wait_time=0 s` 表示就绪后不再额外等待。它不是历史 forest 旋转节点 3 m 配置的逐项复刻。
- 历史 30 秒等待只是手动运行时为大场景加载预留的时间。历史 WHU Route 2 bag 的点云约在录包后 4.04 秒出现，因此 WHU 验证不应固定等待 30 秒。
- Marsim ground-truth odometry 的世界系角速度计算顺序已修正；FAST_LIO 的输入仍是 `/quad_0/imu` 的机体系角速度，所以该修复不改变 FAST_LIO 的 IMU 输入语义。

比较 FAST_LIO 结果时，应同时保存 Marsim launch 名称和关键传感器参数。只有 profile 一致时，点数、submap 数量和轨迹误差才具有直接可比性；即使参数一致，也不要求 bag 或 PCD 文件逐字节相同。
