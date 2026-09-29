# FAST_LIO AI 算法测试记录

## HO-20260921-002：历史 WHU/Maan submap 两级动态复现

测试先验证历史候选能否复现 retained history，再比较 current Forest；没有以当前代码重跑成功
冒充历史基线，也没有为了追近结果调参。

### WHU 历史 composite 复现

- 输入：`bag_single/WHU_TLS_Forest` 五条原始 ROS1 bag。
- 代码：`fe7222` estimator/core 加 `145c731` generate_block/global-shift 语义的 task-local
  composite；Release 构建，不修改活动仓库。
- block：历史/重跑均为 `8/10/7/7/7=39`；全部文件名、anchor stamp、点数和文件大小一致，
  `18/39` 个 PCD 字节相同。
- `7,034,222` 个有序点：mean/RMSE/p99/max=`1.30/2.54/9.44/51.5 mm`。
- 3282 个连续 odom stamp 全等；位置 p99/max=`3.49/7.55 mm`，姿态
  p99/max=`0.0248/0.0437 deg`。
- 结论：`pass_numerical_composite_revision_unproven`。该 composite 可作为数值历史基线，
  不能作为精确 binary/revision 声明。

### Maan `145c731` 复现

- 输入：13 条有效原始 ROS1 bag，Plot2/4 Line3 使用 recollect；1x 回放。
- 参数：逐 Plot TLS map、逐 Line `T_W_G`、`ref_pub_grav_truth` real profile。
- block：历史/重跑均为 Plot1/2/3/4=`146/125/125/94`，合计 490。
- `400/490` 个 PCD 字节相同；`176,991,604` 个点的
  mean/RMSE/p99/max=`0.075/0.234/0.988/33.1 mm`。
- Plot3/Line2、Plot4/Line3 的首次 1x run 跨过初始化边界，rerun2 精确恢复 retained anchor；
  4x Plot1/Line1 未复现历史边界。
- 结论：`pass_source_content_confirmed_at_145c731`，但 at-run binary/TLS-map hash仍未证明。

### Current Forest 对照摘要

- WHU：同为 39 blocks，但 anchor 平均晚 `1.044 s`；route3--5 同 stamp odom 的位置 p99
  `7.967/8.038/6.902 m`、姿态 p99 `9.290/9.468/8.664 deg`。这证明旧/新初始化和后续状态
  演化造成真实差异，不能写成当前版本复现历史 WHU。
- Maan：同为 490 blocks；39,694 个 matched world cloud 点数全等；odom p99
  `2.822 mm / 0.01949 deg`；双向 NN 的最差 block p99 `4.59 cm`。结论为数值等价，保留
  ROS/epoch、0--2 scan 初始化边界和序列化差异。
- 权威报告：`/home/workspace/Forest_CSLAM/AI_test/HO-20260921-002/RESULT.md`；逐 block/帧
  指标在同目录 `attempt_001/results/`。

## WHU-TLS 五路线验证

- 五路线得到 35 个完整 submap，连续 index `0..34`。
- 595 个无序 pair 和 1190 个方向矩阵完成复算。
- Route 1-5 位置 RMSE：`0.0519/0.0909/0.0643/0.0493/0.0315 m`。
- 新旧结果坐标语义、80 帧分块和过滤配置一致；新 bag 帧数/每帧点数不同，因此不要求 submap 数、点数和文件逐字节相同。
- 完整报告：`/home/workspace/simulation/data/pipeline_validation/WHU_TLS_Forest_20260806_145507/comparison/VALIDATION_REPORT.md`。

## 上游影响分析

- renderer 的 near clip、rotor、外参、下采样和启动门会改变输入帧数、点数或起始时间。
- Marsim ground-truth angular velocity 修复不改变 FAST_LIO 读取的独立 IMU 机体系角速度。
- 比较时必须保存 Marsim revision/dirty diff、launch、IMU/LiDAR 频率、下采样、near clip、地图/航点、启动门、FAST_LIO 初始化、重力方向和尾组状态。

后续固定 bag 测试继续记录 profile、revision/config、命令、submap/轨迹指标、结果和分析。
