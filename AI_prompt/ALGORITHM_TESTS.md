# FAST_LIO AI 算法测试记录

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
