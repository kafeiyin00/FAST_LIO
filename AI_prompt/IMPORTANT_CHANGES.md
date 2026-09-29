# FAST_LIO 重要更改记录

## 2026-09-29：历史离线 WHU/Maan 版本与当前版本差异（HO-20260921-002）

### 历史 WHU 不是单一干净 revision

- 历史 WHU 五条 bag 生成于 2025-12，frames/odom/39 个 blocks 于 2026-03-04 生成。输出的
  初始化边界与旧 `MAX_INI_COUNT=10` 一致，但同时包含当时尚未提交、后来进入 `145c731...`
  的 `generate_block` global-odom/global-shift 语义。
- 动态确认的最可信处理链是 `fe7222aa88d0d4671a7db4f9b7fb3b0e320d1210`
  estimator/core，加 evidence-matched 后期 generate_block；它是 composite，不能标成精确
  `fe7222` binary。原 dirty diff、编译产物 hash 和完整运行 manifest 没有留存。
- 历史有效 sim 参数：Marsim PointCloud2、`lidar_type=4`、`point_filter_num=3`、`blind=0.3`、
  `scan_line=6`、surf/map voxel `0.2`、`max_iteration=10`、`det_range=450`、`fov=90`、
  bias covariance `0.01`、单位外参、`+Z`、80 帧、不补尾组。submap 使用 body cloud 并变换到
  第一帧 body anchor；`global_shift` 只写 global odom，PCD 仍是 local anchor 坐标。
- 当前版本使用 `1.0 s / 200 samples`。它相对历史首 anchor 约晚 1 秒，并可使 route3--5
  形成约 `6.9--8.0 m / 8.7--9.5 deg` 的持续 odom 差异。处理历史 WHU 时必须显式选择：
  复现旧基线，还是使用当前算法重新处理；两类结果不能混称为同一前端产物。

### 历史 Maan 对应 `145c731` 源码内容

- Maan canonical `grav_truth` 历史 submap 使用
  `145c731fc3e86271c8856c5209b3cc9e5ccb80a1` 的 reference-map/IMU 初始化代码语义。
  13 条有效 Line 在 1x 原始 ROS1 bag 下恢复全部 `490` blocks；Plot2/4 Line3 必须使用
  recollect，`Line3_wrong` 和无索引 `.orig.bag` 不得作为正式输入。
- 历史 real 参数：Livox CustomMsg、`lidar_type=1`、`point_filter_num=3`、`blind=0.2`、
  `scan_line=4`、surf/map voxel `0.2`、`max_iteration=10`、`det_range=100`、`fov=360`、
  bias covariance `0.0001`、外参 `[-0.011,-0.02329,0.04412]`、`-Z`、
  `1.0 s / 200 samples`、TLS map leaf `0.1`、scan leaf `0.2`、逐 Line `T_W_G`。
- `ref_pub_grav_truth` 的 PCD 是 80 帧 G/world cloud 直接拼接，不是 WHU 式首帧局部坐标；
  block `.odom` 是固定 `T_W_G`，不是动态 FAST-LIO anchor pose。
- 4x 回放会放大 reference-map 建树的墙钟耗时并改变消息边界，不能替代历史 1x 基线。
  初始化门附近个别 Line 会在重复运行中跨 1--2 个 scan，必须记录 anchor 清单而非只比 block 数。

### 相对当前 `799969e` 的含义

- `145c731` 之后与 Maan 主链相关的变化主要是参数名/默认值整理、初始化窗口参数化及文档；
  未发现会改变 Maan ESKF、去畸变、reference-map 匹配或 ikd-tree 数学的实质变更。动态结果也
  支持 Maan 数值等价。
- WHU 则跨越了真实的初始化逻辑变化，不能沿用上述 Maan 等价结论。恢复或评估历史 WHU 时，
  revision、初始化门、重力方向、global shift、bag identity 和 80 帧边界必须作为一个整体冻结。
- 完整证据：`/home/workspace/Forest_CSLAM/AI_test/HO-20260921-002/RESULT.md`。

## 已提交历史：IMU 初始化与重力对齐

- `145c731fc3e86271c8856c5209b3cc9e5ccb80a1` 把旧 `MAX_INI_COUNT=10` 改为实际 `1.0 s / 200 samples` 门；源码旧注释中的 `3.0 s / 600` 不是真实宏值。
- 初始化期间每个 LiDAR 测量组直接返回；门达到后的下一组才产生有效点云。对 Marsim 200 Hz IMU/10 Hz LiDAR，第一帧有效输出约后移 1.0-1.1 秒。
- `generate_block` 每 80 帧保存完整 submap且不补尾组，因此初始化差异可改变 submap 数量和起点。
- 重力对齐从第一批 IMU 中间状态锁定，改为初始化完成后第一帧有效点云时锁定。历史实现重力方向 `[0,0,-1]`，当前已提交配置为 `[0,0,1]`，恢复实验必须同时核对。

## 2026-08-21 未提交配置

- 基线：`dev-zhaoxin@2cc0860e7f78e9cb9e1b1543be31fbd844fe0213`。
- `generate_block.yaml`：关闭 registered data 保存、开启 global shift，并更新固定平移；80 帧分块不变。
- `marsim.yaml`：PCD、地图和 TLS 路径迁入 simulation 根。
- `run_helmet_mid.launch`：默认加载 `marsim.yaml`，更新默认 bag，`autorun=true` 时延后 1 秒执行 rosbag。
- 上游 forest launch 同期改为独立文件，使用 0.1 m 下采样/near clip、WHU 地图和 waypoint，并保留 `/points_loaded` 就绪门。

后续追加算法、配置、topic、坐标系、初始化和复现方式变化。
