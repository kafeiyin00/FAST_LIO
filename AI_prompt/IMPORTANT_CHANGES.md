# FAST_LIO 重要更改记录

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
