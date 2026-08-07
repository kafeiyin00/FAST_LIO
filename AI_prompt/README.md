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
