# FAST_LIO 检测运行资产

现有 `../validation/run_whu_validation.launch` 和 `../data_tools/aggregate_whu_validation.py` 属于本类。正式 FAST_LIO launch/config 不移动。新增入口必须显式传入输入和唯一输出，并记录超时、ROS master 与清理方式。
