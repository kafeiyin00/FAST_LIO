# FAST_LIO AI 辅助资料

本目录统一分为三类：[`IMPORTANT_CHANGES.md`](IMPORTANT_CHANGES.md)、[`ALGORITHM_TESTS.md`](ALGORITHM_TESTS.md) 和 [`testing/`](testing/README.md)。现有 `validation/`、`data_tools/` 是第三类检测资产；本 README 负责导航，不再混合增长全部历史正文。

本目录不替代正式源码、launch 或 config。

## 固定位置环境、编译与运行

```bash
cmake -S /home/workspace/simulation/download/Livox-SDK \
  -B /home/workspace/simulation/environment/build/livox-sdk-2.3.0-v2 \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build /home/workspace/simulation/environment/build/livox-sdk-2.3.0-v2 --parallel 2
cmake --install /home/workspace/simulation/environment/build/livox-sdk-2.3.0-v2

mkdir -p /opt/ws_livox_ros_driver/src
ln -s /home/workspace/simulation/download/livox_ros_driver/src/livox_ros_driver \
  /opt/ws_livox_ros_driver/src/livox_ros_driver
source /opt/ros/noetic/setup.bash
cd /opt/ws_livox_ros_driver && catkin_make -j2 -DCMAKE_BUILD_TYPE=Release

source /home/workspace/simulation/environment/shell_entries.sh
enter-simulation
cd "$SIMULATION_ROOT"
catkin_make --force-cmake --pkg fast_lio -j2 \
  -DCMAKE_BUILD_TYPE=Release -DCeres_DIR="$Ceres_DIR"
```

```bash
enter-simulation
roslaunch fast_lio run_marsim_simulation.launch rviz:=false
roslaunch fast_lio run_helmet_mid.launch autorun:=true rviz:=false
```

检测资产职责：`validation/run_whu_validation.launch` 与 `data_tools/aggregate_whu_validation.py` 归 [`testing/`](testing/README.md) 导航；正式运行仍使用仓库 launch。
