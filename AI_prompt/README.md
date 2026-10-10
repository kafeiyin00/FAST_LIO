# FAST_LIO AI 辅助资料

当前同步回溯点为`baseline-sim-whu-real-maan-20261009`。读取
[IMPORTANT_CHANGES.md](IMPORTANT_CHANGES.md)顶部的baseline发布记录，确认完整commit、依赖版本、
验证范围和有效配置；五仓库对应关系见
[统一baseline索引](/home/workspace/Forest_Interface/AI_prompt/indexes/BASELINE_TAG_INDEX.md)。

本目录统一分为三类：[`IMPORTANT_CHANGES.md`](IMPORTANT_CHANGES.md)、[`ALGORITHM_TESTS.md`](ALGORITHM_TESTS.md) 和 [`testing/`](testing/README.md)。现有 `validation/`、`data_tools/` 是第三类检测资产；本 README 负责导航，不再混合增长全部历史正文。

本目录不替代正式源码、launch 或 config。

## 历史 WHU/Maan submap 版本入口

使用 `/home/workspace/simulation/data/bag_single/` 和 Forest_CSLAM `data/frameReg/` 中的历史
submap 前，必须先读：

- [IMPORTANT_CHANGES.md](IMPORTANT_CHANGES.md) 的“历史离线 WHU/Maan 版本与当前版本差异”；
- [ALGORITHM_TESTS.md](ALGORITHM_TESTS.md) 的 `HO-20260921-002` 动态复现结果。

关键边界：WHU 历史处理链不是一个可证明的干净 commit，而是 `fe7222` estimator/core 与后期
generate_block/global-shift 语义组成的 composite；它使用旧 `MAX_INI_COUNT=10`。Maan 历史
canonical `grav_truth` submap 对应 `145c731` 源码内容、逐 Line `T_W_G` 和 1x 回放。不得用当前
FAST-LIO 默认参数重跑 WHU 后把结果描述为历史等价，也不得用 4x Maan 回放建立历史基线。

## 独立环境导航

固定依赖和运行边界见
[`INDEPENDENT_ENVIRONMENT.md`](../../INDEPENDENT_ENVIRONMENT.md)，FAST_LIO 的标准
增量编译入口见其中的
[FAST_LIO 小节](../../INDEPENDENT_ENVIRONMENT.md#fast-lio-build)。不要创建
`catkin_make` 不会读取的 defaults YAML。

检测资产职责：`validation/run_whu_validation.launch` 与 `data_tools/aggregate_whu_validation.py` 归 [`testing/`](testing/README.md) 导航；正式运行仍使用仓库 launch。
