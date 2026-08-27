# FAST_LIO AI 辅助资料

本目录统一分为三类：[`IMPORTANT_CHANGES.md`](IMPORTANT_CHANGES.md)、[`ALGORITHM_TESTS.md`](ALGORITHM_TESTS.md) 和 [`testing/`](testing/README.md)。现有 `validation/`、`data_tools/` 是第三类检测资产；本 README 负责导航，不再混合增长全部历史正文。

本目录不替代正式源码、launch 或 config。

## 独立环境导航

固定依赖和运行边界见
[`INDEPENDENT_ENVIRONMENT.md`](../../INDEPENDENT_ENVIRONMENT.md)，FAST_LIO 的标准
增量编译入口见其中的
[FAST_LIO 小节](../../INDEPENDENT_ENVIRONMENT.md#fast-lio-build)。不要创建
`catkin_make` 不会读取的 defaults YAML。

检测资产职责：`validation/run_whu_validation.launch` 与 `data_tools/aggregate_whu_validation.py` 归 [`testing/`](testing/README.md) 导航；正式运行仍使用仓库 launch。
