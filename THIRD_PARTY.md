# 来源与许可证

本工程保留上游版权声明。根目录 LICENSE 为 FAST-LIO 的 GPL-2.0；子目录和独立依赖按各自许可证分发。

| 组件 | 来源 / 固定版本 | 说明 |
|---|---|---|
| 哨哨 Web、串口、FAST-LIO 和部署基础 | chenkl112/shaoshao `debd22c1089ef2421fdb5667d0f08007d4aa9e7c` | 源自 PolarisXQ/SCURM_Nav_Tutorial |
| FAST-LIO | FAST_LIO/LICENSE | GPL-2.0；修改速度和协方差发布，沿用静态导出与过滤 |
| Livox ROS Driver2 | livox_ros_driver2/LICENSE.txt | 保留原声明；改为系统 SDK 链接 |
| small_gicp | koide3/small_gicp `57c1106daf83c2c79ee0c58a9c7ed0032298ff4e`，v1.0.1 | MIT，见 LICENSES/small_gicp.txt；构建时拉取 |
| Livox-SDK2 | Livox-SDK/Livox-SDK2 `c0796f04c143143899c87a773d9f6b7136453c0b` | 构建时拉取，按其 LICENSE 分发 |
| HW 配准质量评估参考 | Polyacetone/HWSentryNav26 | MIT，见 LICENSES/HWSentryNav26.txt；重写注册服务和调度，不是 HW 官方版本 |
| HW FDDP 求解器 | Polyacetone/HWSentryNav26 `f5f941288197e14c867d711a4c4cd85bfd7a3194` | 原样复制 `fddp_solver.hpp`，MIT 许可证见 `hw_omni_controller/vendor/LICENSE` |
| HW 原始规划、跟踪与恢复 | Polyacetone/HWSentryNav26 `f5f941288197e14c867d711a4c4cd85bfd7a3194` | 复制 `common`、`path_planner` 及恢复/监控状态机，平地全向适配见 `hw_omni_controller/vendor/hw_nav/README.md`，MIT 许可证同目录 |
| HW 障碍预测思路 | 同上 | 本地重写检测/关联/卡尔曼/形状外推，未复制轮腿地图服务器 |
| Drogon、Trantor、JsonCpp | Ubuntu 24.04 系统开发包 | 使用对应上游许可证，动态链接，不复制依赖源码 |
| Nav2、MPPI、ROS 2、PCL | Ubuntu / ROS Jazzy 系统包 | 使用对应上游许可证，不复制系统二进制 |
| Navcar UART 协议 | Li-chee/Navcar-Comm-Protocol `4eb14adf5d7bc463dca50c092a1b3be66f77ef9a` | 沿用原适配，不改变 MCU 帧格式 |

新增 `shaoshao_hw`、`hw_registration`、`shaoshao_hw_interfaces`、`shaoshao_hw_runtime` 代码使用 MIT，见 LICENSES/shaoshao-hw.txt。历史文档仅作参考，以根 README 和 deployment/README 为本版部署依据。
