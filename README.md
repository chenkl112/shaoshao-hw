# 哨哨 HW

面向四轮全向底盘和顶置 MID360 的独立导航工程。保留 FAST-LIO + small_gicp、网页选点和 Nav2 任务接口；全局规划使用 HW 空间 A*、走廊 Kino A*、MINCO 和速度剖面，局部控制使用 HW FDDP 求解器与整车 XY 速度响应模型；动态障碍检测/跟踪/预测、底盘授权门控和 50 Hz 速度转发使用 C++。UART1 已同步 [Navcar V2](deployment/NavcarV2接入.md)：SI 目标与底盘系轮速反馈。实现边界与电控接口见 [HW 全向导航方案](deployment/HW全向导航方案.md)。

从 [shaoshao](https://github.com/chenkl112/shaoshao) 的 `debd22c1089ef2421fdb5667d0f08007d4aa9e7c` 独立建立，参考 [HWSentryNav26](https://github.com/Polyacetone/HWSentryNav26) 的配准质量评估思路。原仓库保持独立。上车步骤见 [部署说明](deployment/README.md)，串口格式见 [Navcar V2 接入](deployment/NavcarV2接入.md)。

## 本版行为

- FAST-LIO 连续输出局部位姿、内部估计速度和当前帧协方差；不对位置做差分来生成速度。
- 建图同步记录关键帧；保存 PCD 后剔除不在静态地图中的点，生成带地图哈希的检索索引。
- 导航启动后自动检索位置、进行 small_gicp 配准，再用三个独立新扫描确认。匹配不唯一时保持停车。
- 只有通过质量与跳变检查的校正才平滑写入 `map → navcar_odom`。拒绝一次校正不会抹掉已有定位，也不会刷新有效校正时间。
- 短暂退化只在近场余量充足时以至多 **0.05 m/s** 衔接，最长有效校正年龄 **2 s**、累计距离 **5 cm**。窄处、扫描/里程计超时直接停车。
- 定位丢失后保留原目标，停车、全局定位、重新规划，再继续。取消、手动接管、切换模式和地图变化会阻止自动续行。
- HW FDDP 20 Hz 预测未来 2 s 的全向运动，目标合速度上限 **0.5 m/s**、加速度 **0.3 m/s²**、jerk **1 m/s³**；50 Hz 转发目标。平移与小陀螺分通道，四轮分配由电控完成。
- 规划失败保留目标，0.5 s 冷却后重试；跟随 6 s 无进展或检测物理卡住时进入反向退让/安全点恢复，结束后重新规划原目标。见 [规划与脱困适配](validation/hw-planning-recovery.md)。
- 点云与静态地图差分，聚类、匈牙利关联和卡尔曼跟踪，发布带形状的 3 s 障碍预测；MPC 按采样时间对齐并验收未来碰撞。
- 预测、规划初速度、刹停和到位判断统一使用 FAST-LIO 雷达/IMU 融合速度，保留采样时间并补偿到车体中心，不回退到轮速。拨杆授权独立使用 `/hw/control_authority`；V2 原始轮速只供诊断。见 [FAST-LIO 速度预测](deployment/FASTLIO速度预测.md)。

## 网页

原生 HTML/CSS/JavaScript 前端 + C++ Drogon HTTP/WebSocket 服务。状态推送与 float32 二进制点云由 C++ 提供，任务监管服务使用私有 Unix socket。现有页面和操作入口保留，见 [网页架构](deployment/网页架构.md)。

## 实车配置

主要参数集中在 [web_panel/robot.yaml](web_panel/robot.yaml)。

| 项目 | 当前值 |
|---|---|
| 正八边形对边距，含轮子和突出物 | 550 mm |
| 外接圆半径 / 单侧碰撞余量 | 297.658 mm / 20 mm |
| 雷达底面 / yaw 安装面高度 | 280 mm |
| 雷达光学原点高度 | 327 mm，底面 + 47 mm |
| yaw 轴至光心 XY，雷达坐标 | `[-0.01418, 0] m` |
| 导航参考点 | 车体中心的 yaw 轴；方向随雷达 |
| 全局 / 局部栅格 | 0.025 m |
| 导航合速度上限 | 0.5 m/s，按路况变化 |
| UART1 V2 | 2,000,000 baud；m/s、rad/s 直接下发，旋转逆时针为正 |

本版继续使用覆盖任意底盘朝向的外接圆检查碰撞。含余量的直径约 **635.3 mm**。650 mm 通道只有约 **7.3 mm/侧** 的剩余几何空间，2.5 cm 栅格离散、地图和控制误差仍可能使其不可通行，不能保证通过。

栅格分辨率、GICP 的 5 cm 体素和 FAST-LIO 的 15 cm 实时匹配滤波分别服务于不同步骤；细栅格不会恢复未观测的点。

## 启动

需要 Ubuntu 24.04 / ROS 2 Jazzy。先按 [部署说明](deployment/README.md) 安装依赖、确认 IP 和串口，再执行：

```bash
git clone https://github.com/chenkl112/shaoshao-hw.git ~/ccc/shaoshao-hw
cd ~/ccc/shaoshao-hw
bash deployment/bootstrap.sh
# 终端一：串口控制和手动网页
bash ~/ccc/shaoshao-hw-runtime/start_move_web.sh
# 终端二：雷达预览和主网页
bash ~/ccc/shaoshao-hw-runtime/start_mapping.sh --idle
```

打开终端给出的网页地址与 token：8080 为主面板，8081 为手动控制。首次必须重新建图并保存，使地图与关键帧索引成套生成。旧地图不能直接用于自动全局定位，也不能通过修改 YAML 的 resolution 来转换地图。

## 验证与代码入口

本次完成原生 C++ 求解器/整车模型测试和 Python 回归。当前开发环境没有 ROS；新增插件与消息尚需 Jazzy 编译及隔离联调。原版 MPPI 的历史联调结果不能作为新链路的验收证据。方法见 [验证说明](validation/README.md)。尚未连接真实雷达或底盘，模型参数和新串口接口待实测。

| 入口 | 用途 |
|---|---|
| `shaoshao_hw/localizer_node.py`、`policy.py` | 全局检索、配准调度与定位状态 |
| `hw_registration/` | small_gicp 服务；历史 MPPI 插件保留兼容 |
| `hw_omni_controller/` | HW FDDP、XY 整车模型、按时间预测碰撞的 Nav2 控制器 |
| `hw_omni_controller/src/obstacle_predictor.cpp`、`include/hw_omni_controller/obstacles.hpp` | C++ 动态障碍检测、跟踪与预测 |
| `hw_omni_controller/src/command_relay.cpp`、`include/hw_omni_controller/relay.hpp` | C++ 反馈/租约/碰撞门控与 50 Hz 输出 |
| `shaoshao_hw/chassis.py`、`shaoshao_hw_interfaces/msg/` | 网页反馈状态与预留电控接口，执行门控在 C++ |
| `shaoshao_hw/index.py`、`recorder_node.py` | 关键帧与静态索引 |
| `web_panel/navcar_navigation.py` | 参数生成、任务恢复、租约心跳和网页状态 |
| `shaoshao_hw/motion.py` | 速度/加速度/jerk 限制、刹停范围检查 |
| `shaoshao_web/`、`web_panel/stream.js` | C++ 网页服务与 JavaScript WebSocket/二进制点云客户端 |
| `web_panel/supervisor_rpc.py` | 私有本地任务接口 |
| `deployment/` | 独立构建、建图、保存和启动脚本 |

历史通用桥接和 AMCL/DWB 辅助代码保留用于兼容回归，新入口不启动 AMCL 或 DWB。提交改动前运行两套 unittest；修改 ROS/C++ 链路还应运行隔离联调。来源与许可证见 [THIRD_PARTY.md](THIRD_PARTY.md)。
