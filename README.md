# 哨哨 HW

面向 i7-12800HX、麦轮底盘和顶置 MID360 的独立导航工程。沿用哨哨网页的点云、地图、选点，以及 **建图 / 导航 / 待机** 三个模式；定位改为 FAST-LIO + small_gicp，局部控制改为 Nav2 MPPI Omni。

从 [shaoshao](https://github.com/chenkl112/shaoshao) 的 `debd22c1089ef2421fdb5667d0f08007d4aa9e7c` 独立建立，参考 [HWSentryNav26](https://github.com/Polyacetone/HWSentryNav26) 的配准质量评估思路。原仓库保持独立。上车步骤见 [部署说明](deployment/README.md)，串口格式见 [Navcar 接入](web_panel/Navcar接入.md)。

## 本版行为

- FAST-LIO 连续输出局部位姿、内部估计速度和当前帧协方差；不对位置做差分来生成速度。
- 建图同步记录关键帧；保存 PCD 后剔除不在静态地图中的点，生成带地图哈希的检索索引。
- 导航启动后自动检索位置、进行 small_gicp 配准，再用三个独立新扫描确认。匹配不唯一时保持停车。
- 只有通过质量与跳变检查的校正才平滑写入 `map → navcar_odom`。拒绝一次校正不会抹掉已有定位，也不会刷新有效校正时间。
- 短暂退化只在近场余量充足时以至多 **0.05 m/s** 衔接，最长有效校正年龄 **2 s**、累计距离 **5 cm**。窄处、扫描/里程计超时直接停车。
- 定位丢失后保留原目标，停车、全局定位、重新规划，再继续。取消、手动接管、切换模式和地图变化会阻止自动续行。
- MPPI 20 Hz 规划全向平移，合速度上限 **0.5 m/s**；50 Hz 输出限制加速度 **0.3 m/s²**、jerk **1 m/s³**。紧急停车优先。导航角速度始终为零。
- 原 Navcar UART1 V1 协议保持不变。ROS 速度经有效期、控制租约和序号校验后发送到串口；ACK 和目标回显不作为实测姿态/速度。

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
| 串口 / 倍率 | 2,000,000 baud；前后和左右 × 10 |

由于没有实测底盘 yaw，本版以覆盖任意底盘朝向的外接圆检查碰撞。含余量的直径约 **635.3 mm**。650 mm 通道只有约 **7.3 mm/侧** 的剩余几何空间，2.5 cm 栅格离散、地图和控制误差仍可能使其不可通行，不能保证通过。

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

Jazzy 下已完成六个 ROS 包编译和隔离合成数据联调。方法和边界见 [验证说明](validation/README.md)。没有连接真实雷达或底盘，实车定位阈值、横向倍率、刹车距离和窄道通过能力仍需上车测量。

| 入口 | 用途 |
|---|---|
| `shaoshao_hw/localizer_node.py`、`policy.py` | 全局检索、配准调度与定位状态 |
| `hw_registration/` | small_gicp 服务、MPPI 合速度约束插件 |
| `shaoshao_hw/index.py`、`recorder_node.py` | 关键帧与静态索引 |
| `web_panel/navcar_navigation.py` | MPPI 参数、自动恢复、速度输出与碰撞保护 |
| `shaoshao_hw/motion.py` | 速度/加速度/jerk 限制、刹停范围检查 |
| `deployment/` | 独立构建、建图、保存和启动脚本 |

历史通用桥接和 AMCL/DWB 辅助代码保留用于兼容回归，新入口不启动 AMCL 或 DWB。提交改动前运行两套 unittest；修改 ROS/C++ 链路还应运行隔离联调。来源与许可证见 [THIRD_PARTY.md](THIRD_PARTY.md)。
