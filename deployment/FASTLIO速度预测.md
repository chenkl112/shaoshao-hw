# FAST-LIO 速度作为唯一预测状态

导航使用 MID360 雷达和 IMU 经 FAST-LIO 融合后的速度估计，不使用轮速反馈作为速度初值或备用源。
IMU 直接提供加速度和角速度；这里的线速度是 FAST-LIO 滤波器的速度状态，并非对裸 IMU 加速度自行积分。

## 数据链

`FAST-LIO /navcar/state_estimation`（navcar_odom → navcar_imu）
→ C++ 中心点补偿 → `/navcar/odom`（navcar_odom → navcar_center）
→ HW 全局规划初速度、RouteTracker、MPC、独立刹停检查、Nav2 到位/停止判断。

FAST-LIO 发布 `R_odom_imuᵀ * state.vel` 和去偏角速度，twist 的方向遵循 child_frame_id。
IMU 到控制参考点的偏移来自生成的 frames.yaml，使用 `v_center = v_imu + omega × offset`，
同时变换 twist/pose 协方差。保留 LiDAR 帧结束时间戳，不用接收或控制循环时刻给旧速度续期。
地图姿态与速度时间不同步时，使用该 LIO 角速度补偿到速度采样姿态，再转换地图系速度。
小陀螺底盘角速度、轮速和 UART 惯导启动角均不参与此变换。

网页桥接已移除原先用电控反馈覆盖 Odometry.twist 的逻辑。
生产规划、控制、转发和任务节点不再订阅 `/hw/chassis_feedback`。
旧 ChassisFeedback 消息仅为历史兼容保留；向它发布任意数值不会影响当前导航。

## 有效性与恢复

- LIO 的时间戳、坐标系、位置、姿态、线/角速度和协方差必须有效。
- 默认最大年龄 150 ms；非有限值、非对称/非半正定速度协方差会停止运动。
- 平面速度协方差最大特征值不超过 `lio_velocity_std_limit_mps²`，默认标准差上限 0.5 m/s。
- 失效期间停车、保留原目标；收到新的有效 LIO 数据、定位和障碍信息后重新规划原目标。
- 重放、迟到或重复时间戳不能刷新速度，也不能清除较新的故障。
- 不回退到轮速，也不根据位置差分生成速度。

参数位于 robot.yaml 的 navigation.mpc：`lio_timeout_s`、`lio_velocity_std_limit_mps`。
原 tau_x_s/tau_y_s 仍描述目标指令到真实运动的整车响应，当前初值 0.30 s，需实车辨识。
LIO 速度并非无误差，低纹理、打滑和剧烈运动时需结合协方差与定位质量实测调整。

## 控制权单独接入

`/hw/control_authority` 使用 `ControlAuthority`，包含 header、seq、authority_epoch、control_allowed。
由真实拨杆/底盘模式适配器周期发布；只有授权，不含速度、底盘角速度或云台相对角。
接管/撤权会取消目标；过期会停车等待。重新授权不应自动启动已取消的目标。

UART1 V2 当前仍未提供拨杆模式，所以不能从 ACK、轮速是否为零或软件 lease 推断此授权。
改用 LIO 后，预测不再需要电控的采样时间戳和云台相对底盘角；实车自动导航还需补上这一路真实授权。

`/hw/navcar/feedback` 继续供诊断，目标与轮速反解值仍分开保存。串口 NaN 仍作为电机反馈故障停车互锁，
不会被拿来初始化、校正或替代 MPC 的速度状态。

## 验证

原生回归覆盖参考点偏移、协方差变换、样本姿态与当前姿态对齐、LIO 过期/NaN/不确定度/重放及独立授权。
ROS + PTY 模拟发布明显错误的轮速和旧 ChassisFeedback，同时使用正确 LIO 与 ControlAuthority，
检查导航能够到达；注入 LIO 高不确定度后必须停车，恢复后继续原目标。
合成测试不等于实车精度或电机联调验收。

验收记录：[2026-10-24-lio-motion.json](../validation/2026-10-24-lio-motion.json)。8 包构建、四项 CTest、原生 relay、120 项 Python 回归和四套网页回归通过；完整 ROS 链路确认错误轮速不影响预测，LIO 故障停车后可恢复原目标。
