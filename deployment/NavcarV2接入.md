# Navcar UART1 V2 接入

固定来源：Li-chee/Navcar-Comm-Protocol `43333c533538490c1e2262a63a004cde088d7f79`。
[上游原文快照](protocols/navcar-uart1-v2.md)，MIT 许可证见 `LICENSES/Navcar-Comm-Protocol.txt`。
生产适配使用 C++ `hw_move_web`；Python UART 脚本仅保留 V1 历史兼容。

## 版本与配置

文档升级到 V2，线上 VERSION 字节仍为 `0x01`，不能按帧头或 ACK 自动识别固件版本。
帧头、2000000 baud、CRC16/MODBUS、16 字节四浮点载荷、50 ms ACK 等待及最多三次原样重试均不变。
首次控制序号使用最近状态序号 +1，按 uint16 回绕；状态序号可以在约 100 Hz 的多个测量帧中重复。
不把命令序号当成采样序号，也不要求实测值与目标相等。

默认配置是 `web_panel/serial.navcar.example.yaml`，`robot.yaml` 的 transport 同为 `navcar_uart1_v2`。
运行目录 `config/serial.navcar.yaml` 会保留用户设置，旧文件不会被构建脚本覆盖。
确认固件已升级后，用新模板迁移该文件、填写实际串口；源码与串口配置版本不一致时服务拒绝启动。
V2 拒绝旧倍率配置（例如 forward_units_per_mps=10）；平移上限用 `max_translation_mps`，旋转上限用 `max_rot_radps`。
旧固件使用 `serial.navcar.v1.example.yaml`，同时明确设置 robot.yaml 的 transport 为 V1。

## 控制与反馈

| 数据 | 线上含义 | 本地处理 |
|---|---|---|
| 控制 vy | 云台/雷达系前进，m/s | ROS forward 原值下发 |
| 控制 vx | 云台/雷达系向右，m/s | ROS left 取负号，不乘倍率 |
| 控制 rot | 底盘角速度，逆时针为正，rad/s | 正号下发 |
| 控制 yaw_angle | 云台相对启动方向的目标角，度 | 保持目标，不用后续实测值覆盖 |
| 状态 vy/vx | 底盘前后/左右轮速反解，m/s | 发布 body forward / left=-vx |
| 状态 rot | 轮速反解底盘角速度，rad/s | 与指令分开保存 |
| 状态 yaw_angle | 云台相对启动方向的惯导角，度 | 发布 startup yaw，不能当相对底盘角 |

首次有效惯导角只用于初始化保持目标；之后网页 `/api/yaw` 修改目标，周期状态不会覆盖它。
轮速解算基于理想滚动，打滑/悬空时不等于独立测得的对地速度。
任一速度为 NaN/Inf 时明确标记 velocity_valid=false，惯导有效性单独报告；不会将无效帧当丢包而继续显示上一次有效速度。
无效速度或惯导期间，运动输出归零；重新收到有效值后才允许新控制。

`GET /api/state` 的 V2 数据放在 `measurements` 与 `yaw_actual_deg`，`feedback_targets` 为 null。
界面明确区分目标角和惯导实测角；JSON 中无效浮点显示为 null，保留 validity 字段。

## ROS 接口与导航边界

`/hw/navcar/feedback` 使用 `NavcarFeedback`：frame_id 为 `navcar_chassis`，body x 前、y 左、z 上。
header.stamp 明确表示主机接收时间，不是 MCU 采样时间；reception_seq 是主机接收计数。
last_command_seq 是 MCU 最近接受的指令序号，允许重复，不用于反馈重放判定。

当前 V2 未回传以下数据：

- DR16/底盘当前控制模式和硬件接管代次。ACK=0 仅表示协议接受，不表示 UART1 正在驱动电机。
- 云台相对底盘的编码器角；惯导 startup yaw 不提供这项角度。
- MCU 采样时间及 IMU 更新年龄。

因此服务不直接将这份消息转换成 `/hw/chassis_feedback`。
自动导航仍要求外部真实接口补齐硬件授权、采样同步和相对角，完成 body → 雷达坐标系与参考点变换后提供 ChassisFeedback。
仅靠本协议四个值接入时，网页会等待有效导航反馈并保持停车。模拟验收中的这几项来自明确标注的合成传感器，不代表固件具备这些字段。

## 独立旋转与停止

V2 `/api/spin` 有独立 200 ms 看门狗，可在导航平移期间更新，不撤销平移租约或推进人工接管代次。
`/hw/spin_command` 使用当前导航 lease 授权，独立 seq、时间戳和 200 ms 有效期；lease 撤销后对应旋转停止。
旋转消息不更新平移看门狗，平移消息也不关闭或续期旋转。两路最终组合成一个四浮点控制帧。
整个页面的紧急停止、失焦/隐藏仍关闭两路；只关闭旋转不会取消正在运行的导航平移。
硬件拨杆实际模式仍由固件决定，不能以软件 lease 替代。

ACK、CRC、状态超时等通信错误仍停止发运动帧并撤销租约；不靠旧重复帧恢复运动。
错误恢复需重新连接/重启串口服务，重新读取状态同步后由用户建立新任务。

## 验收记录

8 个 ROS 包构建、原生 runtime/V2/HW 导航 CTest、120 项 Python 回归和四套网页回归通过。
独立 ROS + PTY 测试覆盖 V2 数据、NaN、版本不匹配保护和独立看门狗；完整导航模拟覆盖原目标到达、脱困、定位丢失、人工接管和参数重载。
详见 [2026-10-24-navcar-v2.json](../validation/2026-10-24-navcar-v2.json)。合成测试结果不代表实车验收。
