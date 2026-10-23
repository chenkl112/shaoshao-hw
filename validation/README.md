# Jazzy 隔离验证

2026-10-23 已在 Ubuntu 24.04 的临时 ROS 2 Jazzy overlay 中，使用 colcon 构建全部八个包。
生产节点为 C++，网页前端为 JavaScript。开发机没有系统安装 ROS，验证依赖解包至 `/tmp`，
未修改车端配置或连接真实硬件。

最新持续目标行为及验收见 [persistent-goal.md](persistent-goal.md) 和 [2026-10-23-persistent-goal.json](2026-10-23-persistent-goal.json)。

八项审查修复及回归见 [review-fixes.md](review-fixes.md) 和 [2026-10-23-review-fixes.json](2026-10-23-review-fixes.json)。

以下为修复前的基础链路记录 [2026-10-23-results.json](2026-10-23-results.json)：

- 八包构建成功；核心/面板 Python 回归 16 + 104 项通过。
- 原生 C++ 运行时 CTest、MPC（含斜向轨迹整段速度约束）、障碍/反馈/指令转发测试通过。
- 实际 C++ HTTP/WebSocket 集成与三套 JavaScript UI 测试通过。
- C++ 导航闭环完成网页预览/目标、CRC/ACK UART、失定位停车、重定位续行、主动重试、人工接管，以及新目标到达后停车。
- C++ 网页 API 建图模式切换、FAST-LIO、二维 PNG/YAML、PCD 和索引保存通过；模拟巡航期望 0.08 m/s，估计约 0.0800 m/s。

全部使用解析合成房间、合成 IMU/点云和 PTY 模拟 MCU，不连接车辆。测试结果仅证明软件链路，不代表真实定位精度或通道通过能力。

在 Linux / Docker 的仓库根目录执行：

```bash
docker build -t shaoshao-hw:jazzy -f validation/Dockerfile .
docker run --name shaoshao-hw-check \
  --mount "type=bind,src=$PWD,dst=/source,readonly" \
  shaoshao-hw:jazzy bash -c \
  'mkdir -p /workspace/src; cp -a /source /workspace/src/shaoshao-hw; bash /workspace/src/shaoshao-hw/validation/run.sh'
docker cp shaoshao-hw-check:/workspace/runtime ./evidence
```

运行脚本会构建全部八个 ROS 包，依次运行核心和继承 Web 回归、C++ MPC/障碍跟踪/反馈门控/速度转发及静态地图测试，以及两套实际 ROS 联调。首次构建下载固定依赖，需要网络。无 host 网络或设备映射；ROS Domain 72，未映射 Web 端口，不影响车端服务。

`ros_integration.py` 启动真实 small_gicp、Nav2 FDDP 插件、C++ 监督进程、C++ Web 和 C++ 串口程序；模拟传感器发布真实时间戳的 ROS 消息。它验证未知位置检索/配准、Nav2 激活、网页预览/目标、CRC/ACK 串口、失定位停车、恢复后重新规划和行驶、主动重定位 RPC，以及在恢复前手动接管能撤销自动续行。模拟 LIO 输入不用于验证 FAST-LIO 本身。

`lio_mapping_integration.py` 单独启动真实 FAST-LIO、索引记录节点和静态投影节点，发送 Livox CustomMsg 与 IMU。已知轨迹从静止加速至 0.08 m/s，检查原生估计速度、当前帧有限正半定协方差、机体点云、`/map_save`、`/hw/finalize_index` 和地图哈希绑定。它不会使用模拟器位姿替代 LIO 输出。
设置 `SHAOSHAO_MAPPING_SUPERVISOR=1` 时，通过真实网页 API 开始建图和停止保存，
覆盖 C++ manager → stack → 投影 PNG/YAML → PCD → 索引的完整模式切换。

以下为修改前 MPPI 版本的历史验收，不能作为新 FDDP 链路证据：ROS 2 Jazzy / Ubuntu 24.04 容器中六包编译通过；Windows 两套 Python 测试共 112 项，其中两个 POSIX 测试跳过，Linux 执行全部。实际 LIO 合成轨迹末段速度约 0.0797 m/s，保存并校验了两个静态关键帧。GICP 在合成已知场景中恢复正确变换；合成误差不等于实车精度。

输出文件：

- `integration-result.json`：GICP/FDDP/预测/Web/UART 联动结果。
- `integration-debug.json`：失败时的网页状态、原始速度和局部地图摘要。
- `lio-validation/result.json`：真实 LIO/建图索引链结果。
- 运行目录的各模块日志与生成配置。日志可能含本地网页 token，公开前删除或隐藏。

简单回归可直接运行：

```bash
python3 -m unittest discover -s tests -v
python3 -m unittest discover -s web_panel -p 'test*.py' -q
```

这些测试不包含实车电机响应、动态行人、650 mm 窄道和长期运行负载。部署后需补充实车日志与刹车测试。

网页前端与 C++ HTTP/WebSocket 服务可以独立验证，方法见 [网页架构](../deployment/网页架构.md)。该独立验证不替代上面的 ROS 车辆链路联调。

## UART1 V2 验收

`navcar_v2_integration.py` 使用独立 ROS Domain 73、C++ 8081 服务和 PTY MCU，检查 SI/旋转符号、底盘系反馈、重复命令序号、两路独立超时、NaN/惯导独立有效性，以及不会自动伪造 ChassisFeedback。`ros_integration.py` 的 MCU 同步更新为 V2，导航必需的同步时间、雷达系速度和硬件授权仍由显式合成传感器提供。新协议不包含这些字段，真实上车边界见 [V2 接入说明](../deployment/NavcarV2接入.md)。
