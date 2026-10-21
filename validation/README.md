# Jazzy 隔离验证

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

运行脚本会构建全部六个 ROS 包，依次运行核心和继承 Web 回归、C++ 静态地图测试，以及两套实际 ROS 联调。首次构建下载固定依赖，需要网络。无 host 网络或设备映射；ROS Domain 72，未映射 Web 端口，不影响车端服务。

`ros_integration.py` 启动真实 small_gicp、Nav2 MPPI、Web 和串口程序；模拟传感器发布真实时间戳的 ROS 消息。它验证未知位置检索/配准、Nav2 激活、网页预览/目标、CRC/ACK 串口、失定位停车、恢复后重新规划和行驶，以及在恢复前手动接管能撤销自动续行。模拟 LIO 输入不用于验证 FAST-LIO 本身。

`lio_mapping_integration.py` 单独启动真实 FAST-LIO、索引记录节点和静态投影节点，发送 Livox CustomMsg 与 IMU。已知轨迹从静止加速至 0.08 m/s，检查原生估计速度、当前帧有限正半定协方差、机体点云、`/map_save`、`/hw/finalize_index` 和地图哈希绑定。它不会使用模拟器位姿替代 LIO 输出。

本机验收环境为 ROS 2 Jazzy / Ubuntu 24.04 容器：六包编译通过；Windows 两套 Python 测试共 112 项，其中两个 POSIX 测试跳过，Linux 执行全部。实际 LIO 合成轨迹末段速度约 0.0797 m/s，保存并校验了两个静态关键帧。GICP 在合成已知场景中恢复正确变换；合成误差不等于实车精度。

输出文件：

- `integration-result.json`：GICP/MPPI/Web/UART 联动结果。
- `integration-debug.json`：失败时的网页状态、原始速度和局部地图摘要。
- `lio-validation/result.json`：真实 LIO/建图索引链结果。
- 运行目录的各模块日志与生成配置。日志可能含本地网页 token，公开前删除或隐藏。

简单回归可直接运行：

```bash
python3 -m unittest discover -s tests -v
python3 -m unittest discover -s web_panel -p 'test*.py' -q
```

这些测试不包含实车电机响应、动态行人、650 mm 窄道和长期运行负载。部署后需补充实车日志与刹车测试。
