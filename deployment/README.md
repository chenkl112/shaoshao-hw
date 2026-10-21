# 哨哨 HW 上车部署

适用 Ubuntu 24.04 / ROS 2 Jazzy、MID360、Navcar UART1 V1。源码放在 `~/ccc/shaoshao-hw`，运行文件放在 `~/ccc/shaoshao-hw-runtime`，地图放在 `~/scurm_maps/shaoshao-hw`。不覆盖旧工程的源码、配置或地图。

## 安装与构建

先安装 ROS 2 Jazzy 基础环境，再执行：

```bash
sudo apt update
sudo apt install build-essential cmake git python3-dev python3-colcon-common-extensions \
  python3-numpy python3-scipy python3-pil python3-yaml python3-serial \
  libeigen3-dev libpcl-dev libapr1-dev libcurl4-openssl-dev \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-mppi-controller \
  ros-jazzy-pointcloud-to-laserscan ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
  ros-jazzy-ament-cmake-auto ros-jazzy-tf2-geometry-msgs
git clone https://github.com/chenkl112/shaoshao-hw.git ~/ccc/shaoshao-hw
cd ~/ccc/shaoshao-hw
bash deployment/bootstrap.sh
```

构建脚本拉取固定版本 small_gicp 和 Livox-SDK2，先安装 SDK，再构建六个 ROS 包。最多使用两个编译任务。首次构建可能需要较长时间与 sudo；依赖版本不匹配会报错。运行目录的 `shaoshao` 是指向本源码的兼容软链接。

默认运行路径可通过 `export SHAOSHAO_HW_RUNTIME=/绝对路径` 修改；构建和每个运行终端必须使用相同设置。重复构建保留已生成的串口和雷达网络配置。

## 上车前确认

1. 编辑源码 `web_panel/robot.yaml`：当前车体、雷达尺寸已填好；确认串口路径和 `network` 的 IP。
2. 第一次构建会生成运行目录 `config/serial.navcar.yaml` 和 `config/MID360_config.json`。后续修改串口和雷达网络时，修改这两个运行文件；构建不会覆盖它们。首次构建前修改 `robot.yaml` 对应项也可。
3. 雷达网卡设置为 `192.168.10.100/24`，当前雷达为 `192.168.10.101`。按实际网卡配置，确保雷达网段不与其他接口冲突。
4. 用 `ls -l /dev/serial/by-id/` 确认串口。必要时 `sudo usermod -aG dialout "$USER"` 后重新登录。
5. 确认下位机使用 2,000,000 baud，并对失去指令做停车处理；原平移指令倍率为 10。横移、制动和 yaw 保持效果需实测。

本版不要求新增 MCU 实测轮速或 yaw 反馈。雷达在云台上，导航依据真实雷达/IMU位姿和中心偏移。下位机必须按现有 Navcar 约定保持云台目标世界航向、在云台坐标下执行平移。

## 启动、建图与导航

先将旧工程置于待机并退出旧 8080/8081 控制服务，避免两份上位机争用串口。新工程可复用格式兼容的 sensor-only Livox 驱动，不会接管或停止该驱动。

```bash
# 终端一
bash ~/ccc/shaoshao-hw-runtime/start_move_web.sh
# 终端二
bash ~/ccc/shaoshao-hw-runtime/start_mapping.sh --idle
```

访问 `http://车载电脑IP:8080/?token=终端输出的token`；8081 为手动控制。主网页沿用现有哨哨外观。

1. 点击“建图”，用 8081 或已有遥控方式缓慢走遍场地，覆盖通道入口和两侧墙。开始时静止等待 IMU 初始化，保证点云稳定。
2. 点击“待机”或切换“导航”。程序先通过 `/map_save` 保存静态 PCD，再调用 `/hw/finalize_index` 生成配套索引，然后退出建图。任一步失败会显示错误并保留建图进程。
3. 地图目录应包含 `site.pcd`、`site.png`、`site.yaml`、`keyframes/manifest.json` 和 `keyframes/index.npz`。索引必须 complete 且与 PCD 哈希一致。初次导航必须使用新建地图。
4. 进入“导航”后自动检索当前位置，并用三个新扫描确认；确认后才激活 Nav2。看到“地图定位就绪”后，点击地图预览路线，再点“前往目标”。
5. 定位丢失会停车并保留原目标，恢复后重新规划并续行。“取消导航”、手动接管和切换模式都会终止自动恢复。“重新定位”可主动发起全局搜索。

待机保留雷达预览。退出两份网页用对应终端 Ctrl+C；程序或电脑失联还受上位机与下位机超时保护。

## 参数入口

| 参数 | 文件 / 默认值 |
|---|---|
| 导航合速度 | `web_panel/robot.yaml` → `navigation.max_speed_mps: 0.5` |
| 栅格边长 | 同上 `navigation.map_resolution_m: 0.025`；更改后必须重新建图 |
| 外接圆、光心高度 | 同上 `robot.circumscribed_radius_m`、`lidar_height_m` |
| 光心相对 yaw 轴偏移 | 同上 `yaw.lidar_offset_xy_m` |
| 配准质量与时间限制 | 可在同上 `navigation.registration` 覆盖，完整默认值见 `shaoshao_hw/config.py` |
| 加减速度、jerk | 可在同上 `navigation.motion` 覆盖，默认 0.3 / 0.3 / 1.0 |
| 串口与实测倍率 | 运行目录 `config/serial.navcar.yaml` |
| 雷达网络 | 运行目录 `config/MID360_config.json` |
| MPPI、代价地图、碰撞余量 | `web_panel/navcar_navigation.py` |

不要直接修改生成的 `navigation_runtime/nav2.yaml`，下次切换导航会重新生成。改源码参数后退出导航并重新进入；修改 C++ 后重新运行 bootstrap 并重启相关服务。

质量检查使用归一化 GICP 误差、重叠率、Hessian 最小特征值和条件数，并检查校正跳变。默认误差上限 0.30、重叠率至少 0.80、最小特征值 1000、条件数最多 1e6；局部校正跳变限 0.2 m / 0.1 rad，平滑系数 0.4。GICP 误差不是“米”或“毫米定位精度”，应结合实测日志调整。

有效校正时间以扫描采集时刻计算，拒绝或重放旧结果不会续期。1.6 s 内为正常校正时间窗，超过后只允许有足够近场余量时低速衔接；2 s 后停车并重定位。每 30 s 搜索未成功时等待 5 s 重试。无歧义检查只针对检索到并通过配准的候选，高度对称环境可能无法自动区分。

MPPI 预测 2 s、批量 1000、控制 20 Hz；输出 50 Hz。`wz_max=0`，`wz_std=1e-6` 仅避免内部零方差除法，实际输出角速度强制为零。碰撞模型覆盖任意底盘朝向。局部地图完整新帧用于刹停检查，未知区域按不可通行处理。

## 检查与回退

运行日志在新运行目录：`mapping.log`、`navigation.log`、`web.log`、`lidar.log`；串口终端显示连接与协议状态。网页状态包含配准 metrics、最后有效校正年龄和速度保护诊断。日志中的 token 是访问凭据，分享前隐藏。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ccc/shaoshao-hw-runtime/mapping_install/setup.bash
ros2 topic hz /navcar/state_estimation
ros2 topic echo /hw/localization/status
ros2 topic hz /hw/nav_command
```

回旧版时，在新网页取消导航、进入待机，8081停止，再退出新网页/串口进程，按旧版原入口启动。不要同时启动两套控制服务。旧地图与参数无需替换。

当前完成软件编译与合成数据联调，尚未连接真实 MID360/底盘。首次上车需验证速度换算、失联停车、人工接管、定位误差，再逐步增加速度与测试通道。650 mm 通道对当前外接圆模型余量很小，不承诺可通行。
