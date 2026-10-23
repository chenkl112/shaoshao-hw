# 哨哨 HW 上车部署

适用 Ubuntu 24.04 / ROS 2 Jazzy、MID360、四轮全向底盘。新电控协议未定，当前先预留 ROS 接口，详见 [HW 全向导航方案](HW全向导航方案.md)。旧 Navcar UART1 V1 仅保留兼容，不能提供新控制器需要的实测反馈。源码放在 `~/ccc/shaoshao-hw`，运行文件放在 `~/ccc/shaoshao-hw-runtime`，地图放在 `~/scurm_maps/shaoshao-hw`。不覆盖旧工程的源码、配置或地图。

## 安装与构建

先安装 ROS 2 Jazzy 基础环境，再执行：

```bash
sudo apt update
sudo apt install build-essential cmake git python3-dev python3-colcon-common-extensions \
  python3-numpy python3-scipy python3-pil python3-yaml python3-serial \
  libeigen3-dev libpcl-dev libapr1-dev libcurl4-openssl-dev \
  libssl-dev libminizip-dev libpng-dev \
  libdrogon-dev libjsoncpp-dev libtrantor-dev uuid-dev zlib1g-dev libbrotli-dev \
  libhiredis-dev libpq-dev libmariadb-dev libsqlite3-dev libyaml-cpp-dev \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-mppi-controller \
  ros-jazzy-pointcloud-to-laserscan ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
  ros-jazzy-ament-cmake-auto ros-jazzy-tf2-geometry-msgs
git clone https://github.com/chenkl112/shaoshao-hw.git ~/ccc/shaoshao-hw
cd ~/ccc/shaoshao-hw
bash deployment/bootstrap.sh
```

构建脚本拉取固定版本 small_gicp 和 Livox-SDK2，先安装 SDK，再构建八个 ROS 包。最多使用两个编译任务。首次构建可能需要较长时间与 sudo；依赖版本不匹配会报错。运行目录的 `shaoshao` 是指向本源码的兼容软链接。

默认运行路径可通过 `export SHAOSHAO_HW_RUNTIME=/绝对路径` 修改；构建和每个运行终端必须使用相同设置。重复构建保留已生成的串口和雷达网络配置。

## 上车前确认

1. 编辑源码 `web_panel/robot.yaml`：当前车体、雷达尺寸已填好；确认串口路径和 `network` 的 IP。
2. 第一次构建会生成运行目录 `config/serial.navcar.yaml` 和 `config/MID360_config.json`。后续修改串口和雷达网络时，修改这两个运行文件；构建不会覆盖它们。首次构建前修改 `robot.yaml` 对应项也可。
3. 雷达网卡设置为 `192.168.10.100/24`，当前雷达为 `192.168.10.101`。按实际网卡配置，确保雷达网段不与其他接口冲突。
4. 用 `ls -l /dev/serial/by-id/` 确认串口。必要时 `sudo usermod -aG dialout "$USER"` 后重新登录。
5. 确认下位机使用 2,000,000 baud，并对失去指令做停车处理；原平移指令倍率为 10。横移、制动和 yaw 保持效果需实测。

本版需要 `/hw/chassis_feedback` 的真实 XY 速度、底盘角速度、云台相对角与硬件控制权。新串口适配器待协议确定后实现；没有反馈时导航保持停车。上位机发 SI 单位整车目标，电控完成坐标转换、四轮分配和电机闭环。小陀螺使用独立通道，原 V1 目标回显不能代替实测。

## 启动、建图与导航

先将旧工程置于待机并退出旧 8080/8081 控制服务，避免两份上位机争用串口。新工程可复用格式兼容的 sensor-only Livox 驱动，不会接管或停止该驱动。

```bash
# 终端一
bash ~/ccc/shaoshao-hw-runtime/start_move_web.sh
# 终端二
bash ~/ccc/shaoshao-hw-runtime/start_mapping.sh --idle
```

访问 `http://车载电脑IP:8080/?token=终端输出的token`；8081 为手动控制。主网页沿用现有哨哨外观，由 JavaScript 前端和 C++ Drogon 服务提供；内部任务监管、定位调度、地图索引和 8081 串口服务也使用 C++。详见 [网页架构](网页架构.md)。

1. 点击“建图”，用 8081 或已有遥控方式缓慢走遍场地，覆盖通道入口和两侧墙。开始时静止等待 IMU 初始化，保证点云稳定。
2. 点击“待机”或切换“导航”。程序先通过 `/map_save` 保存静态 PCD，调用 `/hw/export_grid` 从该 PCD 重新导出二维 PNG/YAML，再调用 `/hw/finalize_index` 生成配套索引，然后退出建图并记录 `bundle.json`。保存期间暂停网页预览对地图文件的写入。任一步失败会显示错误并保留建图进程。
3. 地图目录应包含 `site.pcd`、`site.png`、`site.yaml`、`keyframes/manifest.json`、`keyframes/index.npz` 和 `bundle.json`。索引必须 complete 且与 PCD 哈希一致。初次导航必须使用新建地图。
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
| 配准质量与时间限制 | 可在同上 `navigation.registration` 覆盖，完整默认值见 `shaoshao_hw_runtime/include/shaoshao_runtime/common.hpp` |
| 加减速度、jerk | 可在同上 `navigation.motion` 覆盖，默认 0.3 / 0.3 / 1.0 |
| 串口与实测倍率 | 运行目录 `config/serial.navcar.yaml` |
| 雷达网络 | 运行目录 `config/MID360_config.json` |
| FDDP 整车响应参数 | `web_panel/robot.yaml` → `navigation.mpc`；0.30 s 为初始假设，待辨识 |
| 动态障碍跟踪 | 同上 `navigation.obstacles`，默认值见 `shaoshao_hw_runtime/include/shaoshao_runtime/common.hpp` |
| 代价地图、碰撞余量 | `shaoshao_hw_runtime/include/shaoshao_runtime/configuration.hpp` |

不要直接修改生成的 `navigation_runtime/nav2.yaml`，下次切换导航会重新生成。改源码参数后退出导航并重新进入；每次进入建图或导航会校验源码配置，生成运行目录 `active_robot.yaml` 快照，各节点使用同一份快照。串口导航限速在新任务申请控制权时同步更新。校验失败会保留上一份有效快照。修改 C++ 后重新运行 bootstrap 并重启相关服务。

质量检查使用归一化 GICP 误差、重叠率、Hessian 最小特征值和条件数，并检查校正跳变。默认误差上限 0.30、重叠率至少 0.80、最小特征值 1000、条件数最多 1e6；局部校正跳变限 0.2 m / 0.1 rad，平滑系数 0.4。GICP 误差不是“米”或“毫米定位精度”，应结合实测日志调整。

有效校正时间以扫描采集时刻计算，拒绝或重放旧结果不会续期。1.6 s 内为正常校正时间窗，超过后只允许有足够近场余量时低速衔接；2 s 后停车并重定位。每 30 s 搜索未成功时等待 5 s 重试。无歧义检查只针对检索到并通过配准的候选，高度对称环境可能无法自动区分。

FDDP 预测 2 s、控制 20 Hz，独立 C++ 节点输出目标 50 Hz；网页只发布任务和租约心跳。局部代价地图使用 map 系，动态障碍预测覆盖 3 s，并按原始观测时间对齐。平移通道不控制小陀螺。碰撞模型继续覆盖任意底盘朝向，未知区域不可通行。独立的二次速度平滑已移除，避免改变 MPC 预测的目标响应。定位退化等原因降低限速时，若当前目标或实测速度仍超限，控制器先进入零速等待；实际速度下降后以新的目标历史恢复 MPC，保留路径和控制权。安全停车优先于普通运动时的目标加速度/jerk限制。

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

C++ 运行链路已在临时 Jazzy 环境构建并完成合成输入联调，详见 [验证记录](../validation/README.md)。尚未连接真实 MID360/底盘。首次上车需验证速度换算、失联停车、人工接管、定位误差，再逐步增加速度与测试通道。650 mm 通道对当前外接圆模型余量很小，不承诺可通行。

C++ 串口导航租约在 0.5 s 无有效新指令后撤销，运动指令仍使用 0.2 s 看门狗；过期租约不能靠重放指令恢复。任务心跳丢失后，relay 只短暂发送零速，不会无限续租。控制页面会话有容量和过期清理，停止请求不受会话容量或未使用的速度输入影响。自动续行遇到规划或控制权申请失败时保留目标，等待 5 s 后重试；人工接管、取消任务、地图变化会撤销续行。
