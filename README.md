# one_drone

基于 **ROS 2 Humble、OpenVINS / MicoVIO、EGO-Planner 和 PX4** 的单无人机导航项目。

无人机通过前视双目与 IMU 或下视 MicoVIO 模组估计位置，通过前视深度图建立局部障碍地图；在 RViz 中指定目标后，规划三维轨迹，并将速度与偏航角速度指令发送给飞控。

## 1. 项目组成

```text
左右红外图像 + IMU → OpenVINS → /odom、odom→base_link
                                      ↓
深度图 + CameraInfo + 相机外参 → EGO 局部占用地图
                                      ↓
RViz 目标 → goal_manager → EGO 轨迹规划 → 轨迹控制 → PX4 Offboard
```

- **定位**：可选择 OpenVINS 双目惯性里程计，或通过 microvio 读取 PX4 融合后的 MicoVIO 定位，统一发布 `/odom` 和 TF。只有 OpenVINS 链路向 PX4 写入外部视觉。
- **建图与避障**：EGO 使用有效深度观测建立三维占用栅格，并按机体尺寸和安全余量膨胀障碍物。未知区域按当前配置允许通行。
- **导航**：RViz 打点后，目标转换到 `odom` 坐标系，由 EGO 规划轨迹，控制模块向 PX4 发送设定值。
- **决策**：进入 PX4 OFFBOARD 后选择航点行为，支持单次执行与循环巡航，通过现有目标管理器调用 EGO。
- **先验地图**：PGM 地图用于 RViz 显示、定位参考和打点；碰撞检测使用深度生成的局部地图。
- **仿真**：Gazebo 提供环境、相机、IMU 和动力学，定位算法使用传感器数据，不读取仿真真值位姿。

TF 关系为：

```text
map → odom → base_link → camera_optical
```

`map→odom` 由 modify 节点按配置及 RViz 面板调整发布，`odom→base_link` 由所选定位节点发布。默认将首次接受定位时的机体位置设为 `odom` 原点（XYZ=0），机头水平朝向定义为 `odom` 的 +X（yaw=0），因此初始 odom 与 base_link 位置重叠；后续位移和转动正常跟随，保留真实 roll/pitch。OpenVINS 和 microvio 包内的 `initial_position_zero`、`initial_yaw_zero` 分别控制这两个约定；固定变换只在节点启动后锁定一次，定位恢复或重置不重新归零；该坐标变换不修改飞控原始视觉输入，也不自动修改 `map→odom`。RViz 使用 `map` 作为 Fixed Frame，EGO 在 `odom` 下规划和控制。

### 目录

```text
src/
├── bringup/       # 总启动文件、启动参数、EGO 参数、先验地图和 RViz 配置
├── perception/    # 相机话题转发、软件双目深度、深度过滤、调试点云
├── localization/  # OpenVINS、定位桥及 microvio 模组遥测接入
├── navigation/    # EGO 规划器、里程计适配、目标管理
├── decision/      # 行为选择与航点执行，支持 once/loop
├── control/       # 轨迹执行、飞行状态管理、PX4 指令适配
└── rviz/          # map→odom 调整插件、导航 TF 显示过滤

deploy/calibration/ # 真机标定与安装外参
tools/              # 标定导入、仿真配置工具
scripts/            # 仿真启动脚本
worlds/             # 仿真场景
```

## 2. 安装与编译

运行环境：**Ubuntu 22.04、ROS 2 Humble、PX4 1.14.3**。真机相机使用 D435i，还需要安装并配置 RealSense ROS 2 驱动及 librealsense。

首次安装：

```bash
git clone https://github.com/PingTIKES/one_drone.git ~/one_drone
cd ~/one_drone
bash setup_env.sh sim
```

真机计算设备使用 `bash setup_env.sh onboard`。安装脚本安装项目依赖并构建工作区；仿真模式还准备外部 PX4 SITL。OpenVINS 源码已包含在项目中，PX4 仿真源码默认位于 `~/PX4-Autopilot-1.14.3`，可通过 `PX4_DIR` 指定。

已有项目更新与日常编译：

```bash
cd ~/one_drone
git pull --ff-only
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

首次完整编译时，可限制并行度以降低内存占用：

```bash
MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install --executor sequential
```

## 3. 启动前配置

日常启动选项统一放在 [src/bringup/params/launch.yaml](src/bringup/params/launch.yaml)，启动命令无需追加这些参数。

| 配置项 | 用途 |
| --- | --- |
| `startup.sim` | `true` 使用仿真配置；`false` 使用真机配置 |
| `startup.rviz` | 是否打开 RViz |
| `startup.yaw_control` | `true` 使用现有平移/yaw 解耦策略；`false` 固定首次有效 PX4 航向，不随路径转向 |
| `startup.flight_control` | `false` 只测试感知、定位和规划；`true` 启动飞控控制节点及起飞/降落服务 |
| `hardware.localization_source` | `openvins` 使用相机定位；`microvio` 读取飞控 USB 融合定位 |
| `hardware.perception_enabled` | `false` 关闭深度处理，仍启动 EGO、目标管理和决策；有相机时设 `true` 更新障碍物 |
| `hardware.calibration_dir` | 当前设备的标定目录 |
| `hardware.target_system`、`hardware.px4_ns` | 飞控编号及 DDS 名称空间 |
| `hardware.cam0_topic`、`cam1_topic`、`imu_topic` | 与标定对应的左右目和 IMU 输入 |
| `hardware.depth_source`、`depth_topic`、`depth_info_topic` | 深度来源及匹配的 CameraInfo |

启动模式以 `launch.yaml` 的实际值为准。仿真飞行将 `startup.sim` 和 `startup.flight_control` 都设为 `true`；真机定位台架检查使用 `sim: false`、`flight_control: false`。

`yaw_control: false` 会在首次收到有效 PX4 本地位置时锁定机头方向，位置保持、起飞和轨迹跟踪都使用该固定 yaw；平移仍独立执行，不再按路径朝向或前视角度缩放水平速度，因此允许侧向和后向移动。定位暂时恢复或 HOLD 不重新锁定；PX4 航向参考重置时修正固定参考值，保持同一物理方向。`true` 保持原有解耦 yaw 策略。修改后重启控制节点生效，此参数不改变 map→odom 或 RViz TF。


### 参数文件位置

| 内容 | 文件 |
| --- | --- |
| 总启动配置 | `src/bringup/params/launch.yaml` |
| EGO 建图、膨胀、规划与优化 | `src/bringup/params/ego_params.yaml` |
| 先验地图选择 | `src/bringup/params/global_config.yaml` |
| PGM 地图及配套 YAML | `src/bringup/map/` |
| RViz 默认显示 | `src/bringup/rviz/ego_navigation.rviz` |
| map→odom 初值 | `src/rviz/modify_map_to_odom/config/config.yaml` |
| 飞行控制、速度、起飞高度及偏航 | `src/control/flight_bridge/config/params.yaml` |
| 打点目标高度 | `src/navigation/goal_manager/config/params.yaml` |
| 决策航点、循环模式及到点容差 | `src/decision/decision_manager/config/decision.yaml` |
| 定位健康检查 | `src/localization/vio_bridge/config/params.yaml` |
| 仿真 OpenVINS 参数 | `src/localization/vio_bridge/config/openvins_sim/estimator_config.yaml` |
| 真机 OpenVINS 参数与标定 | `calibration_dir` 中的 YAML 文件 |
| 感知节点参数 | `src/perception/<功能包>/config/params.yaml` |

ROS 参数文件附有中文说明。修改源码、launch 或包内参数后执行 `colcon build --symlink-install`，再重启算法；真机标定目录由启动时直接读取，修改后重启即可。

整机半径已设为 **0.10 m（包含桨叶）**，安全余量 **0.10 m**，总膨胀参数为 **0.20 m**。当前体素边长为 0.15 m，膨胀向上取整为两格，沿坐标轴的范围为 0.30 m；这两项尺寸参数在 `ego_params.yaml` 中分别调整。

## 4. 仿真启动

先在 `src/bringup/params/launch.yaml` 中将以下两项改为 `true`，其余配置保留：

```yaml
startup:
  sim: true
  flight_control: true
```

`flight_control: false` 不启动 `/takeoff` 服务，调用时会一直等待。修改配置后需要重启算法；已有实机节点应先在其启动终端按 `Ctrl+C` 退出。

**终端 A：启动 Gazebo、PX4 和 DDS Agent。**

```bash
cd ~/one_drone
bash scripts/start_algorithm_sim.sh
```

等待 `[sim] Ready.`。默认场景使用原始 3 m 墙体场地，不添加识别柱；无图形环境可使用 `HEADLESS=1 bash scripts/start_algorithm_sim.sh`。

默认出生位置为 `(1.3, 9.4, 0.5)`，yaw 为 `-π/2`，机头与前视相机朝场内纵向（Gazebo 世界 −Y）。可通过 `SPAWN_POSE=x,y,z,roll,pitch,yaw` 环境变量覆盖，角度单位为 rad；修改后需重启仿真。该朝向属于仿真世界，不代表 OpenVINS 的初始 `odom` 航向。

**终端 B：启动算法与 RViz。**

```bash
cd ~/one_drone
source /opt/ros/humble/setup.bash
source install/setup.bash
source /tmp/one_drone_gz_env.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch bringup startup.launch.py
```

## 5. 真机启动

使用 OpenVINS 和深度导航时，在 `launch.yaml` 中设置 `startup.sim: false`、`hardware.localization_source: openvins`、`hardware.perception_enabled: true`，核对相机话题、标定目录、飞控编号及 DDS 名称空间。初次检查定位时保持 `startup.flight_control: false`。

### 标定文件

当前 D435i 标定位于 [deploy/calibration/uav1](deploy/calibration/uav1/README.md)。更换设备、图像模式或安装位置时更新对应标定。

| 文件 | 内容 |
| --- | --- |
| `estimator_config.yaml` | OpenVINS 运行参数 |
| `kalibr_imucam_chain.yaml` | 双目内参、畸变、相机与 IMU 外参、时间偏移 |
| `kalibr_imu_chain.yaml` | IMU 噪声及采样参数 |
| `body.yaml` | 机体到 IMU 的安装变换 `T_body_imu` |

可通过 `tools/import_kalibr.py` 导入 Kalibr 结果，参数说明使用 `python3 tools/import_kalibr.py --help` 查看。当前 D435i 的深度安装外参可由机体安装关系和左目外参推导；其他设备或深度对齐模式需提供对应外参。

### 终端 A：相机驱动

以下模式对应当前标定：双红外 `640×480 Y8 @ 30 Hz`、硬件深度和组合 IMU。

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false enable_depth:=true \
  depth_module.depth_profile:=640x480x30 \
  enable_infra1:=true enable_infra2:=true \
  depth_module.infra_profile:=640x480x30 \
  depth_module.infra1_format:=Y8 depth_module.infra2_format:=Y8 \
  enable_gyro:=true enable_accel:=true \
  gyro_fps:=200 accel_fps:=63 unite_imu_method:=2
```

相机启动后，在另一终端关闭红外发射器：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 param set /camera/camera depth_module.emitter_enabled 0
```

### 飞控通信

飞行控制使用 PX4 uXRCE-DDS 客户端与电脑上的 Micro XRCE-DDS Agent。当前 MicoVIO 占用 TELEM2，飞控原生 USB 使用 MAVLink（供 microvio/QGroundControl），不能将这个 USB 设备直接当作 DDS 串口。

推荐将空闲 **TELEM1** 接到电脑的 **3.3 V USB-TTL** 转换器（TX→RX、RX→TX、GND→GND），或接 RK3566 上电平匹配且已启用的 UART。按飞控接口定义确认引脚，不将 RS-232 电平直接接入飞控。

通过飞控原生 USB 在 QGroundControl 参数页面设置并保存：

| 参数 | 值 | 含义 |
| --- | --- | --- |
| `MAV_0_CONFIG` | `0` | 当前它占用 TELEM1，先释放；其他 MAVLink 实例也不能占用 TELEM1 |
| `UXRCE_DDS_CFG` | `101`（TELEM1） | DDS 客户端串口 |
| `SER_TEL1_BAUD` | `921600` | 两端串口波特率 |
| `UXRCE_DDS_DOM_ID` | `0` | 与电脑 ROS_DOMAIN_ID 一致 |
| `UXRCE_DDS_KEY` | `1` | 单机客户端标识，不能为 0 |

**保留 TELEM2 的 MAVLink 配置，不要禁用 MicoVIO 链路。** 重启飞控后，在 QGroundControl 的 MAVLink Console 执行 `uxrce_dds_client status` 查看运行状态。操作参数前先停止占用飞控 USB 的 microvio；参数配置完成后退出 QGroundControl，再使用 microvio 读取该 USB。

电脑端启动 Agent，下面 `/dev/ttyUSB0` 是新 USB-TTL 转换器的示例设备名，须替换为实际路径（推荐 `/dev/serial/by-id/...`），与飞控原生 USB 的 `/dev/ttyACM0` 不同：

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

各 ROS 终端 `export ROS_DOMAIN_ID=0` 并统一 RMW 环境，确认 `/fmu/out/vehicle_status`、`/fmu/out/vehicle_local_position` 持续发布。有 DDS 前缀时使用实际名称空间，并同步填写 `hardware.px4_ns`。`px4_msgs` 必须匹配飞控固件；当前项目对应 PX4 1.14。仅运行 microvio 独立 USB 定位检查时无需 DDS Agent。

参数依据：[PX4 1.14 DDS 配置](https://docs.px4.io/v1.14/en/middleware/uxrce_dds)。本节说明的是接线和配置步骤，不表示检测到新 TELEM1 链路已接通。

### 终端 B：算法与 RViz

```bash
cd ~/one_drone
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch bringup startup.launch.py
```

先进行无桨检查：静置初始化后缓慢移动机体，确认 TF、位移尺度、点云方向及回到原处的误差。飞行联调前检查飞控标定、遥控接管与失控保护，再将 `startup.flight_control` 改为 `true` 并重启算法。仿真结果不能代替真机验证。

### MicoVIO 模组定位检查

当前真机配置为无相机定位检查：`sim: false`、`flight_control: false`、`hardware.localization_source: microvio`、`hardware.perception_enabled: false`。MicoVIO 接飞控 TELEM2，电脑 USB 接飞控。在 source 工作区并统一 RMW 环境后启动：

```bash
ros2 launch bringup startup.launch.py
ros2 topic echo /vio_health
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
```

总启动保留先验地图、modify、RViz 和 `/ego/odom`，由 microvio 发布 `/odom`、`/vio_health`、`/vio_diagnostics`、`/vio_reset_event` 及 `odom→base_link`。OpenVINS 和 VIO→PX4 回灌节点互斥关闭。无相机时仍启动 EGO、目标管理和决策，可使用有效 microvio 里程计在地面打点规划；没有深度观测就没有实时障碍信息，PGM 仍仅用于显示和打点。下视 ToF 不能替代前视障碍地图。接回相机后启用 `perception_enabled: true` 并核对深度安装外参，即可更新局部障碍地图。控制执行以 PX4 OFFBOARD 为准，无深度不阻止执行有效 EGO 指令。

也可使用独立包读取飞控融合定位（不要与总启动同时运行）：

```bash
ros2 launch microvio microvio.launch.py
```

使用前 source 本工作区并按真机章节统一 RMW 环境。串口、波特率及输出话题放在 `src/localization/microvio/config/params.yaml`，输出为 `/microvio/odom`、`/microvio/health` 和 `/microvio/diagnostics`。独立入口只检查定位，不启动飞控控制；总启动通过上述 `localization_source` 选择定位来源。[接线、启动及导航接入说明](src/localization/microvio/README.md)。

### 笔记本通过网络观察 RK3566

RK3566 运行算法，笔记本运行 RViz。SSH 只用于启动板端程序，ROS 话题通过局域网 DDS 传输，不使用 SSH X11 转发。两端需网络互通、允许 DDS 发现和数据通信，并同步系统时间。

**两端终端均设置相同环境**（下面使用 domain 0，须与飞控 DDS 配置一致）：

```bash
source /opt/ros/humble/setup.bash
source ~/one_drone/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

**RK3566（SSH 终端）**：将板端 `src/bringup/params/launch.yaml` 的 `startup.rviz` 设为 `false`，按实机配置启动：

```bash
ros2 launch bringup startup.launch.py
```

**笔记本本地终端**：只启动观察入口，不在笔记本重复启动总 launch：

```bash
ros2 launch bringup rviz.launch.py
```

笔记本入口只运行 RViz 和本地 TF 显示过滤器，不打开飞控串口、不启动定位、规划或飞控控制。默认白底、map 俯视，显示先验地图、里程计、导航 TF、全局路径、局部路径和目标；点云及图像默认关闭，可在 Displays 中手动打开。这是纯观察配置，仅保留视角操作；没有 2D Goal Pose、Publish Point 或 MapOdomModify 面板，不发布导航目标或 TF 调整。

远程参数位于 `src/bringup/params/rviz_remote.yaml`，显示配置为 `src/bringup/rviz/remote_navigation.rviz`。实机 `use_sim_time: false`；远程看仿真时改为 `true`，并确保接收 `/clock`。笔记本使用 `rviz_tf_shift` 包中的 TF 显示过滤脚本，不加载 modify 插件。

若没有数据显示，先在笔记本执行 `ros2 topic list`、`ros2 topic hz /ego/odom`，确认能发现并接收板端数据；仅 SSH 连接成功不代表 DDS 网络已经打通。不要在笔记本再启动 map_server 或 modify 发布者来掩盖网络问题。

## 6. 起飞、打点与降落

启用飞控控制并确认定位与飞控连接正常后，在**新终端 C** 设置与算法相同的环境。仿真使用：

```bash
source /opt/ros/humble/setup.bash
source ~/one_drone/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

真机按第 5 节使用 `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`。若设置了 `ROS_DOMAIN_ID`，各终端也需一致。

确认服务存在后起飞：

```bash
ros2 service type /takeoff
ros2 service call /takeoff std_srvs/srv/Trigger '{}'
ros2 topic echo /flight_state
```

第一条应输出 `std_srvs/srv/Trigger`；若没有服务，先检查算法终端是否启动了 `flight_bridge`。

手动起飞后可直接用遥控器切入 OFFBOARD，不需要调用 `/takeoff` 或 `/resume_navigation`。控制节点会提前发布静止设定值及心跳，以满足 PX4 切入 OFFBOARD 的协议要求；在其他飞行模式下不会主动解锁或切换模式。进入 OFFBOARD 后，无可用 EGO 指令时锁定当前位置，有新鲜且数值有效的 EGO 指令和里程计时执行轨迹速度。退出 OFFBOARD 后释放 EGO 控制，取消尚未完成的起飞流程，不会自动切回或重新解锁。

地图、深度、VIO 健康字符串均不作为 OFFBOARD 执行门控，也不等待额外恢复确认。指令或必需里程计过期时保持位置，新指令恢复后自动继续；若 PX4 位置本身不可用，只能请求零速度，不能保证水平定点。VIO 重置或规划器重启会丢弃旧指令，接收新坐标阶段的数据后继续。

`/takeoff` 保留为可选的一键起飞服务：需要有效 PX4 本地位置，用当前高度加 `takeoff_altitude` 计算目标，再请求 OFFBOARD 和解锁。手动已进入 OFFBOARD 时不会重复执行起飞。无定位时人工离地不经过这个服务。microvio 的首次坐标归零不再等待估计器辅助标志；恒定位置模式等状态仍如实发布在健康诊断中。

路径规划与飞控执行独立：有效里程计、TF 和规划器就绪后即可打点，未起飞、无深度相机、`flight_control: false` 均不阻止规划。EGO 始终启动；`perception_enabled` 只控制深度处理。有深度时更新障碍物，没有深度时按未知区域允许通行的原有策略规划，不伪造深度健康状态。

使用方式：

1. 在 RViz 的 **MapOdomModify** 面板调整 `map→odom`，使机体显示位置与先验地图对应。该调整没有人工确认门控。
2. 使用 **2D Goal Pose** 在地图上指定目标。
3. 查看路径和 `/navigation_state`。目标从 `map` 转换到 `odom`，默认目标高度为 2 m，可在目标管理参数中修改。

`flight_state` 在手动 OFFBOARD 下显示 `HOLD`（保持）或 `CRUISE`（执行 EGO）；其他飞行模式显示 `IDLE`。只有显式调用 `/takeoff` 才执行 `PRESTREAM → ARMING → TAKEOFF`。生成路径不会自动解锁或起飞。

### 自动航点决策

`startup.launch.py` 已包含 `decision_manager`。在 `src/decision/decision_manager/config/decision.yaml` 中填写 `decision_1.waypoints`，默认列表为空，不会向未经配置的场地坐标飞行。坐标格式为 `[x, y, z]`，单位 m；`frame_id` 支持 `map` 和 `odom`。

- `mode: once`：依次执行所有航点，在最后一个点结束任务并保持位置。
- `mode: loop`：最后一个点到达后回到第一个点，持续循环。
- `decision_manager.ros__parameters.initial_decision`：当前固定选择的行为，默认为 `decision_1`。
- `enabled: false`：禁用自动任务，仅使用 RViz 手动打点。

进入 PX4 OFFBOARD 时选择任务并提交首个航点，目标管理器在里程计、TF 和规划器就绪后下发给 EGO。decision 不等待 `CRUISE`、深度或 VIO 健康字符串；到点推进依据新鲜里程计。重复 OFFBOARD 状态消息不会重新开始任务；退出后再次进入 OFFBOARD 则从首点开始。decision 不主动解锁或起飞。

每点满足水平距离、高度、低速及持续时间条件后才切换，第一版是**逐点停稳巡航**，不支持高速途经点衔接。决策航点保留各自高度；RViz 二维打点仍采用 `goal_altitude`。导航接收确认使用转换后的 `odom` 目标，到点检查使用新鲜的 `/ego/odom`。

里程计过期时暂停到点推进，恢复后继续当前航点。VIO 重置会清除当前任务，不会自动重发旧坐标目标；需退出并重新进入 OFFBOARD 才开始新一轮。RViz 手动打点接管后，当前自动任务取消，本次 OFFBOARD 内不再自动抢回目标。自动任务不会自行降落。

查看当前行为、航点索引（从 0 开始）、已完成圈数及等待原因：

```bash
ros2 topic echo /decision/state --qos-durability transient_local
```

后续行为选择逻辑可扩展 `src/decision/decision_manager/src/decision_node.py` 中的 `select_decision()`；任务航点及到点阈值均在 `decision.yaml` 中修改。

任务结束时降落：

```bash
ros2 service call /land std_srvs/srv/Trigger '{}'
```

停止运行时，在各启动终端按 `Ctrl+C`。

## 7. 常用检查与故障定位

| 话题 | 用途 |
| --- | --- |
| `/vio_health`、`/vio_diagnostics` | 定位是否可用及具体原因 |
| `/odom`、`/ego/odom` | 机体定位与规划器里程计 |
| `/ego/depth_fresh` | 深度输入是否新鲜 |
| `/ego/occupancy_inflate` | 深度生成的膨胀占用点云，坐标系为 `odom` |
| `/ego/planning/bspline`、`/ego/position_cmd` | 规划轨迹与轨迹执行指令 |
| `/navigation_state` | 目标是否已发送给规划器 |
| `/flight_state`、`/flight_hold_reason` | 飞行状态及悬停原因 |

常用命令：

```bash
ros2 topic echo --once /vio_health
ros2 topic echo --once /vio_diagnostics
ros2 topic hz /ego/odom
ros2 topic echo --once /ego/depth_fresh
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo --once /flight_hold_reason --qos-durability transient_local
```

- **打点后无路径**：检查 TF、`/ego/odom`、目标高度与地图范围；无需先起飞、启用深度或进入 `CRUISE`，但必须具备有效里程计、TF 和运行中的规划器。
- **TF 停止更新或定位漂移**：查看 `/vio_diagnostics`，检查双目和 IMU 的帧率、源时间戳、标定及安装外参。不要把停留的 TF 当作当前真实位置。
- **进入 HOLD**：OFFBOARD 中没有可用 EGO 指令时保持位置；检查轨迹与里程计是否更新，恢复后自动继续，无需 `/resume_navigation`。VIO 坐标重置会清除地图、轨迹和旧目标，需要重新打点。
- **难以复现的定位或飞控异常**：同步保存左右目、IMU、`/uav1/odomimu`、`/vio_diagnostics`、PX4 状态和本地位置的 rosbag，以及飞控 ULog，用于区分传感器、定位与控制问题。

本项目使用前视感知，未知区域允许通行；没有独立定位备份，VIO 完全失效时无法保证继续自主定位。真机精度与可用速度需要通过实际测试确认。

## 8. 补充资料

- [控制模块说明](docs/control_architecture.md)
- [当前真机标定说明](deploy/calibration/uav1/README.md)
- 上游来源及许可证：各算法目录中的 `UPSTREAM.md`、`LICENSE`。
