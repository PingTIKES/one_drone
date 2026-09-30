# one_drone

`one_drone` 是一台无人机的三维局部自主导航工程，运行环境为 **Ubuntu 22.04、ROS 2 Humble、OpenVINS、EGO-Planner / EGO-Swarm 单机模式和 MicoAir PX4 1.14.3**。仿真与真机使用同一套算法节点；Gazebo 只提供环境、相机、IMU 和飞行动力学，算法不读取仿真真值位姿。

当前仓库只保留单机定位、三维局部建图、轨迹规划和 PX4 控制。Nav2、PGM 地图服务器、2D 代价地图、行为树、`cmd_vel` 控制链、`map→odom` 人工调整及相关功能包已经移除。多机协同、目标识别和任务决策暂不在本阶段范围内。

> 当前状态：所有 ROS 2 功能包和总启动文件已在 Ubuntu 22.04 / Humble 上完成干净编译，并在 PX4 1.14.3 SITL 与 3 m 墙体识别柱场景中完成起飞、EGO 规划、轨迹执行和降落闭环验证。2 m 前向目标的最终 VIO 水平位置误差约 7 cm。真机尚未试飞验证，因此默认速度限制为 0.5 m/s，真机验证稳定后再逐步提高。

## 算法链路

```text
左右红外灰度图 + IMU
  └─ OpenVINS
      ├─ /odom + TF odom→base_link
      ├─ /vio_health
      └─ PX4 vehicle_visual_odometry

/odom
  └─ ego_odom_adapter（机体系速度转 odom 世界系速度）
      └─ /ego/odom

双目深度图 + CameraInfo + T_body_depth + /ego/odom
  └─ EGO GridMap（三维占用栅格和膨胀）
      └─ B-spline 局部规划器
          └─ /ego/planning/bspline
              └─ traj_server
                  └─ /ego/position_cmd
                      └─ flight_bridge
                          └─ PX4 Offboard 速度和偏航角速度设定值

RViz 2D Goal Pose
  └─ /navigation_goal
      └─ goal_manager（VIO、飞行状态、深度门控）
          └─ /ego/goal
```

全链使用 `odom` 作为规划世界坐标。OpenVINS 启动位置就是 `(0,0,0)` 附近，起飞后 RViz 中的 `odom→base_link` 决定飞机位置；无需 PGM、`map` 坐标或人工对齐。EGO 的局部地图只由实际深度观测生成，未知区域不直接写成障碍。地图采用三维体素，能够在后续阶段扩展升降绕障。

`flight_bridge` 执行

```text
v_cmd = v_ego + Kp × (p_ego - p_actual)
```

并把 odom 世界系速度转换到机体系和 PX4 NED。EGO 的 XYZ B-spline 与 yaw 已解耦，`flight_bridge` 单独执行感知航向策略：目标方向在机头 ±30° 内保持当前 yaw 并允许横移；30°–100° 边平移边缓慢转向；100°–150° 保留 25% 平移速度并继续对齐；只有接近正后方、超过 150° 时才停止平移，避免前视相机不可见区域内的后退盲飞。最大偏航角速度默认为 0.35 rad/s。到点时，水平位置误差不超过 0.15 m 且 EGO 水平速度不超过 0.10 m/s 会进入 `ARRIVAL_HOLD`，水平速度和 yaw 角速度归零；退出阈值为进入阈值的 1.5 倍，避免 VIO 微小抖动反复触发旋转。

## 目录与功能包

```text
src/
├── bringup/
│   ├── launch/startup.launch.py     # 唯一算法启动入口
│   ├── params/ego_params.yaml       # EGO GridMap、规划和优化参数
│   └── rviz/ego_navigation.rviz     # odom、TF、三维占用和轨迹显示
├── perception/
│   ├── camera_stream/               # 真机图像与 IMU 话题转发
│   ├── stereo_depth/                # 仿真/软件双目深度
│   └── obstacle_cloud/              # 调试用深度点云与自体掩膜
├── localization/
│   └── vio_bridge/                  # OpenVINS 健康检查、/odom、TF、PX4 外部视觉
├── navigation/
│   ├── ego_bridge/                  # /odom 与深度健康适配
│   ├── ego_planner/                 # EGO-Swarm ROS 2 单机规划核心
│   └── goal_manager/                # RViz 目标门控与 /ego/goal 发布
└── control/
    └── flight_bridge/               # PositionCommand 到 PX4 Offboard
```

EGO 核心来自 `ZJU-FAST-Lab/ego-planner-swarm` 的 `ros2_version` 分支，固定来源提交写在 `src/navigation/ego_planner/UPSTREAM.md`，许可证保留在同目录 `LICENSE`。仓库只引入单机运行必需的 `plan_env`、`path_searching`、`bspline_opt`、`traj_utils`、`ego_planner` 和 `quadrotor_msgs`。

| 功能包 | 主要输入 | 主要输出 | 参数位置 |
| --- | --- | --- | --- |
| `camera_stream` | 真机驱动左右目、IMU | `/uav1/cam0/image_raw`、`cam1`、`imu0` | 包内 `config/params.yaml` |
| `stereo_depth` | 左右红外灰度图、双目标定 | `/uav1/d435i/depth/image_raw`、CameraInfo | 包内 `config/params.yaml` |
| `obstacle_cloud` | 深度、CameraInfo、相机外参 | `/uav1/obstacles`，仅调试显示 | 包内 `config/params.yaml` |
| `vio_bridge` | `/uav1/odomimu`、双目时间戳 | `/odom`、`odom→base_link`、`/vio_health`、PX4 外部视觉 | 包内 `config/params.yaml` |
| `ego_bridge` | `/odom`、深度图 | `/ego/odom`、`/ego/depth_fresh` | 包内 `config/params.yaml` |
| `ego_planner` | `/ego/odom`、深度、CameraInfo、`/ego/goal` | 三维占用、B-spline、可视化 Marker | `bringup/params/ego_params.yaml` |
| `goal_manager` | `/navigation_goal`、VIO、深度、飞行状态 | `/ego/goal`、`/navigation_state` | 包内 `config/params.yaml` |
| `flight_bridge` | `/ego/position_cmd`、`/ego/odom`、PX4 状态 | PX4 Offboard 设定值、飞行状态与安全状态 | 包内 `config/params.yaml` |

## 关键参数

所有常用参数均在 YAML 中修改，无需改源码。

`src/bringup/params/ego_params.yaml` 的初始 RK3566 配置：

- 体素分辨率 `grid_map/resolution: 0.15`
- 地图尺寸 `32 × 36 × 3 m`
- 局部更新范围 `4.5 × 4.5 × 2 m`
- 障碍膨胀 `0.30 m`
- 深度范围 `0.30–5.0 m`，最大射线 `4.5 m`
- 深度降采样 `skip_pixel: 4`
- 最大规划速度 `0.5 m/s`，最大加速度 `1.0 m/s²`
- 规划视距 `4.5 m`
- 单机模式 `drone_id: 0`，移动目标预测数量 `0`

速度上限必须同时修改以下位置：

1. `bringup/params/ego_params.yaml` 中 `manager/max_vel`、`optimization/max_vel` 和 `bspline/limit_vel`；
2. `control/flight_bridge/config/params.yaml` 中 `max_horizontal_speed`。

先完成 0.5 m/s 的避障和 VIO 稳定验证，再提高到 1.0 m/s。达到 3 m/s 前必须实测深度有效距离、端到端延迟、制动距离、转弯半径和 RK3566 规划耗时；不能只修改速度数值。

GridMap 不再使用上游硬编码相机安装关系。`startup.launch.py` 从仿真模型或真机 `body.yaml` 取得 `T_body_depth`，并传入 `grid_map/cam2body`；内参由对应深度 `CameraInfo` 动态更新。深度与里程计采用近似时间同步。

## 首次安装与构建

```bash
git clone https://github.com/PingTIKES/one_drone.git ~/one_drone
cd ~/one_drone
bash setup_env.sh sim       # 仿真机
# 或 bash setup_env.sh onboard
```

脚本固定并校验 MicoAir PX4 1.14.3、`px4_msgs` release/1.14 和 OpenVINS 版本，安装 PCL、Eigen、cv_bridge 和 CycloneDDS，然后构建工作区。已有仓库不要再次 `git clone`。若 `apt` 被 `packagekitd` 占锁，等系统更新完成后重试，不要删除锁文件。

日常修改 YAML、launch 或 Python 后执行：

```bash
cd ~/one_drone
colcon build --symlink-install
```

## 仿真流程

### 1. 启动环境和 PX4 1.14.3

终端 A：

```bash
cd ~/one_drone
bash scripts/start_algorithm_sim.sh
```

等待输出 `[sim] Ready.`。无图形环境可在命令前加 `HEADLESS=1`。脚本使用 3 m 墙体且带彩色识别柱的场景，并生成 `/tmp/one_drone_gz_env.sh`。

### 2. 一键启动全部算法

终端 B：

```bash
cd ~/one_drone
source /opt/ros/humble/setup.bash
source ~/catkin_ws_ov/install/setup.bash
source ~/one_drone/install/setup.bash
source /tmp/one_drone_gz_env.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
PYTHONNOUSERSITE=1 ros2 launch bringup startup.launch.py sim:=true rviz:=true
```

不再启动第二套导航 launch。`startup.launch.py` 已包含传感器桥、OpenVINS、深度、EGO、轨迹服务器、目标管理、PX4 控制和 RViz。

### 3. 起飞前检查

```bash
ros2 topic hz /uav1/cam0/image_raw
ros2 topic hz /uav1/cam1/image_raw
ros2 topic hz /uav1/imu0
ros2 topic hz /uav1/d435i/depth/image_raw
ros2 topic hz /ego/odom
ros2 topic echo --once /vio_health
ros2 topic echo --once /ego/depth_fresh
ros2 topic hz /ego/occupancy_inflate
ros2 run tf2_ros tf2_echo odom base_link
```

必须满足：起飞前 `/vio_health` 为 `VALID`、`/ego/odom` 连续、深度为 `true`、三维膨胀占用持续发布、TF 方向与 Gazebo 中的移动一致。飞行中短时数据缺口会显示 `DEGRADED`，控制器自动停止平移并保持当前位置；数据恢复且运动连续时自动回到 `VALID`。持续超过 2 秒的断流或真实位姿跳变会显示 `INVALID` 并进入 `HOLD`；VIO 连续稳定 1 秒，且 PX4 仍已解锁、处于 OFFBOARD、位置有效、没有 failsafe 后，系统自动回到原飞行阶段，并从恢复后的新位置重新规划保存的目标。当前软件双目实测约 3.5 Hz，因此深度心跳超时为 0.5 秒；若实际频率低于 2.5 Hz，应先解决算力或图像同步问题。RViz Fixed Frame 默认为 `odom`，白色背景，TF Marker Scale 为 2.5。

### 4. 起飞、打点和降落

```bash
ros2 service call /takeoff std_srvs/srv/Trigger '{}'
ros2 topic echo /flight_state
```

`flight_state` 依次经过 `PRESTREAM → ARMING → TAKEOFF → CRUISE`。进入 `CRUISE` 后使用 RViz 的 **2D Goal Pose** 打点。工具发布 `/navigation_goal`，`goal_manager` 将高度设为默认 2 m 后发布 `/ego/goal`。观察：

```bash
ros2 topic echo /navigation_state
ros2 topic hz /ego/planning/bspline
ros2 topic hz /ego/position_cmd
ros2 topic echo /flight_safety_status --qos-durability transient_local
```

任务结束主动降落：

```bash
ros2 service call /land std_srvs/srv/Trigger '{}'
```

## 真机流程

目标飞控为 **MicoAir743v2-AIO-35A，PX4 1.14.3**。当前只有 D435i 和飞控，没有独立定位备份，因此 VIO 完全失效后无法继续自主定位。

### 1. 机械安装和标定

- D435i 刚性安装在机头正前方，左右红外和深度视野不得被桨叶、保护架或线束遮挡。
- 标定左右红外内参、双目外参、相机与 IMU 的时间偏差和外参。
- 测量 `T_body_imu` 与 `T_body_depth`，定义为 **body FLU ← sensor** 的 4×4 变换。
- 采集静止、平移和多方向转动数据，检查重投影误差、尺度、时间戳单调性和 IMU 噪声参数。

导入 Kalibr 结果：

```bash
python3 tools/import_kalibr.py \
  --camchain /path/to/camchain-imucam.yaml \
  --imu /path/to/imu.yaml \
  --body /path/to/body.yaml \
  --output ~/one_drone_calibration
```

改变分辨率、镜头相对位置或相机设备后必须重标定。仿真标定不能用于真机。

### 2. PX4 和传感器检查

- 在 QGroundControl 中确认机型、飞控朝向、传感器标定、遥控接管、急停、电池和失控保护。
- 配置 PX4 EKF2 融合外部视觉，确认实际 `MAV_SYS_ID` 和 uXRCE DDS 名称空间。
- 启动 D435i 左右红外、深度、陀螺仪与加速度计，确认组合 IMU 与图像使用同一时基。
- 无桨状态下验证 PX4 能收到 `vehicle_visual_odometry`，移动机体时 `/ego/odom`、RViz 与实物方向一致。

### 3. 启动真机算法

```bash
cd ~/one_drone
source /opt/ros/humble/setup.bash
source ~/catkin_ws_ov/install/setup.bash
source ~/one_drone/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
PYTHONNOUSERSITE=1 ros2 launch bringup startup.launch.py \
  sim:=false rviz:=true depth_source:=hardware \
  calibration_dir:=/home/ubuntu22/one_drone_calibration \
  target_system:=1 px4_ns:=/ \
  cam0_topic:=/actual/right/infrared \
  cam1_topic:=/actual/left/infrared \
  imu_topic:=/actual/synchronized/imu \
  depth_topic:=/actual/depth/image_rect_raw \
  depth_info_topic:=/actual/depth/camera_info
```

示例话题和 `target_system` 必须替换为当前设备实际值。真机深度进入 EGO GridMap，OpenVINS 仍使用左右红外灰度图和 IMU。先完成无桨台架和小范围 0.5 m/s 试飞，再逐级放开速度。

## 安全状态与故障排查

`flight_bridge` 保留显式 `/takeoff`、`/land` 和 `/resume_navigation` 服务。深度暂时中断或 VIO 为 `DEGRADED` 时停止执行轨迹并用 PX4 本地位置悬停，数据恢复后自动继续；VIO 为 `INVALID` 或 PX4 本地位置失效时进入 `HOLD`，满足上述稳定条件后自动退出。`goal_manager` 会保存最后一个合法目标以及恢复期间新打的目标，并在 `CRUISE` 恢复时要求 EGO 从新位姿重新规划。`/resume_navigation` 保留为自动恢复条件长期不满足时的人工备用入口。本节点不会主动发送故障降落命令，PX4 自身 estimator failsafe 仍然具有最终控制权。

```bash
ros2 topic echo --once /flight_hold_reason std_msgs/msg/String --qos-durability transient_local
ros2 topic echo --once /flight_safety_status std_msgs/msg/String --qos-durability transient_local
ros2 topic echo --once /vio_diagnostics
ros2 topic echo --once /navigation_state --qos-durability transient_local
```

`[init]: not enough feats` 表示 OpenVINS 初始化窗口缺少持续特征；若之后 VIO 变为 `VALID`，短暂出现可以接受。持续出现 `No IMU measurements to propagate with` 或 ZUPT 无 IMU，需检查左右目约 30 Hz、IMU 约 200 Hz，以及所有 `header.stamp` 是否处于同一个仿真或系统时钟。不要通过关闭 ZUPT 掩盖同步问题。

EGO 无轨迹时先检查：

1. `/ego/odom` 是否有频率且 `frame_id=odom`；
2. 深度与 CameraInfo 是否发布，QoS 是否匹配；
3. `/ego/occupancy_inflate` 是否有数据；
4. 飞行状态是否为 `CRUISE`；
5. `/navigation_state` 是否为 `GOAL_SENT_TO_EGO`；
6. 目标是否位于当前 32 × 36 × 3 m 地图边界内。

VIO 跳变或自动进入 HOLD 时，录制：

```bash
ros2 bag record -o flight_bags/vio_fault_$(date +%Y%m%d_%H%M%S) \
  /clock /uav1/cam0/image_raw /uav1/cam1/image_raw /uav1/imu0 \
  /uav1/odomimu /vio_health /vio_diagnostics /ego/odom \
  /ego/position_cmd /px4_1/fmu/out/vehicle_local_position \
  /px4_1/fmu/out/vehicle_status
```

真机将 `/px4_1` 改为实际 PX4 名称空间，并同时保存 `.ulg`。把 rosbag 目录与对应 ULog 一并提供，才能区分图像/IMU 时间缺口、OpenVINS 跳变、PX4 EKF reset 或控制链超时。

## 已知边界

- 当前是单机 EGO 模式，不做 EGO-Swarm 多机轨迹广播与碰撞协调。
- 当前目标高度固定为 2 m，EGO 内部已是三维规划，但 RViz 2D Goal Pose 不提供目标高度；可在 `goal_manager/config/params.yaml` 修改。
- 只有前视深度。目标落在后方时会先以低速转入视场，无法感知的后方区域不会被假定为安全。
- 无独立定位备份时，OpenVINS 完全失效不能保证继续自主飞行。
- 仿真通过只证明软件链与当前模型兼容，不能替代真机标定、台架测试和受控场地试飞。
