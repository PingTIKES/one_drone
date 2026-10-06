# one_drone

`one_drone` 是一台无人机的三维局部自主导航工程，运行环境为 **Ubuntu 22.04、ROS 2 Humble、OpenVINS（默认）/ ORB-SLAM3（保留对照测试）、EGO-Planner / EGO-Swarm 单机模式和 MicoAir PX4 1.14.3**。仿真与真机使用同一套算法节点；Gazebo 只提供环境、相机、IMU 和飞行动力学，算法不读取仿真真值位姿。

当前仓库只保留单机定位、先验 PGM 地图显示、三维局部建图、轨迹规划和 PX4 控制。先验地图及手动 `map→odom` 调整用于 RViz 定位和打点；EGO 的避障仍只使用深度观测。Nav2 规划器、2D 代价地图、行为树和 `cmd_vel` 控制链已移除。多机协同、目标识别和任务决策暂不在本阶段范围内。

> 当前状态：所有 ROS 2 功能包和总启动文件已在 Ubuntu 22.04 / Humble 上完成干净编译，并在 PX4 1.14.3 SITL 与 3 m 墙体识别柱场景中完成起飞、EGO 规划和轨迹执行闭环验证。2 m 前向目标的最终 VIO 水平位置误差约 7 cm；暂停深度处理 3 秒的故障注入中，飞机停止轨迹，深度恢复后从当前位置重新规划并到达目标。真机尚未试飞验证，因此默认速度限制为 0.5 m/s，真机验证稳定后再逐步提高。

## 算法链路

```text
左右红外灰度图 + IMU
  └─ OpenVINS 双目惯性定位
      ├─ /odom + TF odom→base_link
      ├─ /vio_health
      └─ PX4 vehicle_visual_odometry

/odom + /vio_health + /vio_diagnostics
  └─ ego_odom_adapter（机体系速度转 odom 世界系速度）
      └─ /ego/odom

双目深度图
  └─ depth_filter（空间去噪、时间滤波、小空洞填补）
      └─ CameraInfo + 相机安装外参 + /ego/odom
          └─ EGO GridMap（三维占用栅格和膨胀）
      └─ B-spline 局部规划器
          └─ /ego/planning/bspline
              └─ traj_server
                  └─ /ego/position_cmd
                      └─ trajectory_validator → trajectory_controller
                          └─ yaw_manager → flight_supervisor
                              └─ FlightSetpoint → px4_adapter
                                  └─ PX4 Offboard 速度和偏航角速度设定值

RViz 2D Goal Pose
  └─ /navigation_goal（map）
      └─ goal_manager（按当前 map→odom 转换目标；检查 VIO、飞行状态和深度）
          └─ /ego/goal（odom）
```

TF 链为 `map→odom→base_link`：modify 节点按配置和 RViz 面板调整发布 `map→odom`，定位桥根据 OpenVINS 输出发布 `odom→base_link`。先验 PGM 在 `map` 下显示，RViz 以 `map` 为 Fixed Frame，打点在 `map` 下；`goal_manager` 把目标变换到 `odom`，EGO 仍在 `odom` 下规划与控制。手动对齐不会触发起飞门控，但打点位置是否正确取决于当前 `map→odom`。EGO 的局部地图只由实际深度观测生成，未知区域不直接写成障碍；PGM **不参与** EGO 碰撞检测。

`trajectory_controller` 执行

```text
v_cmd = v_ego + Kp × (p_ego - p_actual)
```

`yaw_manager` 根据当前速度和未来 0.3 秒的轨迹加速度调整航向；运动方向超出相机约 87° 的水平视角时，将平移速度比例限制到 25%，并继续缓慢转向。原有 ±30° 航向保持、100° 大角度限速、150° 后方停止平移及到点 `ARRIVAL_HOLD` 仍保留。`trajectory_validator` 拒绝帧、时间戳、非有限数值、速度、加速度或跟踪误差异常的样本。`flight_supervisor` 决定起飞、HOLD 与恢复；`px4_adapter` 是唯一 PX4 指令发布者，负责转换到 PX4 NED 和发布 `FlightSetpoint` 审计话题。以上为同一进程内的模块，避免多个节点同时争夺 Offboard。当前 yaw 已使用相机 FOV、轨迹前瞻和 VIO 健康状态；尚未建立“已观测区域”的方向模型或 FUEL Frontier 层，不能把这版称作完整 FUEL 主动感知。

## 目录与功能包

```text
src/
├── bringup/
│   ├── launch/startup.launch.py     # 唯一算法启动入口
│   ├── params/ego_params.yaml       # EGO GridMap、规划和优化参数
│   ├── params/global_config.yaml   # 选择要加载的先验 PGM 地图 YAML
│   ├── map/                        # PGM 与配套 YAML
│   └── rviz/ego_navigation.rviz     # 先验地图、TF、三维占用和轨迹显示
├── perception/
│   ├── camera_stream/               # 真机图像与 IMU 话题转发
│   ├── stereo_depth/                # 仿真/软件双目深度
│   ├── depth_filter/                # 深度时空滤波与小空洞填补
│   └── obstacle_cloud/              # 调试用深度点云与自体掩膜
├── localization/
│   ├── orb_slam3/                   # 内置 ORB-SLAM3、ROS 2 双目惯性接口与配置
│   ├── open_vins/                   # 内置 OpenVINS：ov_core、ov_init、ov_msckf、ov_eval
│   └── vio_bridge/                  # 定位健康检查、/odom、TF、PX4 外部视觉
├── navigation/
│   ├── ego_bridge/                  # /odom 与深度健康适配
│   ├── ego_planner/                 # EGO-Swarm ROS 2 单机规划核心
│   └── goal_manager/                # RViz 目标门控与 /ego/goal 发布
├── control/
│   ├── flight_interfaces/           # FlightSetpoint、SystemStatus 消息
│   └── flight_bridge/               # 控制、安全与 PX4 适配模块，单一 Offboard 发布者
└── rviz/
    ├── modify_map_to_odom/          # 可调 map→odom TF 发布
    └── rviz_tf_shift/               # RViz 手动平移/旋转面板
```

EGO 核心来自 `ZJU-FAST-Lab/ego-planner-swarm` 的 `ros2_version` 分支，固定来源提交写在 `src/navigation/ego_planner/UPSTREAM.md`，许可证保留在同目录 `LICENSE`。仓库只引入单机运行必需的 `plan_env`、`path_searching`、`bspline_opt`、`traj_utils`、`ego_planner` 和 `quadrotor_msgs`。

| 功能包 | 主要输入 | 主要输出 | 参数位置 |
| --- | --- | --- | --- |
| `camera_stream` | 真机驱动左右目、IMU | `/uav1/cam0/image_raw`、`cam1`、`imu0` | 包内 `config/params.yaml` |
| `stereo_depth` | 左右红外灰度图、双目标定 | `/uav1/d435i/depth/image_raw`、CameraInfo | 包内 `config/params.yaml` |
| `depth_filter` | 软件或真机原始深度 | `/uav1/d435i/depth/image_filtered` | 包内 `config/params.yaml` |
| `obstacle_cloud` | 滤波深度、CameraInfo、相机外参 | `/uav1/obstacles`，仅调试显示 | 包内 `config/params.yaml` |
| `orb_slam3` | 已标定的左右目与相机 IMU | `/uav1/odomimu`、`/orbslam/tracking_state`、`/estimator_reset` | `orb_slam3/config/params.yaml`；启动时转换当前标定 |
| `ov_core` / `ov_init` | 图像特征、IMU 和标定（内部库接口） | 跟踪结果及初始化状态 | 由 `ov_msckf` 使用同一估计器配置 |
| `ov_msckf` | `/uav1/cam0/image_raw`、`cam1`、`imu0` | `/uav1/odomimu` 等 VIO 输出 | `vio_bridge/config/openvins_sim` 或真机标定目录 |
| `ov_eval` | 离线轨迹及计时文件 | 误差统计和评估图表；默认启动不运行 | 各评估命令参数 |
| `vio_bridge` | `/uav1/odomimu`、双目图像、IMU | `/odom`、`odom→base_link`、`/vio_health`、`/vio_diagnostics`、PX4 外部视觉 | 包内 `config/params.yaml` |
| `ego_bridge` | `/odom`、深度图 | `/ego/odom`、`/ego/depth_fresh` | 包内 `config/params.yaml` |
| `ego_planner` | `/ego/odom`、深度、CameraInfo、`/ego/goal` | 三维占用、B-spline、可视化 Marker | `bringup/params/ego_params.yaml` |
| `modify_map_to_odom` | 配置中的 X/Y/Rotation、RViz 面板调整 | TF `map→odom` | 包内 `config/config.yaml` |
| `nav2_map_server` | `bringup/map` 中选定的 YAML/PGM | `/map`，只供显示与打点 | `bringup/params/global_config.yaml` |
| `goal_manager` | `map` 或 `odom` 下的 `/navigation_goal`、TF、VIO、深度、飞行状态 | `odom` 下的 `/ego/goal`、`/navigation_state` | 包内 `config/params.yaml` |
| `flight_interfaces` | 控制链内部命令、飞行与传感器状态 | `FlightSetpoint`、`SystemStatus` ROS 2 消息定义 | 无运行参数 |
| `flight_bridge` | `/ego/position_cmd`、`/ego/odom`、PX4 状态 | PX4 Offboard 设定值、`/flight_setpoint`、`/system_status` 及原有状态话题 | 包内 `config/params.yaml` |

## 关键参数

所有常用参数均在 YAML 中修改，无需改源码。

`src/bringup/params/ego_params.yaml` 的初始 RK3566 配置：

- 体素分辨率 `grid_map/resolution: 0.15`
- 地图尺寸 `32 × 36 × 3 m`
- 局部更新范围 `4.5 × 4.5 × 2 m`
- 整机碰撞半径 `0.10 m`（包含桨叶最外缘，直径 `0.20 m`），额外安全余量 `0.10 m`，总障碍膨胀参数 `0.20 m`（当前 0.15 m 体素取整为 2 格）
- 深度范围 `0.30–5.0 m`，最大射线 `4.5 m`
- 深度降采样 `skip_pixel: 4`
- 最大规划速度 `0.5 m/s`，最大加速度 `1.0 m/s²`
- 规划视距 `4.5 m`
- 单机模式 `drone_id: 0`，移动目标预测数量 `0`

速度上限必须同时修改以下位置：

1. `bringup/params/ego_params.yaml` 中 `manager/max_vel`、`optimization/max_vel` 和 `bspline/limit_vel`；
2. `control/flight_bridge/config/params.yaml` 中 `max_horizontal_speed`。

先完成 0.5 m/s 的避障和 VIO 稳定验证，再提高到 1.0 m/s。达到 3 m/s 前必须实测深度有效距离、端到端延迟、制动距离、转弯半径和 RK3566 规划耗时；不能只修改速度数值。

GridMap 不再使用上游硬编码相机安装关系。软件双目模式从双目标定和 `T_body_imu` 推导深度相机外参；当前 D435i 的硬件深度模式从 `T_body_imu`、相机–IMU 标定推导该外参（深度光学坐标系与左红外光学坐标系重合），也可由 `body.yaml` 中显式 `T_body_depth` 覆盖。启动文件将外参传入 `grid_map/cam2body`；深度内参由对应的 `CameraInfo` 动态更新。深度与里程计采用近似时间同步。

第一阶段 VIO 可靠性增强不修改 OpenVINS 核心。`vio_bridge` 从相同的左右目输入低频统计可跟踪角点，同时检查图像、IMU、里程计新鲜度，位置/姿态/速度协方差、速度变化率和角速度。`/vio_diagnostics` 以 JSON 给出 `quality`、`confidence`、`feature_count`、协方差、数据年龄与降级原因。状态处理为：

- `VALID/GOOD`：正常速度和 yaw 策略；
- `DEGRADED`：轨迹仍有效时把最大水平速度限制为正常值的 40%，yaw 限制为 0.10 rad/s；里程计或深度已经陈旧时改为位置悬停；
- `INVALID/BAD`：停止轨迹执行并进入可恢复 `HOLD`。

起飞与 `INVALID` 后恢复要求 VIO 连续稳定 3 秒。特征阈值、协方差门限、软降级速度和 yaw 均可分别在 `vio_bridge/config/params.yaml` 与 `flight_bridge/config/params.yaml` 调整。

## 首次安装与构建

```bash
git clone https://github.com/PingTIKES/one_drone.git ~/one_drone
cd ~/one_drone
bash setup_env.sh sim       # 仿真机
# 或 bash setup_env.sh onboard
```

OpenVINS 固定版本源码直接随仓库提供，来源及补丁见 `src/localization/open_vins/UPSTREAM.md`。脚本固定并校验外部 MicoAir PX4 1.14.3 和 `px4_msgs` release/1.14，安装 Ceres、OpenCV、Boost、PCL、Eigen、cv_bridge 和 CycloneDDS，然后在本项目一次构建全部功能包。仿真 PX4 仍放在外部 `~/PX4-Autopilot-1.14.3`（可用 `PX4_DIR` 指定）；onboard 模式不下载或编译 PX4。已有仓库不要再次 `git clone`。若 `apt` 被 `packagekitd` 占锁，等系统更新完成后重试，不要删除锁文件。

日常修改 YAML、launch 或 Python 后执行：

```bash
cd ~/one_drone
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
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
source ~/one_drone/install/setup.bash
source /tmp/one_drone_gz_env.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch bringup startup.launch.py
```

运行上面的仿真算法命令前，将 `src/bringup/params/launch.yaml` 的 `startup.sim` 设为 `true`；启动文件自动读取 `simulation` 内的软件深度、飞控编号和 DDS 命名空间配置。该文件当前默认 `startup.sim: false`，用于已标定的真机。`startup.rviz` 控制 RViz 开关，日常不需要在命令后追加参数。

不再启动第二套导航 launch。`startup.launch.py` 已包含传感器桥、OpenVINS、先验地图服务器、modify、深度、EGO、轨迹服务器、目标管理、PX4 控制和 RViz。默认地图由 `src/bringup/params/global_config.yaml` 的 `map:` 指定，可填 `src/bringup/map` 内的其他 YAML 文件名；也可在 `launch.yaml` 的 `startup.map_file` 中填写地图文件名或绝对 YAML 路径。改完参数或地图后运行 `colcon build --symlink-install` 并重启 launch。

RViz 左侧 **MapOdomModify** 面板可手动平移/旋转 `map→odom`，初值在 `src/rviz/modify_map_to_odom/config/config.yaml`。先观察 `base_link` 在先验地图上的位置，再调整面板。该调整不更改 OpenVINS 的 `odom→base_link` 或 EGO 局部占用，也不需要人工确认才能起飞。

`ego_planner_node` 与其他算法节点进程隔离；异常退出后由 launch 在 0.1 秒后自动拉起。`/ego/map_heartbeat` 同时提供规划器实例编号和最近成功融合的深度时间戳；轨迹服务器与控制桥独立监控。心跳断流超过 0.75 个真实秒、地图超过 0.75 个 ROS 秒未更新，或检测到实例更换，都会停用旧轨迹，控制桥进入 HOLD。地图与定位恢复后，保留的目标从当前位置重新规划；VIO 坐标重置则仍丢弃旧目标。0.1 秒是重启等待，不代表检测、初始化和建图总耗时。到点距离小于 0.2 m 时直接等待目标，不再无限生成短轨迹。

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
ros2 topic echo --once /map --qos-durability transient_local
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

必须满足：起飞前 `/vio_health` 为 `VALID`、`/ego/odom` 连续、深度为 `true`、三维膨胀占用持续发布、TF 方向与 Gazebo 中的移动一致。飞行中低置信度会显示 `DEGRADED`，输入仍新鲜时控制器限速继续；里程计或深度陈旧时改为位置悬停。持续断流或真实位姿跳变会显示 `INVALID` 并进入 `HOLD`；VIO 连续稳定 3 秒，且 PX4 仍已解锁、处于 OFFBOARD、位置有效、没有 failsafe 后，未发生坐标重置时，系统自动回到原飞行阶段，并从恢复后的新位置重新规划保存的目标；发生 VIO 重置时会清图、停旧轨迹、丢弃旧目标，等待地图重建后恢复，必须重新打点。当前过滤后的软件双目实测约 2.5–3 Hz、最大调度间隔接近 1 秒，因此控制层深度心跳超时为 1.2 秒，新增地图执行检查在最后成功融合的深度超过 0.75 个 ROS 秒时先停止执行，EGO 内部原有 1.5 秒深度超时仍保留；同步恢复后清除超时并从当前位置重新规划。若平均频率低于 2.5 Hz，应先解决算力或图像同步问题。RViz Fixed Frame 默认为 `map`，白色背景，TF Marker Scale 为 2.5。

### 4. 起飞、打点和降落

```bash
ros2 service call /takeoff std_srvs/srv/Trigger '{}'
ros2 topic echo /flight_state
```

`flight_state` 依次经过 `PRESTREAM → ARMING → TAKEOFF → CRUISE`。进入 `CRUISE` 后使用 RViz 的 **2D Goal Pose** 在先验地图上打点。工具以 `map` 坐标发布 `/navigation_goal`，`goal_manager` 读取当前 TF、转换为 `odom` 目标，并将高度设为默认 2 m 后发布 `/ego/goal`。如果 TF 尚未出现，目标暂存并在 TF 可用后发送。观察：

```bash
ros2 topic echo /navigation_state
ros2 topic hz /ego/planning/bspline
ros2 topic hz /ego/position_cmd
ros2 topic echo /flight_safety_status --qos-durability transient_local
```

RViz 默认以 `map` 为 Fixed Frame，`EGOGlobalPath`（青色）和 `EGOOptimalPath`（红色）分别显示全局参考路径与局部优化控制点连线。路径 Marker 仍在 `odom` 坐标系，由 TF 显示在先验地图上；若打点后无路径，检查 `map→odom`、`/ego/visualization/global_path`、`/ego/visualization/optimal_path` 的消息及 `/ego/planning/bspline` 是否在规划成功后发布：

```bash
ros2 topic echo --once /ego/visualization/global_path --qos-durability transient_local
ros2 topic echo --once /ego/visualization/optimal_path --qos-durability transient_local
```

任务结束主动降落：

```bash
ros2 service call /land std_srvs/srv/Trigger '{}'
```

## 真机流程

目标飞控为 **MicoAir743v2-AIO-35A，PX4 1.14.3**。当前只有 D435i 和飞控，没有独立定位备份，因此 VIO 完全失效后无法继续自主定位。

### 1. 机械安装和标定

- D435i 刚性安装在机头正前方，左右红外和深度视野不得被桨叶、保护架或线束遮挡。
- 以实际运行的相机图像流标定左右红外内参、有效畸变、双目外参；随后用**相同分辨率、格式、图像话题及左右顺序**标定相机与 IMU 的时间偏差和外参。
- 沿用当前安装的实测 `T_body_imu`，定义为 **body FLU ← IMU** 的 4×4 变换。当前 D435i 的 `T_body_depth` 会从它和相机–IMU 外参推导；更换安装位置后重新测量。其他相机或深度坐标关系不同时显式提供 `T_body_depth`。
- 采集静止、平移和多方向转动数据，检查重投影误差、尺度、时间戳单调性和 IMU 噪声参数。

#### D435i 左右目内参、畸变与双目外参

当前实测设备为 D435i，序列号 `135122071701`。本轮选择两路 `640×480 Y8 @ 30 Hz`；ROS 驱动实际发布 `/camera/camera/infra1/image_rect_raw`、`/camera/camera/infra2/image_rect_raw`，均为 `mono8`。Y8 红外图像已由 RealSense 校正，因此 Kalibr 求得的是**该运行图像流的有效内参和剩余畸变**，不是镜头未校正原始像素的物理畸变。不要把旧的 `640×400 Y16` 标定用于这一路图像，也不要仅凭 `CameraInfo` 的零畸变替代本次结果。左右目顺序固定为 `infra1 → cam0`、`infra2 → cam1`；项目内部的 `/uav1/cam0/image_raw`、`/uav1/cam1/image_raw` 只是 `camera_stream` 的转发话题名，**不表示驱动输入必须叫 `image_raw`**。

关闭 RealSense Viewer，在独立终端启动相机；标定及以后真机运行均保持相同的左右目分辨率、格式和左右顺序：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=false enable_color:=false \
  enable_infra1:=true enable_infra2:=true \
  depth_module.infra_profile:=640x480x30 \
  depth_module.infra1_format:=Y8 depth_module.infra2_format:=Y8 \
  enable_gyro:=false enable_accel:=false
```

在另一终端关闭发射器，并检查两路图像、分辨率和实际频率。`image_rect_raw` 是本机已验证的话题；本轮没有 `/camera/camera/infra1/image_raw` 或 `/camera/camera/infra2/image_raw`：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 param set /camera/camera depth_module.emitter_enabled 0
ros2 topic echo --once /camera/camera/infra1/camera_info
ros2 topic echo --once /camera/camera/infra2/camera_info
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/infra2/image_rect_raw
```

本轮使用的实体标定板为 `6×6 AprilGrid`，单个 tag 黑边宽 `35 mm`、相邻黑边间距 `10 mm`；必须以**打印件实测尺寸**为准。建标定板文件，固定相机、缓慢移动标定板，使其覆盖画面中央、边缘、四角及不同距离和倾角，同时保持左右目清晰：

```bash
mkdir -p ~/kalibr_config ~/kalibr_bags
cat > ~/kalibr_config/target.yaml <<'EOF'
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.035
tagSpacing: 0.2857
EOF
ros2 bag record -o ~/kalibr_bags/stereo_d435i_640x480_y8_rect \
  /camera/camera/infra1/image_rect_raw \
  /camera/camera/infra2/image_rect_raw
```

本轮包时长 `190.9 s`，左/右目分别 `5727/5728` 帧，平均约 `30 Hz`。结束录包后先用 `ros2 bag info ~/kalibr_bags/stereo_d435i_640x480_y8_rect` 确认两路帧数，再转换并运行 Kalibr：

```bash
rosbags-convert \
  --src ~/kalibr_bags/stereo_d435i_640x480_y8_rect \
  --dst ~/kalibr_bags/stereo_d435i_640x480_y8_rect_ros1.bag \
  --src-typestore ros2_humble --dst-typestore ros1_noetic
docker run -it --rm \
  -v ~/kalibr_bags:/data/bags \
  -v ~/kalibr_config:/data/config \
  kalibr:latest
```

在 Kalibr 容器内：

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
cd /data/bags
rosrun kalibr kalibr_calibrate_cameras \
  --bag stereo_d435i_640x480_y8_rect_ros1.bag \
  --topics /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw \
  --models pinhole-radtan pinhole-radtan \
  --target /data/config/target.yaml --bag-freq 4.0 --dont-show-report
```

只有检查了生成的 `camchain*.yaml`、`results*.txt` 和报告后，才能把 camera-only 的结果用于下一阶段。重点检查左右重投影误差、主点是否在 `640×480` 画面内、双目基线是否接近设备约 `50 mm`、畸变是否与已校正图像相符；若某项异常，应检查图像模式、标定板实测尺寸和画面覆盖，不要手动缩放内参。本步骤**不生成** OpenVINS 所需的相机–IMU 外参或时间偏移。

本轮产物为 `~/kalibr_bags/stereo_d435i_640x480_y8_rect_ros1-camchain.yaml`。cam0/cam1 焦距约 `386.6/387.0 px`，主点约 `(321.3, 235.6)/(320.1, 235.6) px`；基线 `50.17 mm`。报告的重投影误差 X/Y 标准差分别约 `0.409/0.313 px` 和 `0.435/0.321 px`，角点分布覆盖画面大部分区域。两路拟合的 `k1` 约 `0.019/0.016`，属于已校正图像流的**剩余畸变拟合**；不要把它解释为镜头原始畸变。以上结果可用于下一阶段，但仍需同图像模式的相机–IMU 标定和无桨运动验证。

#### D435i 相机–IMU 联合标定（当前结果待 IMU 验证）

联合标定仍使用上述 `640×480 Y8 image_rect_raw` 双目流，另外启用 `/camera/camera/imu` 组合流。当前录包为 `~/kalibr_bags/camimu_d435i_640x480_y8_rect`：时长 `169.9 s`，左/右目 `5097/5098` 帧（约 `30 Hz`），IMU `34021` 帧（约 `200 Hz`）。录制时关闭红外发射器；相机驱动启用 `enable_gyro:=true enable_accel:=true gyro_fps:=200 accel_fps:=63 unite_imu_method:=2`。左右目使用与上一节完全相同的图像话题、分辨率、格式和顺序。确认三路数据同时存在后录制：

```bash
ros2 bag record -o ~/kalibr_bags/camimu_d435i_640x480_y8_rect \
  /camera/camera/infra1/image_rect_raw \
  /camera/camera/infra2/image_rect_raw \
  /camera/camera/imu
ros2 bag info ~/kalibr_bags/camimu_d435i_640x480_y8_rect
```

本轮 `~/kalibr_config/imu.yaml` 的噪声密度来自同一台 D435i 此前的静止数据分析；其中陀螺仪随机游走仍是暂定值，**本次联合标定没有重新估计 IMU 噪声或内部比例尺**。使用该文件及上一节的 camera-only `camchain.yaml` 运行 Kalibr：

```bash
rosbags-convert \
  --src ~/kalibr_bags/camimu_d435i_640x480_y8_rect \
  --dst ~/kalibr_bags/camimu_d435i_640x480_y8_rect_ros1.bag \
  --src-typestore ros2_humble --dst-typestore ros1_noetic
# 进入上一节的 Kalibr 容器后：
cd /data/bags
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag camimu_d435i_640x480_y8_rect_ros1.bag \
  --cam stereo_d435i_640x480_y8_rect_ros1-camchain.yaml \
  --imu /data/config/imu.yaml \
  --target /data/config/target.yaml --dont-show-report
```

当前产物为 `camimu_d435i_640x480_y8_rect_ros1-camchain-imucam.yaml`、同名前缀的 `-imu.yaml`、`-results-imucam.txt` 和 `-report-imucam.pdf`。cam0/cam1 重投影误差均值分别为 `0.278/0.290 px`；双目基线 `50.174 mm`，与 camera-only 结果一致。Kalibr 的相机时间偏移约为 `-2.702/-2.649 ms`，定义为 `t_imu = t_cam + timeshift_cam_imu`；导入程序会保留其符号，并把 Kalibr 的 `T_cam_imu` 求逆为 OpenVINS 的 `T_imu_cam`。这些结果说明图像几何和两路同步在本次数据上相互一致，但不能单凭这几点判定 IMU 及整机定位已可飞行。

本次加速度计归一化残差均值为 `4.44`（实际均值约 `0.055 m/s²`）；驱动还报告 `IMU Calibration is not available, default intrinsic and extrinsic will be used`，表示驱动读取不到设备的 IMU 标定数据而使用默认参数，不能仅凭此警告判断 IMU 无法工作。当前使用 `model: calibrated`，本轮沿用已有噪声参数，暂不进行六面静置及 IMU 内部参数重标。无桨联调时检查 IMU 数据连续性与 VIO 表现，出现问题后结合数据判断是否需要进一步标定；不要把暂定噪声值写成这次联合标定新测得的结果。参见 [Kalibr 相机–IMU 标定要求](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)和 [D435i IMU 说明](https://github.com/realsenseai/librealsense/blob/master/doc/d435i.md)。

当前安装的 `T_body_imu` 已有实测值：IMU 位于机体参考点前方约 `0.040 m`、左右居中、上方约 `0.014 m`。在 `/camera/camera/imu` 的 `frame_id` 为 `camera_imu_optical_frame` 且轴定义为 X 右、Y 下、Z 前时，`body FLU ← IMU` 为：

```yaml
T_body_imu:
  - [ 0.0,  0.0,  1.0,  0.040]
  - [-1.0,  0.0,  0.0,  0.000]
  - [ 0.0, -1.0,  0.0,  0.014]
  - [ 0.0,  0.0,  0.0,  1.000]
source: measured body mounting
```

将这份内容存为 `~/kalibr_config/body.yaml`。新一轮 `640×480` 相机内参和相机–IMU 标定不改变机体安装外参；但须核对实际安装未移动，并在无桨测试前确认 IMU 的 `frame_id` 与轴方向。对当前 D435i，设备 SDK 报告深度到左红外光学坐标系的变换为单位矩阵，因此真机 `depth_source:=hardware` 默认计算 `T_body_depth = T_body_imu × T_imu_cam0`；`T_imu_cam0` 来自本次 Kalibr 结果。自动推导只接受本文的左红外、深度图和深度 CameraInfo 话题组合，并检查运行时深度图及 CameraInfo 的 `camera_depth_optical_frame` 和尺寸；其他设备或对齐方式必须在 `body.yaml` 显式给出 `T_body_depth`。无桨测试仍须核对点云方向与实物障碍物一致。`tools/import_kalibr.py` 不会凭空生成机体安装关系。以上检查完成后，再用**本次联合标定**输出导入，不要使用旧的 `640×400` 文件：

```bash
# 当前参数已导入 deploy/calibration/uav1；下面演示重新导入到新目录。
python3 tools/import_kalibr.py \
  --camchain ~/kalibr_bags/camimu_d435i_640x480_y8_rect_ros1-camchain-imucam.yaml \
  --imu ~/kalibr_bags/camimu_d435i_640x480_y8_rect_ros1-imu.yaml \
  --body ~/kalibr_config/body.yaml \
  --output ~/one_drone/deploy/calibration/uav1_new
```

当前 D435i（序列号 `135122071701`）的四份配置已导入项目 [deploy/calibration/uav1](deploy/calibration/uav1/README.md)，包含中文参数注释；本轮三个方向的点云运动检查已由用户确认通过。真机启动命令使用该目录，启动时直接读取文件，修改标定参数无需 `colcon build`，重启节点即可生效。导入脚本拒绝覆盖已有目录；重新导入到 `uav1_new` 后，检查新结果并将启动参数 `calibration_dir` 切换到该目录。

启动文件在真机模式读取 `estimator_config.yaml`、`kalibr_imucam_chain.yaml`、`kalibr_imu_chain.yaml` 和 `body.yaml`。改变分辨率、镜头相对位置或相机设备后必须重标定；若仅改变整机安装位置，还需更新机体外参。仿真标定不能用于真机。

### 2. PX4 和传感器检查

- 在 QGroundControl 中确认机型、飞控朝向、传感器标定、遥控接管、急停、电池和失控保护。
- 配置 PX4 EKF2 融合外部视觉，确认实际 `MAV_SYS_ID` 和 uXRCE DDS 名称空间。
- 启动 D435i 的 `640×480 Y8 @ 30 Hz` 左右红外、深度、陀螺仪与加速度计；关闭红外发射器，使用 `unite_imu_method:=2` 的组合 IMU，并确认图像与 IMU 时间戳连续且同步。深度流另设为设备支持的运行档位。
- 无桨状态下验证 PX4 能收到 `vehicle_visual_odometry`，移动机体时 `/ego/odom`、RViz 与实物方向一致。

### 3. 启动真机算法

真机运行时先在独立终端启动 RealSense 驱动。以下为与本轮图像标定一致的左右目模式，同时打开硬件深度和组合 IMU；这套完整并发配置仍须在无桨台架上检查帧率和时间戳：

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

另开终端执行 `ros2 param set /camera/camera depth_module.emitter_enabled 0`，并核对 `/camera/camera/imu`、`/camera/camera/depth/image_rect_raw`、`/camera/camera/depth/camera_info` 均在持续发布；自动外参要求深度图与 CameraInfo 的 `header.frame_id` 均为 `camera_depth_optical_frame`。相机–IMU 联合标定阶段只需双红外与组合 IMU，硬件深度可先关闭以减少带宽。

```bash
cd ~/one_drone
source /opt/ros/humble/setup.bash
source ~/one_drone/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch bringup startup.launch.py
```

所有启动选项预先填写在 `src/bringup/params/launch.yaml`：`startup.sim: false` 选择 `hardware` 真机配置，其中已填入当前 D435i 图像、IMU、深度话题，以及 `~/one_drone/deploy/calibration/uav1` 标定目录、`target_system: 1` 和 `px4_ns: /`。更换设备、话题或飞控编号时修改此文件；`~` 会展开为当前用户目录。RealSense 驱动和飞控 Agent 仍须在算法启动前单独启动。

手动摆动检查定位时，将 `startup.flight_control` 设为 `false`（当前台架配置），启动相机和算法即可；无需连接飞控 DDS，`flight_bridge` 不启动，起飞服务和 Offboard 控制指令不会发布。移除桨叶、静置至 VIO 初始化后，缓慢前后、左右、上下移动机体并观察 `odom→base_link`；确认方向、位移尺度、静止漂移和回到起点后的误差。准备进行飞行控制联调时，将 `startup.flight_control` 改为 `true`，并先建立飞控 DDS 通信。仿真起飞同样需要该项为 `true`。

真机双目配对时间差由标定目录 `estimator_config.yaml` 中的 `stereo_sync_max_interval` 控制，当前为 `0.003 s`；一侧丢帧时丢弃无法同步的图像，不将相差一两帧的左右图像作为同一时刻的双目输入。`vio_bridge/config/params.yaml` 的 `future_tolerance: 0.02` 允许设备校时后时间戳略超前于电脑时钟，保留原始时间戳；真正过期、倒退、跳变或高协方差数据仍受原有检查约束。

D435i 的双目基线约 5 cm，真机 `estimator_config.yaml` 使用 `fi_max_baseline: 120.0`、`fi_max_cond_number: 100000.0`，支持几米外、短基线条件下的特征三角化。`fi_max_baseline` 是特征距离与有效基线的比值，不是以米表示的安装基线；默认 40 在机体静止后可能持续拒绝较远特征，造成“检测到角点，但没有视觉更新”。2026-10-06 的两分钟手动摆动录包中，原版独立回放末段发散至百米级，调整后末段静止位置保持稳定；这项回放没有独立真值，不能据此宣称实机绝对定位精度。相机内外参和 IMU 噪声未因本次排查更改。

启动算法前须确认组合 IMU `/camera/camera/imu` 和硬件深度两个话题实际存在；仅当前的双目内参录包配置关闭了 IMU 与深度，**不能直接用于真机飞行**。`target_system` 和 `px4_ns` 仍须按实际飞控核实。真机深度进入 EGO GridMap，OpenVINS 使用左右红外灰度图和 IMU。先完成无桨台架和小范围 0.5 m/s 试飞，再逐级放开速度。

## 安全状态与故障排查

`flight_bridge` 保留显式 `/takeoff`、`/land` 和 `/resume_navigation` 服务。VIO 为 `DEGRADED` 且里程计、深度仍新鲜时限速继续执行；任一输入陈旧时用 PX4 本地位置悬停。VIO 为 `INVALID` 或 PX4 本地位置失效时进入 `HOLD`，满足上述稳定条件后自动退出。`goal_manager` 在普通短时降级期间保存目标，恢复 `CRUISE` 后重新规划；VIO 重置事件会丢弃旧目标和恢复完成前保存的目标，需在重置完成后重新打点。`/resume_navigation` 保留为自动恢复条件长期不满足时的人工备用入口。本节点不会主动发送故障降落命令，PX4 自身 estimator failsafe 仍然具有最终控制权。

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
- 只有前视深度。目标落在后方时会先以低速转入视场，未知区域仍按用户选择不作为障碍，当前没有“未知区域禁止通行”的门控；这并不意味着未观测区域实际无障碍。
- 无独立定位备份时，OpenVINS 完全失效不能保证继续自主飞行。
- 仿真通过只证明软件链与当前模型兼容，不能替代真机标定、台架测试和受控场地试飞。

## 参数修改与生效

常用 ROS 参数位于各功能包 `config/params.yaml`，EGO 参数在 `src/bringup/params/ego_params.yaml`；仿真/真机选择、RViz、输入话题、标定目录、飞控身份、深度单位及进程重启设置统一在 `src/bringup/params/launch.yaml`。这些文件逐项提供中文含义、单位和覆盖来源。修改后执行 `colcon build --symlink-install` 并重启启动入口；多数参数在节点初始化时读取，不能假设 `ros2 param set` 后立即生效。

新增可调项包括：VIO 质量检测图像缩小比例、角点数量/质量/间距/窗口；软件双目的视差搜索范围、匹配窗口、唯一性及斑块过滤、OpenCV 线程数和同步队列；规划器到点容差；进程 respawn 延迟。角点统计属于外围质量检测，与 OpenVINS 的 `num_pts` 不同。

OpenVINS 前端与估计器参数在 `src/localization/vio_bridge/config/openvins_sim/estimator_config.yaml`（真机为自己的 `calibration_dir/estimator_config.yaml`）；启动入口不再强制覆盖日志级别、计时记录、线程数。标定几何和飞控身份仍以标定文件及启动参数为准，相关 YAML 项注明了覆盖来源。`sync_slop` 和 `frame_decimation` 留为 null 时自动选择仿真/真机默认值，填数值即可覆盖。


## 深度有效性、VIO 重置和时间同步

- 零深度、NaN、Inf、超出有效量程的像素不生成射线，不以最远量程替代，因此不能靠无效帧清除旧障碍。真实有效射线仍能更新空闲空间和清除旧占用。未知体素仍不阻止规划。
- `depth_filter/config/params.yaml` 的 `fill_holes` 默认 `false`，保留原始深度空洞；如主动开启插值，补出的深度会参与建图。VIO 重置会清掉时间滤波历史。
- `/vio_reset_event` 使用可靠、持久化的 `std_msgs/msg/Header`：`stamp` 是 ROS 时钟下的重置界限，`frame_id` 是唯一事件编号及原因，**不是 TF 坐标名**。桥接启动/重启、人工 reset、检测到不连续、接受恢复后的坐标时都会发布事件。
- 收到事件后：飞控桥接进入 HOLD；目标管理丢弃旧目标；EGO 清原始占用、膨胀层、射线计数和深度历史，取消规划状态；轨迹服务器停发旧 `PositionCommand` 并拒绝重放旧样条。排队的重置前深度、里程计及目标不能重新激活旧任务。
- `/ego/trajectory_reset_ready` 确认旧轨迹已停用；`/ego/map_reset_ready` 确认当前事件之后至少融合了 `reset_depth_frames` 帧有效且同步的深度。两者都使用同一个事件 Header；只有编号匹配、VIO 连续稳定、深度和 PX4 状态有效时才允许恢复。恢复 CRUISE 表示可接受新目标，不会继续旧任务。无需人工 map–odom 确认。
- 外部决策节点发 `/navigation_goal` 时必须使用当前 ROS 时间填写 `header.stamp`，不能重放重置前目标；仿真节点需要共同使用 `/clock`。整套仿真时钟重启时请同时重启算法节点。

可调参数（修改后重启节点）：

| 文件 | 参数 | 默认值及含义 |
|---|---|---|
| `src/navigation/ego_bridge/config/params.yaml` | `depth_timeout` | 0.5 s；根据图像源时间戳计算年龄，不是消息到达时间 |
| 同上 | `future_tolerance` | 0.02 s；允许源时间戳超前的容差 |
| `src/bringup/params/ego_params.yaml` | `grid_map/depth_max_age` | 0.5 s；建图允许的深度源时间戳年龄 |
| 同上 | `grid_map/depth_odom_sync_tolerance` | 0.05 s；深度与对应里程计的最大时间差 |
| 同上 | `grid_map/future_tolerance` | 0.02 s；深度超前时钟容差 |
| 同上 | `grid_map/reset_depth_frames` | 3；重置后有效建图帧数，空深度不计数 |

重复、倒退、过期、超前或不同步的深度会被拒绝；节点节流输出拒绝原因。源时间戳年龄与帧间隔不同：2.5 Hz 的新帧可以有效，但延迟 1 秒才送达的图不能刷新深度健康状态。

排错时查看：

```bash
ros2 topic echo /vio_reset_event --qos-durability transient_local
ros2 topic echo /ego/map_reset_ready --qos-durability transient_local
ros2 topic echo /ego/trajectory_reset_ready --qos-durability transient_local
ros2 topic echo /navigation_state
```

回归验证（先 source 工作空间，独立 ROS domain，不连接 PX4）：

```bash
ROS_DOMAIN_ID=83 python3 -m unittest discover -s tests -v
ROS_DOMAIN_ID=83 python3 tests/integration_depth_reset.py
```


### 轨迹时钟与运行恢复

- 轨迹生成、采样和重规划使用节点 ROS 时钟：仿真随 `/clock` 推进，暂停时轨迹不会继续走。计算耗时、规划器心跳失联检测使用单调时钟，避免暂停掩盖进程卡死。
- `/ego/map_heartbeat` 的 `frame_id` 是规划器实例标识；`stamp` 是最后成功融合的深度源时间。心跳仍在发但 stamp 不更新，表示进程存活但没有有效新地图。初始化/重置重建期间 stamp 为零。全无效深度、过期帧和不同步帧不会刷新地图时间。
- `DEGRADED` 下不能执行轨迹时只在进入悬停时记录位置，后续保持该位置，避免目标跟随漂移。
- 数据未准备好时目标排队；VIO、深度或地图恢复会重试尚未发送的目标，正常重复心跳不会反复下发同一目标。HOLD 恢复和规划器实例更换时允许重规划保留的目标。
- `planner_heartbeat_timeout` 和 `map_timeout` 分别位于 `flight_bridge/config/params.yaml`、`goal_manager/config/params.yaml` 及 `bringup/params/ego_params.yaml` 的 `traj_server` 段，默认均为 0.75 s；应保持三处一致。地图超时按 ROS 时间计算，不能用消息到达频率替代。

额外时钟回归测试：

```bash
ROS_DOMAIN_ID=84 python3 tests/integration_sim_clock.py
```


## 内置 OpenVINS 与旧工作区迁移

项目内包含 `ov_core`（特征跟踪和基础算法）、`ov_init`（静态/动态初始化）、`ov_msckf`（视觉惯性估计和 ROS 节点）、`ov_eval`（离线评估工具）。上游示例数据包 `ov_data` 不参与本项目运行，未引入。源码保留原许可证。

项目启动仍读取 `vio_bridge/config/openvins_sim` 或真机 `calibration_dir`；不需要修改 OpenVINS 上游示例配置。安装脚本不再下载或修改外部 OpenVINS 工作区，静态初始化补丁已纳入源码。

从旧版本升级请打开新终端，只 source ROS Humble，然后构建，避免继承旧的 OpenVINS overlay。也请移除个人 shell 启动文件中旧 VIO 工作区的自动 source 行。

```bash
cd ~/one_drone
git pull --ff-only
source /opt/ros/humble/setup.bash
# 初次迁移清理 CMake 缓存，避免旧依赖路径；限制并发降低内存峰值
MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install --executor sequential --cmake-clean-cache
source install/setup.bash
ros2 pkg prefix ov_msckf
# 应指向 ~/one_drone/install/ov_msckf
```

如果缺少系统依赖，先运行 `bash setup_env.sh sim` 或 `bash setup_env.sh onboard`。安装脚本默认串行构建包、每个包使用 2 个编译任务；资源允许时可设 `BUILD_JOBS=4`。日常源码、参数和 launch 修改仍使用项目根目录的 `colcon build --symlink-install`。旧外部 VIO 工作区可以保留备份，本项目不再依赖它。`startup.launch.py` 启动时会检查 `bringup` 与当前选中估计器（默认 `ov_msckf`）是否解析到同一个安装工作区，避免加载旧工作区的估计器。

### RViz 导航 TF 显示

RViz 通过 `rviz_tf_shift` 包的 `tf_display_filter.py` 接收独立的 `/rviz/tf`、`/rviz/tf_static`，默认仅包含 `map → odom → base_link`。显示列表在 `src/bringup/params/rviz_tf.yaml` 修改；原始 `/tf` 和 `/tf_static` 不受影响。新增过滤节点后先执行 `colcon build --symlink-install --packages-select rviz_tf_shift bringup` 并重新 source 工作空间，再重启启动文件。单独启动 RViz 时也需要启动过滤节点并重映射两个 TF 话题。


## ORB-SLAM3 双目惯性对照测试（默认关闭）

`startup.launch.py` 默认启动 OpenVINS，选择配置位于 `src/bringup/params/launch.yaml` 的 `startup.estimator: openvins`。ORB 源码与配置保留，可单独运行 `ros2 launch bringup orbslam_test.launch.py` 进行对照测试；该入口固定关闭飞控控制。测试前停止已有 startup，两套估计器不能同时发布 `/uav1/odomimu`。OpenVINS 的 ZUPT、`num_pts`、初始化配置继续作用于默认启动。

ORB-SLAM3 包位于 `src/localization/orb_slam3`，核心、Pangolin 与压缩词典随仓库提供，版本和补丁记录在 `UPSTREAM.md`；不需要外部 ORB 工作区。首次构建建议限制并行度：

```bash
cd ~/one_drone
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install --executor sequential
source install/setup.bash
ros2 launch bringup orbslam_test.launch.py
```

继续从 `src/bringup/params/launch.yaml` 选择真机/仿真与输入话题。台架测试保持 `flight_control: false`，真机相机驱动按前文单独启动。可调 ORB 参数及中文说明在 `src/localization/orb_slam3/config/params.yaml`。启动时从当前 `calibration_dir` 自动生成临时 ORB 配置：读取两目内参/残余畸变、双目外参、相机到 IMU 外参、IMU 噪声和时间偏移；额外执行标定矫正使双目满足 ORB 水平极线约束，并相应旋转相机到 IMU 的外参。机体安装参数仍由 `body.yaml` 提供，不使用官方示例标定。

真机 ORB 直接订阅 RealSense 话题，绕过用于软件深度的 Python 转发。输入图像与 IMU 需处于相同时间域；左右目时间差、IMU 间隔、积压和时间倒退都会检查。时间偏移按 Kalibr 的 `t_imu = t_cam + timeshift_cam_imu` 应用一次。

先让双目看到纹理丰富的环境，再缓慢进行平移及俯仰/横滚运动，以完成惯性初始化；一直静置可能无法完成初始化。初始化或跟踪丢失时不发布虚假定位：

```bash
ros2 topic echo /orbslam/tracking_state
ros2 topic hz /uav1/odomimu
ros2 topic echo /vio_health
```

`TRACKING` 表示惯性已初始化、跟踪成功且内点达标，其他状态用于排查初始化、丢失、同步或处理超时。输出为相机频率的 IMU 位姿与机体系速度，经过现有定位桥生成 `/odom` 与 `odom→base_link`；`map→odom` 仍由 modify 发布。ORB 重建地图、回环或全局校正时，经 `/estimator_reset` 转成 `/vio_reset_event`，清除旧 EGO 地图、轨迹及目标，需要重新打点。

上游不提供在线位姿协方差，配置中的方差是明确标注的桥接代理值；`VALID` 不能证明定位误差小于 10 cm。当前为算法对照版本，需实际完成摆动返回原点、重复轨迹和时间延迟测试，再评估是否适合飞行。词典解压和编译会增加本地磁盘占用，但未引入数据集或新录包。

### ORB 初始化校正与 TF 消失排查

ORB 的惯性初始化、全局惯性 BA 与尺度调整可能改变整个估计坐标系；这类事件与回环一样需要发出显式地图校正通知。清空并复用同一个地图 ID 也会增加校正计数。接口从同一份地图锁保护的位姿快照读取校正计数，在校正后的位姿进入定位桥前发布重置事件，清除旧规划数据。首次完成惯性初始化也会通知定位桥，不应把初始化坐标调整当作无人机真实瞬移。

`IMU_INITIALIZING` 与 `LOW_INLIERS` 已分别显示，并每 5 秒记录一次暂停原因、内点数及校正计数。跟踪确实丢失时仍暂停里程计；RViz 的 TF 显示超时为 15 秒，长时间没有新 `odom→base_link` 会隐藏坐标轴，不能将最后一帧反复更新为假实时定位来掩盖问题。

若出现 `Not enough motion for initializing. Reseting...`，是 ORB 上游在第二阶段惯性优化完成前检查到平移不足：仍需连续进行往返平移和小幅俯仰/横滚，不能仅绕 yaw 转动，也不能见到一帧 `TRACKING` 就立即结束运动。运动初始化时间取决于有效关键帧位移，并非简单等待若干秒。测试过程中保持 `flight_control: false`。

## 机体碰撞尺寸

已确认整机半径为 10 cm，包含桨叶最外缘。调整 `src/bringup/params/ego_params.yaml` 的 `grid_map/body_radius` 与 `grid_map/safety_margin`，规划器使用两者之和作为障碍物膨胀距离。当前为 `0.10 + 0.10 = 0.20 m`，体素边长仍为 `0.15 m`，膨胀按体素向上取整为 2 格，沿坐标轴范围为 `0.30 m`；真实间隙还受体素离散和占用点位置影响。尺寸参数不代表自体点云掩膜：真机自体掩膜仍需实测机架几何，仿真仍使用 x500 掩膜。旧配置只提供 `grid_map/obstacles_inflation` 时继续兼容；提供 `body_radius` 时以半径和安全余量为准。修改后编译并重启规划器生效。
