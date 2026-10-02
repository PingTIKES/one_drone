# one_drone

`one_drone` 是一台无人机的三维局部自主导航工程，运行环境为 **Ubuntu 22.04、ROS 2 Humble、OpenVINS、EGO-Planner / EGO-Swarm 单机模式和 MicoAir PX4 1.14.3**。仿真与真机使用同一套算法节点；Gazebo 只提供环境、相机、IMU 和飞行动力学，算法不读取仿真真值位姿。

当前仓库只保留单机定位、三维局部建图、轨迹规划和 PX4 控制。Nav2、PGM 地图服务器、2D 代价地图、行为树、`cmd_vel` 控制链、`map→odom` 人工调整及相关功能包已经移除。多机协同、目标识别和任务决策暂不在本阶段范围内。

> 当前状态：所有 ROS 2 功能包和总启动文件已在 Ubuntu 22.04 / Humble 上完成干净编译，并在 PX4 1.14.3 SITL 与 3 m 墙体识别柱场景中完成起飞、EGO 规划和轨迹执行闭环验证。2 m 前向目标的最终 VIO 水平位置误差约 7 cm；暂停深度处理 3 秒的故障注入中，飞机停止轨迹，深度恢复后从当前位置重新规划并到达目标。真机尚未试飞验证，因此默认速度限制为 0.5 m/s，真机验证稳定后再逐步提高。

## 算法链路

```text
左右红外灰度图 + IMU
  └─ OpenVINS
      ├─ /odom + TF odom→base_link
      ├─ /vio_health
      └─ PX4 vehicle_visual_odometry

/odom + /vio_health + /vio_diagnostics
  └─ ego_odom_adapter（机体系速度转 odom 世界系速度）
      └─ /ego/odom

双目深度图
  └─ depth_filter（空间去噪、时间滤波、小空洞填补）
      └─ CameraInfo + T_body_depth + /ego/odom
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
│   ├── depth_filter/                # 深度时空滤波与小空洞填补
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
| `depth_filter` | 软件或真机原始深度 | `/uav1/d435i/depth/image_filtered` | 包内 `config/params.yaml` |
| `obstacle_cloud` | 滤波深度、CameraInfo、相机外参 | `/uav1/obstacles`，仅调试显示 | 包内 `config/params.yaml` |
| `vio_bridge` | `/uav1/odomimu`、双目图像、IMU | `/odom`、`odom→base_link`、`/vio_health`、`/vio_diagnostics`、PX4 外部视觉 | 包内 `config/params.yaml` |
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
ros2 run tf2_ros tf2_echo odom base_link
```

必须满足：起飞前 `/vio_health` 为 `VALID`、`/ego/odom` 连续、深度为 `true`、三维膨胀占用持续发布、TF 方向与 Gazebo 中的移动一致。飞行中低置信度会显示 `DEGRADED`，输入仍新鲜时控制器限速继续；里程计或深度陈旧时改为位置悬停。持续断流或真实位姿跳变会显示 `INVALID` 并进入 `HOLD`；VIO 连续稳定 3 秒，且 PX4 仍已解锁、处于 OFFBOARD、位置有效、没有 failsafe 后，未发生坐标重置时，系统自动回到原飞行阶段，并从恢复后的新位置重新规划保存的目标；发生 VIO 重置时会清图、停旧轨迹、丢弃旧目标，等待地图重建后恢复，必须重新打点。当前过滤后的软件双目实测约 2.5–3 Hz、最大调度间隔接近 1 秒，因此控制层深度心跳超时为 1.2 秒，新增地图执行检查在最后成功融合的深度超过 0.75 个 ROS 秒时先停止执行，EGO 内部原有 1.5 秒深度超时仍保留；同步恢复后清除超时并从当前位置重新规划。若平均频率低于 2.5 Hz，应先解决算力或图像同步问题。RViz Fixed Frame 默认为 `odom`，白色背景，TF Marker Scale 为 2.5。

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

常用 ROS 参数位于各功能包 `config/params.yaml`，EGO 参数在 `src/bringup/params/ego_params.yaml`，进程重启设置在 `src/bringup/params/launch.yaml`。这些文件逐项提供中文含义、单位和覆盖来源。修改后执行 `colcon build --symlink-install` 并重启启动入口；多数参数在节点初始化时读取，不能假设 `ros2 param set` 后立即生效。

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
ROS_DOMAIN_ID=83 PYTHONNOUSERSITE=1 python3 -m unittest discover -s tests -v
ROS_DOMAIN_ID=83 PYTHONNOUSERSITE=1 python3 tests/integration_depth_reset.py
```


### 轨迹时钟与运行恢复

- 轨迹生成、采样和重规划使用节点 ROS 时钟：仿真随 `/clock` 推进，暂停时轨迹不会继续走。计算耗时、规划器心跳失联检测使用单调时钟，避免暂停掩盖进程卡死。
- `/ego/map_heartbeat` 的 `frame_id` 是规划器实例标识；`stamp` 是最后成功融合的深度源时间。心跳仍在发但 stamp 不更新，表示进程存活但没有有效新地图。初始化/重置重建期间 stamp 为零。全无效深度、过期帧和不同步帧不会刷新地图时间。
- `DEGRADED` 下不能执行轨迹时只在进入悬停时记录位置，后续保持该位置，避免目标跟随漂移。
- 数据未准备好时目标排队；VIO、深度或地图恢复会重试尚未发送的目标，正常重复心跳不会反复下发同一目标。HOLD 恢复和规划器实例更换时允许重规划保留的目标。
- `planner_heartbeat_timeout` 和 `map_timeout` 分别位于 `flight_bridge/config/params.yaml`、`goal_manager/config/params.yaml` 及 `bringup/params/ego_params.yaml` 的 `traj_server` 段，默认均为 0.75 s；应保持三处一致。地图超时按 ROS 时间计算，不能用消息到达频率替代。

额外时钟回归测试：

```bash
ROS_DOMAIN_ID=84 PYTHONNOUSERSITE=1 python3 tests/integration_sim_clock.py
```
