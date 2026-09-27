# one_drone

基于 **Ubuntu 22.04、ROS 2 Humble、OpenVINS、Nav2、PX4 v1.14.3** 的单机自主导航实验工程。当前任务只包含定位、先验地图、局部避障、路径规划和飞行控制。仿真和真机使用同一套算法节点；Gazebo 只提供场景、相机、IMU 与飞行动力学，算法不读取 Gazebo 真值位姿。仓库不包含集群和目标识别任务。

> 状态：已在一台 PX4 1.14.3 SITL 无人机上验证前视双目/IMU → OpenVINS → Nav2 → PX4 Offboard → 到点。**真机尚未试飞验证**，必须完成实测标定、逐项检查与无桨台架测试，再在有隔离和人工接管条件的场地试飞。

## 数据和控制路径

```text
仿真 Gazebo 或真机 D435i 的左右红外灰度图 + IMU
       │
       ├──> OpenVINS ──> /uav1/odomimu ──> vio_bridge ──> /odom + TF odom→base_link
       │                                                └──> PX4 external vision
       └──> 双目深度或真机深度 ──> /uav1/obstacles
                                         └──> 高度切片 /navigation_obstacles

先验 PGM /map ──> Nav2 静态层 + 障碍层 + 膨胀层
RViz 2D Pose Estimate ──> map_odom ──> 静态 TF map→odom
RViz 2D Goal Pose ──> goal_manager ──> Nav2 A* planner + DWB controller
                                                └──> velocity_smoother
                                                       └──> /cmd_vel_smoothed
                                                              └──> flight_bridge
                                                                    └──> PX4 NED 速度设定值
```

`map→odom` 表示手动地图对齐，仅在操作者重定位时更新；`odom→base_link` **只由 OpenVINS 发布**。在 RViz 中查看 `/map`、`/odom`、TF 和 `/plan`。2D Pose Estimate 指定无人机此刻在地图中的真实位置和朝向，系统按当前 VIO 位姿求 `T_map_odom = T_map_base × inverse(T_odom_base)`。右侧 `MapOdom` 面板也能直接输入 `map→odom` 的 x/y/yaw。重定位会取消正在执行的目标，需要重新打点。VIO 原点重置后也必须重新定位。

规划是 2D 的，默认巡航高度 2 m。PGM 是 1.5–2.5 m 高度层的先验障碍图；运行时的 `/navigation_obstacles` 是深度点云经过机体同高切片、稀疏化后的点云。局部/全局代价地图使用先验静态层、点云障碍层和膨胀层。Nav2 DWB 允许 x/y 平移，目标朝向容差宽，不要求机头先沿路径方向。相机看不到的动态障碍物不会凭空出现；静态场地障碍由 PGM 表达。

## 包与接口

| 包 | 职责 | 主要输入 | 主要输出／可调配置 |
| --- | --- | --- | --- |
| `uav_localization` | 校验 VIO、跳变恢复、TF/里程计和 PX4 外部视觉 | `/uav1/odomimu`、双目时间戳 | `/odom`、`odom→base_link`、`/vio_health`、PX4 `vehicle_visual_odometry`；`src/uav_localization/config/params.yaml` |
| `uav_perception` | 相机话题转接、软件双目、深度反投影和机架遮挡过滤 | 左右灰度图、IMU、深度图 | `/uav1/d435i/depth/image_raw`、`/uav1/obstacles`；`src/uav_perception/config/params.yaml` |
| `one_drone_navigation` | 手动地图对齐、巡航高度障碍切片、目标门控 | `/initialpose`、`/goal_pose`、点云、VIO 状态 | `map→odom`、`/navigation_obstacles`、Nav2 目标；`src/one_drone_navigation/config/params.yaml` |
| `one_drone_bringup` | PGM 地图、Nav2 planner/controller/costmaps/smoother、RViz | 地图和以上话题 | `/map`、`/plan`、`/cmd_vel_smoothed`；`src/one_drone_bringup/config/nav2.yaml` |
| `one_drone_control` | 起飞/悬停/降落服务、机体系 FLU 到 PX4 本地 NED 速度转换 | `/cmd_vel_smoothed`、PX4 本地位置和状态 | PX4 Offboard 设定值、`/flight_state`；`src/one_drone_control/config/params.yaml` |
| `uav_rviz_plugins` | RViz 手动 map→odom 面板 | 操作者输入 | `/map_odom/set` |

配置文件由 launch 加载。相机安装外参、PX4 system id 和驱动话题由启动参数覆盖；这些值必须来自当前飞机的实测或实际连接。修改配置后重新 `colcon build --symlink-install` 并重启 launch。核心运行参数无需编辑 Python 源码。

## 仿真：从零运行

1. 准备机器。安装 Ubuntu 22.04/ROS 2 Humble，确保 Gazebo Garden、图形驱动与磁盘空间充足。运行环境安装脚本，它会检出并校验固定版本的 MicoAir PX4 1.14.3、OpenVINS 与对应 `px4_msgs`，给 SITL 加入仿真时钟适配，构建算法工作区：

   ```bash
   git clone https://github.com/PingTIKES/one_drone.git ~/one_drone
   cd ~/one_drone
   ./setup_env.sh sim
   ```

   默认路径为 `~/PX4-Autopilot-1.14.3` 和 `~/catkin_ws_ov`，可用 `PX4_DIR`、`OV_WS` 环境变量修改。`setup_env.sh` 是执行脚本，**不要 `source`**。如果此前已有同名 PX4 目录但不是脚本要求的固定提交，先另选空目录作为 `PX4_DIR`。仓库只存储压缩后的 3 m 墙体模型；`scripts/prepare_field_model.sh` 会校验并解压它，仿真世界含六个有色识别柱。地图 PGM 与此世界配套。

2. 终端 A：启动一台 Gazebo/PX4 1.14.3 和 Micro XRCE Agent。默认打开 Gazebo 图形窗口；无图形环境使用 `HEADLESS=1`。

   ```bash
   cd ~/one_drone
   ./scripts/start_algorithm_sim.sh
   ```

3. 终端 B：启动全部算法、Nav2 和 RViz。仿真时钟环境文件由终端 A 生成。

   ```bash
   cd ~/one_drone
   source /opt/ros/humble/setup.bash
   source ~/catkin_ws_ov/install/setup.bash
   source ~/one_drone/install/setup.bash
   source /tmp/one_drone_gz_env.sh
   PYTHONNOUSERSITE=1 ros2 launch one_drone_bringup navigation.launch.py sim:=true rviz:=true
   ```

4. 等待 `/vio_health` 为 `VALID`、`/odom` 和深度点云连续发布。在 RViz **Fixed Frame = map**，点 `2D Pose Estimate`，在先验地图上选当前起飞点（默认 Gazebo 出生位置约 x=1.3、y=9.4），拖箭头指定真实机头方向。这一步是人工全局重定位，不等同于 VIO 初始化。检查 `map→odom→base_link` TF 连通且机体在地图上的位置正确。

5. 发出起飞命令。无人机应先进入 `PRESTREAM`、`ARMING`、`TAKEOFF`，到约 2 m 后成为 `CRUISE`。在 RViz 点击 **2D Goal Pose** 打一个空旷目标点，导航目标经 Nav2 规划、控制和速度平滑后交给 PX4。观察 `/navigation_state`、`/flight_state`、`/plan`、`/cmd_vel_smoothed`。

   ```bash
   ros2 service call /takeoff std_srvs/srv/Trigger '{}'
   ros2 topic echo /flight_state
   ros2 topic echo /navigation_state
   # 完成后主动降落
   ros2 service call /land std_srvs/srv/Trigger '{}'
   ```

   也可用 `ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped` 发点，`header.frame_id` 必须为 `map`。目标方位不会强迫飞机先转向。若目标在禁飞格、地图外或障碍内部，Nav2 会拒绝或报告无法找到路径。

## 真机：标定、连接和运行

目标飞控为 **MicoAir743v2-AIO-35A，PX4 1.14.3**。当前没有第二套独立定位传感器。PX4 对 OpenVINS 外部视觉的融合用于飞控本地位置；飞控估计与 OpenVINS 可能相关，二者不能当成互相独立的冗余定位。VIO 或深度中断时本工程取消导航、请求当前位置悬停并等待人工 `/resume_navigation`；如果 PX4 自身丢失本地位置，PX4 自己的 failsafe 仍可能接管并降落。本工程无法在无定位备份时保证任何情况下都不会非撞击降落。

1. **机械与时间同步。** 将 D435i 刚性安装在机头正前方，确保双红外、深度视野不被桨叶/机架挡住，测量机体 FLU 到相机 IMU 与深度光学系的外参。相机/IMU 数据需使用同一时基并确认时间戳单调；不可把不同电脑的未同步系统时间混用。确认飞机重心、桨方向、飞控朝向、遥控接管/急停、PX4 电池/失控保护和场地净空。

2. **离线标定。** 对该台 D435i 的双红外做内参、双目外参和相机 IMU 时延/外参标定（例如 Kalibr）；在静止和多方向运动数据上检查重投影误差、同步和尺度。测量 IMU 噪声密度/随机游走与 `T_body_imu`，硬件深度路径还需 `T_body_depth`。准备 `camchain-imucam.yaml`、`imu.yaml` 和 `body.yaml`，然后导入并校验：

   ```bash
   python3 tools/import_kalibr.py \
     --camchain /path/to/camchain-imucam.yaml \
     --imu /path/to/imu.yaml \
     --body /path/to/body.yaml \
     --output ~/one_drone_calibration
   ```

   `body.yaml` 中 `T_body_imu`、`T_body_depth` 是 4×4 矩阵，表示 **body FLU ← sensor**。输出目录必须是新目录。仿真内参/外参不能直接用于真机。使用标定对应的红外图像格式及分辨率；变更分辨率、镜头位置或设备需重标定。

3. **安装环境并确认 PX4 与 DDS。** 在伴随计算机运行 `./setup_env.sh onboard`。飞控刷机、参数、EKF2 外部视觉融合和 uXRCE DDS 启动需在 QGroundControl/PX4 侧按该机硬件配置完成，并确认实际 `MAV_SYS_ID`。启动 Micro XRCE Agent，检查 `/fmu/out/vehicle_local_position`、`/fmu/out/vehicle_status` 的**实际前缀**，以及飞控能接收 `vehicle_visual_odometry`。仿真的 `/px4_1/fmu/...` 是实例 1；真机可能是 `/fmu/...`，此时启动时传 `px4_ns:=/`。不要假设仿真的 system id 2 等于真机 id。

4. **相机驱动和话题检查。** 启动 `realsense2_camera` 的红外双目、深度、陀螺仪、加速度计和组合 IMU 流。用 `ros2 topic list`、`ros2 topic hz`、`ros2 topic echo --once .../camera_info` 核实左右目同步且与标定一致、IMU 频率和深度量纲。将下面的五个启动参数改为当前驱动的真实话题；相机断开或帧率过低时先修 USB 带宽与驱动。

5. **无桨台架验证，再低风险试飞。** 用对应场地的 PGM/YAML 先验地图（`map_file`）；内置地图仅适合仓库所附的 RMUC 仿真场。启动后检查 TF、/odom、/vio_health、/navigation_obstacles 和代价地图。在 RViz 用 2D Pose Estimate 对齐到场地地图，移动飞机/转机头，确认 RViz 方向与真机一致。确认 PX4 接受外部视觉且本地位置稳定，再上桨按场地规程做小范围悬停与近距离目标测试。全过程保留遥控人工接管。

   ```bash
   cd ~/one_drone
   source /opt/ros/humble/setup.bash
   source ~/catkin_ws_ov/install/setup.bash
   source ~/one_drone/install/setup.bash
   PYTHONNOUSERSITE=1 ros2 launch one_drone_bringup navigation.launch.py \
     sim:=false rviz:=true depth_source:=hardware \
     calibration_dir:=/home/ubuntu22/one_drone_calibration \
     map_file:=/absolute/path/to/field.yaml \
     target_system:=1 px4_ns:=/ \
     cam0_topic:=/actual/right/infrared \
     cam1_topic:=/actual/left/infrared \
     imu_topic:=/actual/synchronized/imu \
     depth_topic:=/actual/depth/image_rect_raw \
     depth_info_topic:=/actual/depth/camera_info
   ```

   `target_system` 和话题均为**示例占位**，运行前按真机修改。真机深度使用硬件深度数据，但 OpenVINS 仍使用左右红外灰度图和 IMU。测距的 `depth_scale` 默认 0.001（16 位毫米）；若驱动发布 32FC1 米，节点按米解释。飞行命令与仿真相同：`/takeoff`、RViz 2D Goal Pose、`/land`。失效进入 `HOLD` 后排除原因、重新定位，再调用 `/resume_navigation` 并重新打点。

## 排查与观测

```bash
ros2 topic hz /uav1/cam0/image_raw
ros2 topic hz /uav1/cam1/image_raw
ros2 topic hz /uav1/imu0
ros2 topic hz /uav1/odomimu
ros2 topic echo --once /vio_health
ros2 topic echo --once /localization_ready
ros2 topic hz /navigation_obstacles
ros2 topic echo --once /flight_state
ros2 topic echo --once /navigation_state
ros2 run tf2_ros tf2_echo map base_link
```

`HOLD` 时不会自动恢复导航；检查 VIO/深度/PX4 本地位置与地图对齐。如果 OpenVINS 的轨迹跳到几百米，停止试飞并录制左右目、IMU、`/uav1/odomimu`、`/vio_health` 和 PX4 本地位置/状态。地图中的黑色区域来自先验 PGM 或已观测障碍层，排查时分别看 RViz 的 `PriorMap`、`LocalCostmap` 和 `DepthObstacles`。OpenVINS 的坐标原点任意，不能把仿真的 PX4 坐标直接当成地图坐标。

## 版本与边界

`setup_env.sh` 固定 MicoAir PX4 1.14.3 提交、`px4_msgs` release/1.14 提交和 OpenVINS 提交，以减少消息协议及行为差异。Gazebo 物理与图像模型不是实物传感器的完整复制；仿真跑通只证明该链路在当前模型上工作。当前固定高度 2D 规划不会规划升降绕障，不会处理地图外未知场地、移动障碍预判或 VIO 完全失效后的自主返航。多机、视觉目标识别和任务决策留待后续独立开发。
