# one_drone

基于 **Ubuntu 22.04、ROS 2 Humble、OpenVINS、Nav2、PX4 v1.14.3** 的单机自主导航实验工程。当前任务只包含定位、先验地图、局部避障、路径规划和飞行控制。仿真和真机使用同一套算法节点；Gazebo 只提供场景、相机、IMU 与飞行动力学，算法不读取 Gazebo 真值位姿。仓库不包含集群和目标识别任务。

> 状态：已在一台 PX4 1.14.3 SITL 无人机上验证前视双目/IMU → OpenVINS → Nav2 → PX4 Offboard → 到点。**真机尚未试飞验证**，必须完成实测标定、逐项检查与无桨台架测试，再在有隔离和人工接管条件的场地试飞。

## 当前启动入口

启动分两步：`bringup` 负责相机、VIO、点云、地图、飞控接口与 RViz；地图和定位就绪后，再单独启动 `nav` 中的官方 Nav2 planner、controller、BT navigator 和 velocity smoother。旧包名 `flight_bringup`、`one_drone_bringup` 已不再使用。

首次获取此版本或切换自旧目录结构后，先按下文安装依赖并在仓库根目录运行 `colcon build --symlink-install`。已有环境且已构建时，直接运行：

```bash
cd ~/one_drone
source /opt/ros/humble/setup.bash
source ~/catkin_ws_ov/install/setup.bash
source ~/one_drone/install/setup.bash
source /tmp/one_drone_gz_env.sh
PYTHONNOUSERSITE=1 ros2 launch bringup startup.launch.py sim:=true rviz:=true
```

另开终端，加载同一套 ROS 工作空间和仿真时钟环境后运行：

```bash
cd ~/one_drone
source /opt/ros/humble/setup.bash
source ~/catkin_ws_ov/install/setup.bash
source ~/one_drone/install/setup.bash
source /tmp/one_drone_gz_env.sh
PYTHONNOUSERSITE=1 ros2 launch nav bringup_launch.py use_sim_time:=true
```

完整仿真和真机流程见下文。

## 数据和控制路径

```text
仿真 Gazebo 或真机 D435i 的左右红外灰度图 + IMU
       │
       ├──> OpenVINS ──> /uav1/odomimu ──> vio_bridge ──> /odom + TF odom→base_link
       │                                                └──> PX4 external vision
       └──> 双目深度或真机深度 ──> /uav1/obstacles
                                         └──> 高度切片 /navigation_obstacles

先验 PGM /map ──> Nav2 静态层 + 障碍层 + 膨胀层
RViz MapOdomModify ──> Qt 共享内存 ──> modify_map_to_odom ──> TF map→odom
                    └──> map_odom 确认门控（仅用于地图目标规划）
RViz 2D Goal Pose ──> goal_manager ──> Nav2 A* planner + DWB controller
                                                └──> velocity_smoother
                                                       └──> /cmd_vel_smoothed
                                                              └──> flight_bridge
                                                                    └──> PX4 NED 速度设定值
```

`modify_map_to_odom` 使用哨兵工程的 Qt 共享内存机制持续发布 `map→odom` TF；其数值在操作者重定位前保持不变。`odom→base_link` **只由 OpenVINS 发布**。RViz 的 `MapOdomModify` 面板使用哨兵工程的圆形方向控件：W/S 调整 map→odom 的 X，A/D 调整 Y，Q/E 调整 yaw；也可输入 X/Y/Rotation 后按「强制发布」，Rotation 单位为弧度。这里输入的是 **TF 的平移和旋转，不是飞机在地图中的位姿**。起飞和 PX4 本地悬停只依赖本地定位，不要求先确认 map→odom；在先验地图上打点导航前，必须按场地中的实际位置确定地图坐标与 VIO 里程计坐标的关系。仿真中的 Gazebo 真值位姿不进入算法，也不是确认依据。默认 X/Y/Rotation 均为 0，可在 `src/rviz/modify_map_to_odom/config/config.yaml` 修改初值。手动调整会取消执行中的目标，需要重新打点；VIO 原点重置后必须重新确认地图定位。

Qt 共享内存在同一台电脑上传递面板调整；面板同时发布 `/map_odom/set`，TF 节点订阅该话题，所以 RViz 与真机伴随计算机分开运行时也能调整。跨机使用时须保证两个 ROS 2 节点处于同一 DDS 域，并用 `ros2 run tf2_ros tf2_echo map odom` 检查调整确实到达飞行端。

规划是 2D 的，默认巡航高度 2 m。PGM 是 1.5–2.5 m 高度层的先验障碍图；运行时的 `/navigation_obstacles` 是深度点云经过机体同高切片、稀疏化后的点云。全局代价地图使用先验静态层与膨胀层；局部代价地图使用先验静态层、点云障碍层与膨胀层。点云只影响局部避障，不改写先验全局路径。Nav2 DWB 允许 x/y 平移，目标朝向容差宽，不要求机头先沿路径方向。相机看不到的动态障碍物不会凭空出现；静态场地障碍由 PGM 表达。

## 目录、功能包与接口

`src` 第一层按职责分为 `bringup/`、`perception/`、`localization/`、`navigation/`、`control/` 和 `rviz/`。`bringup/` 本身就是 ROS 2 包，直接包含 `launch/`、`map/`、`params/`、`rviz/`；其余目录内放对应功能包。Python 包的实现位于包内 `src/`。`bringup/launch/startup.launch.py` 在末尾逐行列出启动节点，方便上场时查看或注释；`navigation/nav/launch/bringup_launch.py` 是独立的 Nav2 入口。

```text
src/
├── bringup/                         # 传感器、定位、地图、控制启动和上场配置
│   ├── launch/startup.launch.py      # 文件末尾逐行列出启动节点，不启动 Nav2
│   ├── params/nav2_params.yaml       # planner、controller、costmap、smoother
│   ├── map/                        # 先验 PGM/YAML
│   ├── behavior_trees/             # Nav2 行为树
│   └── rviz/navigation.rviz         # RViz 默认视图
├── perception/
│   ├── camera_stream/                # 真机相机与 IMU 话题转发
│   ├── stereo_depth/                 # 双目深度
│   └── obstacle_cloud/               # 深度转障碍点云
├── localization/
│   ├── vio_bridge/                   # OpenVINS 到里程计及 PX4
│   └── map_alignment/                # 地图对齐就绪门控
├── navigation/
│   ├── nav/                          # 单独启动官方 Nav2 功能包
│   ├── obstacle_filter/              # 飞行高度障碍切片
│   └── goal_manager/                 # 目标门控
├── control/flight_bridge/            # 平滑速度转 PX4 Offboard 指令
└── rviz/
    ├── modify_map_to_odom/           # map→odom TF
    └── rviz_tf_shift/                # 手动地图对齐面板
```

| 包 | 职责 | 主要输入 | 主要输出／可调配置 |
| --- | --- | --- | --- |
| `perception/camera_stream` | 真机左右目和 IMU 话题转发 | 相机驱动话题 | `/uav1/cam0/image_raw`、`/uav1/cam1/image_raw`、`/uav1/imu0` |
| `perception/stereo_depth` | 左右目软件双目匹配 | 左右灰度图、标定 | `/uav1/d435i/depth/image_raw` |
| `perception/obstacle_cloud` | 深度反投影及机架遮挡过滤 | 深度图、内外参 | `/uav1/obstacles` |
| `localization/vio_bridge` | 校验 VIO、跳变恢复、里程计、PX4 外部视觉 | `/uav1/odomimu`、双目时间戳 | `/odom`、`odom→base_link`、`/vio_health`、PX4 `vehicle_visual_odometry` |
| `localization/map_alignment` | 手动定位就绪和 VIO 重置门控 | `/map_odom/applied`、VIO 诊断 | `/localization_ready`、`/map_odom/current` |
| `navigation/obstacle_filter` | 巡航高度障碍切片 | `/uav1/obstacles`、里程计 | `/navigation_obstacles` |
| `navigation/goal_manager` | 目标和定位、感知状态门控 | `/navigation_goal`、定位与避障状态 | Nav2 `NavigateToPose` 目标、`/navigation_state` |
| `bringup` | 启动感知、定位、地图服务、飞控接口、RViz；集中管理上场配置 | YAML 参数、PGM 地图、传感器话题 | `/map`、`/odom`、`/navigation_obstacles`、启动清单 |
| `navigation/nav` | 单独启动官方 Nav2 planner、controller、BT navigator、velocity smoother | `/map`、TF、`/odom`、`/navigation_obstacles`、目标 | `/plan`、`/cmd_vel`、`/cmd_vel_smoothed` |
| `control/flight_bridge` | 起飞/悬停/降落、机体系 FLU 到 PX4 本地 NED 速度转换 | `/cmd_vel_smoothed`、PX4 本地状态 | PX4 Offboard 设定值、`/flight_state` |
| `rviz/modify_map_to_odom` | 哨兵式 map→odom TF 发布节点 | 共享内存、`/map_odom/set` | `/tf` 中的 `map→odom`、`/map_odom/applied` |
| `rviz/rviz_tf_shift` | RViz 手动地图对齐面板 | 操作者输入、导航话题 | `/map_odom/set` |

配置文件由 launch 加载。各包自身的参数位于包内 `config/params.yaml`；Nav2 planner、controller、costmap、inflation 和 velocity smoother 的参数集中在 `src/bringup/params/nav2_params.yaml`，由单独的 `nav` 启动文件读取。赛场地图放进 `src/bringup/map/`，启动 `bringup` 时用 `map_file:=/绝对路径/地图.yaml` 指定；RViz 默认配置是 `src/bringup/rviz/navigation.rviz`。相机安装外参、PX4 system id 和驱动话题由启动参数覆盖；这些值必须来自当前飞机的实测或实际连接。按本仓库的 `--symlink-install` 构建后，修改已有 YAML、PGM 或 RViz 配置只需重启相关 launch；新增配置文件或修改安装规则时重新构建。核心运行参数无需编辑 Python 源码。

## 仿真：从零运行

1. 准备机器。安装 Ubuntu 22.04/ROS 2 Humble，确保 Gazebo Garden、图形驱动与磁盘空间充足。运行环境安装脚本，它会检出并校验固定版本的 MicoAir PX4 1.14.3、OpenVINS 与对应 `px4_msgs`，给 SITL 加入仿真时钟适配，构建算法工作区：

   ```bash
   # 仅首次安装时执行；如果 ~/one_drone 已存在，跳过 git clone。
   git clone https://github.com/PingTIKES/one_drone.git ~/one_drone
   cd ~/one_drone
   bash setup_env.sh sim
   ```

   默认路径为 `~/PX4-Autopilot-1.14.3` 和 `~/catkin_ws_ov`，可用 `PX4_DIR`、`OV_WS` 环境变量修改。`setup_env.sh` 用 `bash` 执行，**不要 `source`**。如果此前已有同名 PX4 目录但不是脚本要求的固定提交，先另选空目录作为 `PX4_DIR`。若安装时出现 `packagekitd` 占用 `/var/lib/apt/lists/lock`，等待系统软件更新完成后重新运行 `bash setup_env.sh sim`；不要删除锁文件。仓库只存储压缩后的 3 m 墙体模型；`scripts/prepare_field_model.sh` 会校验并解压它，仿真世界含六个有色识别柱。地图 PGM 与此世界配套。

2. 终端 A：启动一台 Gazebo/PX4 1.14.3 和 Micro XRCE Agent。默认打开 Gazebo 图形窗口；无图形环境使用 `HEADLESS=1`。等终端输出 `[sim] Ready.` 后再执行第 3 步；Gazebo 窗口出现不代表相机和 IMU 已经开始发布。

   ```bash
   cd ~/one_drone
   bash scripts/start_algorithm_sim.sh
   ```

3. 终端 B：启动感知、定位、地图、飞控接口和 RViz。仿真时钟环境文件由终端 A 生成。

   ```bash
   cd ~/one_drone
   source /opt/ros/humble/setup.bash
   source ~/catkin_ws_ov/install/setup.bash
   source ~/one_drone/install/setup.bash
   source /tmp/one_drone_gz_env.sh
   PYTHONNOUSERSITE=1 ros2 launch bringup startup.launch.py sim:=true rviz:=true
   ```

4. 等待 `/vio_health` 为 `VALID`、`/odom` 和深度点云连续发布。启动时 OpenVINS 可能短暂打印 `[ZUPT]: There are no IMU data to check for zero velocity with!!`；只有随后变为 `VALID` 且里程计持续发布，才能继续。若提示持续出现或 VIO 始终无效，按下文“运行检查”的 IMU/双目频率与时间戳步骤排查，**不要起飞**。起飞前无需确认 map→odom。若随后要使用先验地图打点，另需检查 `/map` 已发布且 `ros2 lifecycle get /map_server` 显示 `active`；在 RViz **Fixed Frame = map**，依据场地中飞机的实际位置和朝向，用 `MapOdomModify` 调整 map→odom，按「强制发布」确认 `/localization_ready=true` 和 `map→odom→base_link` TF 连通。这里不读取 Gazebo 真值；所做的是将 VIO 的任意原点接到先验地图坐标系。

5. 终端 C：单独启动 Nav2。与终端 B 使用相同的 ROS 工作空间和仿真时钟，等待 `/planner_server`、`/controller_server`、`/bt_navigator`、`/velocity_smoother` 都进入 `active` 后再打点；启动 Nav2 不需要重启终端 B。

   ```bash
   cd ~/one_drone
   source /opt/ros/humble/setup.bash
   source ~/catkin_ws_ov/install/setup.bash
   source ~/one_drone/install/setup.bash
   source /tmp/one_drone_gz_env.sh
   PYTHONNOUSERSITE=1 ros2 launch nav bringup_launch.py use_sim_time:=true
   ```

6. 确认 `/vio_health=VALID`、PX4 本地位置有效后发出起飞命令。地图未确认或深度点云暂时中断时，仍可起飞并在 PX4 本地坐标悬停；VIO 或 PX4 本地定位失效时 `/takeoff` 会返回具体缺失项。无人机应先进入 `PRESTREAM`、`ARMING`、`TAKEOFF`，到约 2 m 后成为 `CRUISE`。**在 RViz 打 2D Goal Pose 前**，确认 `/localization_ready=true`、`/obstacle_fresh=true`、先验地图与场地位置关系正确、Nav2 已激活。导航目标经 Nav2 规划、控制和速度平滑后交给 PX4。观察 `/navigation_state`、`/flight_state`、`/plan`、`/cmd_vel_smoothed`。

   ```bash
   ros2 service call /takeoff std_srvs/srv/Trigger '{}'
   ros2 topic echo /flight_state
   ros2 topic echo /navigation_state
   # 完成后主动降落
   ros2 service call /land std_srvs/srv/Trigger '{}'
   ```

   RViz 的 2D Goal Pose 发布到 `/navigation_goal`，只能经过 `goal_manager` 进入 Nav2。也可用 `ros2 topic pub --once /navigation_goal geometry_msgs/msg/PoseStamped` 发点，`header.frame_id` 必须为 `map`。`/navigation_state` 使用 transient local QoS，查看最近一次门控结果可运行 `ros2 topic echo --once /navigation_state --qos-durability transient_local`。目标方位不会强迫飞机先转向。若目标在禁飞格、地图外或障碍内部，Nav2 会拒绝或报告无法找到路径。

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

3. **安装环境并确认 PX4 与 DDS。** 在伴随计算机运行 `bash setup_env.sh onboard`。飞控刷机、参数、EKF2 外部视觉融合和 uXRCE DDS 启动需在 QGroundControl/PX4 侧按该机硬件配置完成，并确认实际 `MAV_SYS_ID`。启动 Micro XRCE Agent，检查 `/fmu/out/vehicle_local_position`、`/fmu/out/vehicle_status` 的**实际前缀**，以及飞控能接收 `vehicle_visual_odometry`。仿真的 `/px4_1/fmu/...` 是实例 1；真机可能是 `/fmu/...`，此时启动时传 `px4_ns:=/`。不要假设仿真的 system id 2 等于真机 id。

4. **相机驱动和话题检查。** 启动 `realsense2_camera` 的红外双目、深度、陀螺仪、加速度计和组合 IMU 流。用 `ros2 topic list`、`ros2 topic hz`、`ros2 topic echo --once .../camera_info` 核实左右目同步且与标定一致、IMU 频率和深度量纲。将下面的五个启动参数改为当前驱动的真实话题；相机断开或帧率过低时先修 USB 带宽与驱动。

5. **无桨台架验证，再低风险试飞。** 用对应场地的 PGM/YAML 先验地图（`map_file`）；内置地图仅适合仓库所附的 RMUC 仿真场。启动后检查 TF、/odom、/vio_health、/navigation_obstacles 和代价地图。确认 PX4 接受外部视觉且本地位置稳定后，才按场地规程做小范围起飞和本地悬停。使用地图目标前，在 RViz 用 `MapOdomModify` 面板根据飞机在场地中的实测位置调整 map→odom，再按「强制发布」确认 `/localization_ready`；移动飞机/转机头，确认 RViz 方向与真机一致，然后做近距离目标测试。全过程保留遥控人工接管。

   ```bash
   cd ~/one_drone
   source /opt/ros/humble/setup.bash
   source ~/catkin_ws_ov/install/setup.bash
   source ~/one_drone/install/setup.bash
   PYTHONNOUSERSITE=1 ros2 launch bringup startup.launch.py \
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

   地图服务和定位正常后，在另一终端加载同样的三个 ROS 环境，并单独启动 Nav2：

   ```bash
   cd ~/one_drone
   source /opt/ros/humble/setup.bash
   source ~/catkin_ws_ov/install/setup.bash
   source ~/one_drone/install/setup.bash
   PYTHONNOUSERSITE=1 ros2 launch nav bringup_launch.py use_sim_time:=false
   ```

   `target_system` 和话题均为**示例占位**，运行前按真机修改。真机深度使用硬件深度数据，但 OpenVINS 仍使用左右红外灰度图和 IMU。测距的 `depth_scale` 默认 0.001（16 位毫米）；若驱动发布 32FC1 米，节点按米解释。飞行命令与仿真相同：`/takeoff`、RViz 2D Goal Pose、`/land`。失效进入 `HOLD` 后排除原因、重新定位，再调用 `/resume_navigation` 并重新打点。

## 排查与观测

```bash
ros2 topic hz /uav1/cam0/image_raw
ros2 topic hz /uav1/cam1/image_raw
ros2 topic hz /uav1/imu0
ros2 topic hz /uav1/odomimu
ros2 topic echo --once /vio_health
ros2 topic echo --once /localization_ready std_msgs/msg/Bool --qos-durability transient_local
ros2 topic echo --once /obstacle_fresh
ros2 topic hz /navigation_obstacles
ros2 topic echo --once /flight_state
ros2 topic echo --once /flight_hold_reason std_msgs/msg/String --qos-durability transient_local
ros2 topic echo --once /navigation_state
ros2 run tf2_ros tf2_echo map base_link
```

`HOLD` 时不会自动恢复导航；`/flight_hold_reason` 保留最近一次进入 `HOLD` 的原因。`/cmd_vel` 和 `/cmd_vel_smoothed` 有数据只证明 Nav2 在输出；`flight_bridge` 在 `HOLD` 时向 PX4 发位置保持设定值，不执行这些速度。`/localization_ready=false` 不阻止起飞和本地悬停，但会阻止或取消地图目标；RViz 调整 map→odom 后按「强制发布」确认，再打点。深度点云暂时中断时，`flight_bridge` 停止执行导航速度并使用 PX4 本地位置悬停，`goal_manager` 取消不安全的地图目标；点云恢复后需要重新打点。若曾发生 VIO 重置，还需重新对齐。VIO 或 PX4 本地位置失效进入 `HOLD` 后排除原因，再调用 `/resume_navigation`。若 OpenVINS 的轨迹跳到几百米，停止试飞并录制左右目、IMU、`/uav1/odomimu`、`/vio_health` 和 PX4 本地位置/状态。地图中的黑色区域来自先验 PGM 或局部已观测障碍层，排查时分别看 RViz 的 `PriorMap`、`LocalCostmap` 和 `DepthObstacles`。OpenVINS 的坐标原点任意，不能把仿真的 PX4 坐标直接当成地图坐标。

启动阶段的 `[init]: not enough feats to compute disp: 0,46 < 15` 表示初始化窗口前半段缺少可持续跟踪的特征；短暂出现后若 `/vio_health` 变为 `VALID`、`/uav1/odomimu` 连续发布，则初始化已完成。`[ZUPT]: There are no IMU data to check for zero velocity with!!` 是一次零速更新所需的**相机时间区间内**少于两条 IMU 样本，OpenVINS 会跳过这次零速更新；它本身不能证明整个 IMU 话题没有发布。`No IMU measurements to propagate with` 也是特定时间区间的样本不足。若持续出现或 `/vio_health` 不能变成 `VALID`，在算法运行时检查 `ros2 topic hz /uav1/imu0`（预期约 200 Hz）、`ros2 topic hz /uav1/cam0/image_raw` 和 `/uav1/cam1/image_raw`（各约 30 Hz），再用 `ros2 topic echo --once /uav1/imu0 --field header.stamp` 和相机的 `header.stamp` 核对是否处于同一仿真时间；同时确认终端 B 已 source 终端 A 生成的 `/tmp/one_drone_gz_env.sh`，且没有多套 Gazebo/PX4 残留。频率正常仍持续报错时，录制 `/clock`、双目和 IMU 供逐帧核对时间戳；不要靠关闭 ZUPT 掩盖时间同步问题。仿真退出后应由启动脚本回收 Gazebo、PX4 和 Agent；不要同时开启多套同名仿真。

## 版本与边界

`setup_env.sh` 固定 MicoAir PX4 1.14.3 提交、`px4_msgs` release/1.14 提交和 OpenVINS 提交，以减少消息协议及行为差异。Gazebo 物理与图像模型不是实物传感器的完整复制；仿真跑通只证明该链路在当前模型上工作。当前固定高度 2D 规划不会规划升降绕障，不会处理地图外未知场地、移动障碍预判或 VIO 完全失效后的自主返航。多机、视觉目标识别和任务决策留待后续独立开发。
