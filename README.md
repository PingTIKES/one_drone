# one_drone

基于 **Ubuntu 22.04、ROS 2 Humble、OpenVINS、Nav2、PX4 v1.14.3** 的单机自主导航实验工程。当前任务只包含定位、先验地图全局规划、点云局部避障和飞行控制。仿真和真机使用同一套算法节点；Gazebo 只提供场景、相机、IMU 与飞行动力学，算法不读取 Gazebo 真值位姿。仓库不包含集群和目标识别任务。

> 状态：此前已在一台 PX4 1.14.3 SITL 无人机上验证前视双目/IMU → OpenVINS → Nav2 → PX4 Offboard → 到点。纯 odom 模式已验证 Nav2 激活和路径生成；本次切换后的默认 map 全局／odom 局部模式尚未完成整机飞行复测。**真机尚未试飞验证**，必须完成实测标定、逐项检查与无桨台架测试，再在有隔离和人工接管条件的场地试飞。

## 当前启动入口

启动分两步：`bringup` 负责相机、VIO、点云、飞控接口与 RViz；定位就绪后，再单独启动 `nav` 中的官方 Nav2 planner、controller、BT navigator 和 velocity smoother。**默认是 `map` 全局规划 + `odom` 局部控制**：RViz 的 Fixed Frame 和 2D Goal Pose 使用 `map`；先验 PGM 进入全局代价地图，深度点云进入 `odom` 局部代价地图。地图目标不再等待人工确认门控。当前 `map→odom` 的默认值是零平移、零旋转；只有它与实际地图坐标关系一致时，全局路径才对应真实场地。纯 `odom` 模式仍可显式选择。旧包名 `flight_bringup`、`one_drone_bringup` 已不再使用。

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

默认：先验 PGM /map ──> map 全局静态层 + 膨胀层
      默认零值 TF map→odom（不作为目标门控）
      已观测点云 ──> odom 局部障碍层 + 膨胀层
RViz 2D Goal Pose ──> goal_manager ──> Nav2 A* planner + DWB controller
                                                └──> velocity_smoother
                                                       └──> /cmd_vel_smoothed
                                                              └──> flight_bridge
                                                                    └──> PX4 NED 速度设定值
```

默认 `map` 模式下，`modify_map_to_odom` 持续发布 `map→odom` TF，初值在 `src/rviz/modify_map_to_odom/config/config.yaml` 中为 X/Y/Rotation 全零；`odom→base_link` **只由 OpenVINS 发布**。本次仅移除了 `/localization_ready` 的人工确认门控，RViz 保留手动调整面板，但不要求按「强制发布」。这不等于自动完成地图定位：如果无人机在地图上的真实位置与 VIO 原点不重合，零值 `map→odom` 会让全局路径映射到错误场地位置；不能仅凭 RViz 路径出现就判断可以安全飞行。仿真中的 Gazebo 真值位姿不进入算法。

旧的 RViz 手动工具仍保留在仓库，默认启动流程不要求操作。跨机使用时须保证节点处于同一 DDS 域，并用 `ros2 run tf2_ros tf2_echo map odom` 查看实际 TF。

规划是 2D 的，默认巡航高度 2 m。运行时的 `/navigation_obstacles` 是深度点云经过机体同高切片、稀疏化后的点云。默认 `map` 模式加载 1.5–2.5 m 高度层的先验 PGM；全局代价地图在 `map` 中使用先验静态层与膨胀层，局部代价地图在 `odom` 中只使用点云障碍层与膨胀层。点云只影响局部避障，不改写先验全局路径。可选纯 `odom` 模式使用 60 m 的滚动自由全局窗口；该模式看不到的障碍物不会进入全局规划，不能把未知空间当作已验证的安全空间。Nav2 行为树只做路径规划和路径跟踪，不包含 Spin、BackUp、Wait 等恢复行为；`NavigateThroughPoses` 的兼容树也使用同样的最小流程。Nav2 DWB 允许 x/y 平移，目标朝向容差宽，不要求机头先沿路径方向。相机看不到的障碍物不会凭空出现；默认地图模式用 PGM 表达静态场地障碍。

## 目录、功能包与接口

`src` 第一层按职责分为 `bringup/`、`perception/`、`localization/`、`navigation/`、`control/` 和 `rviz/`。`bringup/` 本身就是 ROS 2 包，直接包含 `launch/`、`map/`、`params/`、`rviz/`；其余目录内放对应功能包。Python 包的实现位于包内 `src/`。`bringup/launch/startup.launch.py` 在末尾逐行列出启动节点，方便上场时查看或注释；`navigation/nav/launch/bringup_launch.py` 是独立的 Nav2 入口。

```text
src/
├── bringup/                         # 传感器、定位、地图、控制启动和上场配置
│   ├── launch/startup.launch.py      # 文件末尾逐行列出启动节点，不启动 Nav2
│   ├── params/global_config.yaml     # 地图模式选用的全局地图
│   ├── params/nav2_params.yaml       # 默认 map 全局/odom 局部参数
│   ├── params/nav2_odom_params.yaml  # 可选纯 odom 导航参数
│   ├── map/                        # 先验 PGM/YAML
│   ├── behavior_trees/             # Nav2 行为树
│   ├── rviz/navigation.rviz          # 默认地图视图
│   └── rviz/navigation_odom.rviz     # 可选纯 odom 视图
├── perception/
│   ├── camera_stream/                # 真机相机与 IMU 话题转发
│   ├── stereo_depth/                 # 双目深度
│   └── obstacle_cloud/               # 深度转障碍点云
├── localization/
│   ├── vio_bridge/                   # OpenVINS 到里程计及 PX4
│   └── map_alignment/                # 保留的旧确认节点，默认不启动
├── navigation/
│   ├── nav/                          # 单独启动官方 Nav2 功能包
│   ├── obstacle_filter/              # 飞行高度障碍切片
│   └── goal_manager/                 # 目标门控
├── control/flight_bridge/            # 平滑速度转 PX4 Offboard 指令
└── rviz/
    ├── modify_map_to_odom/           # map→odom TF
    └── rviz_tf_shift/                # 保留的手动调整面板，无需操作
```

| 包 | 职责 | 主要输入 | 主要输出／可调配置 |
| --- | --- | --- | --- |
| `perception/camera_stream` | 真机左右目和 IMU 话题转发 | 相机驱动话题 | `/uav1/cam0/image_raw`、`/uav1/cam1/image_raw`、`/uav1/imu0` |
| `perception/stereo_depth` | 左右目软件双目匹配 | 左右灰度图、标定 | `/uav1/d435i/depth/image_raw` |
| `perception/obstacle_cloud` | 深度反投影及机架遮挡过滤 | 深度图、内外参 | `/uav1/obstacles` |
| `localization/vio_bridge` | 校验 VIO、跳变恢复、里程计、PX4 外部视觉 | `/uav1/odomimu`、双目时间戳 | `/odom`、`odom→base_link`、`/vio_health`、PX4 `vehicle_visual_odometry` |
| `localization/map_alignment` | 保留的旧人工确认节点，默认不启动 | `/map_odom/applied`、VIO 诊断 | `/localization_ready`、`/map_odom/current` |
| `navigation/obstacle_filter` | 巡航高度障碍切片 | `/uav1/obstacles`、里程计 | `/navigation_obstacles` |
| `navigation/goal_manager` | 目标和定位、感知状态门控 | `/navigation_goal`、定位与避障状态 | Nav2 `NavigateToPose` 目标、`/navigation_state` |
| `bringup` | 启动感知、定位、默认地图服务、飞控接口、RViz；集中管理上场配置 | YAML 参数、PGM 地图、传感器话题 | `/map`、`/odom`、`/navigation_obstacles`、启动清单 |
| `navigation/nav` | 单独启动官方 Nav2 planner、controller、BT navigator、velocity smoother | `/map`、TF、`/odom`、`/navigation_obstacles`、目标 | `/plan`、`/cmd_vel`、`/cmd_vel_smoothed` |
| `control/flight_bridge` | 起飞/悬停/降落、机体系 FLU 到 PX4 本地 NED 速度转换 | `/cmd_vel_smoothed`、PX4 本地状态 | PX4 Offboard 设定值、`/flight_state` |
| `rviz/modify_map_to_odom` | 哨兵式 map→odom TF 发布节点 | 共享内存、`/map_odom/set` | `/tf` 中的 `map→odom`、`/map_odom/applied` |
| `rviz/rviz_tf_shift` | 保留的手动地图调整面板，无需操作 | 操作者输入、导航话题 | `/map_odom/set` |

配置文件由 launch 加载。各包自身的参数位于包内 `config/params.yaml`；默认 Nav2 参数集中在 `src/bringup/params/nav2_params.yaml`；可选纯 odom 参数在 `src/bringup/params/nav2_odom_params.yaml`。赛场地图的 YAML 和图像放进 `src/bringup/map/`，在 `src/bringup/params/global_config.yaml` 的 `map:` 一行选用；临时切换可在启动 `bringup` 时传 `map_file:=/绝对路径/地图.yaml` 覆盖该设置。默认 `navigation_mode:=map` 会读取地图；纯 odom 模式不读取。默认 RViz 配置是 `src/bringup/rviz/navigation.rviz`。相机安装外参、PX4 system id 和驱动话题由启动参数覆盖；这些值必须来自当前飞机的实测或实际连接。按本仓库的 `--symlink-install` 构建后，修改已有 YAML、PGM 或 RViz 配置只需重启相关 launch；新增配置文件或修改安装规则时重新构建。核心运行参数无需编辑 Python 源码。

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

3. 终端 B：启动感知、定位、先验地图服务、飞控接口和 RViz。仿真时钟环境文件由终端 A 生成。

   ```bash
   cd ~/one_drone
   source /opt/ros/humble/setup.bash
   source ~/catkin_ws_ov/install/setup.bash
   source ~/one_drone/install/setup.bash
   source /tmp/one_drone_gz_env.sh
   PYTHONNOUSERSITE=1 ros2 launch bringup startup.launch.py sim:=true rviz:=true
   ```

4. 等待 `/vio_health` 为 `VALID`、`/odom` 和深度点云连续发布。启动时 OpenVINS 可能短暂打印 `[ZUPT]: There are no IMU data to check for zero velocity with!!`；只有随后变为 `VALID` 且里程计持续发布，才能继续。若提示持续出现或 VIO 始终无效，按下文“运行检查”的 IMU/双目频率与时间戳步骤排查，**不要起飞**。默认模式下，确认 `/map` 已发布、`ros2 lifecycle get /map_server` 为 `active`，以及 `odom→base_link` TF 正常。RViz **Fixed Frame = map**；不再等待 `/localization_ready`。在打点前仍要核对 RViz 中机体与场地障碍的相对位置和朝向，确认当前 `map→odom` 与场地相符；若不相符，先不要执行地图路径。Gazebo 真值位姿不进入算法。

5. 终端 C：单独启动 Nav2。与终端 B 使用相同的 ROS 工作空间和仿真时钟，等待 `/planner_server`、`/controller_server`、`/bt_navigator`、`/velocity_smoother` 都进入 `active` 后再打点；启动 Nav2 不需要重启终端 B。

   ```bash
   cd ~/one_drone
   source /opt/ros/humble/setup.bash
   source ~/catkin_ws_ov/install/setup.bash
   source ~/one_drone/install/setup.bash
   source /tmp/one_drone_gz_env.sh
   PYTHONNOUSERSITE=1 ros2 launch nav bringup_launch.py use_sim_time:=true
   ```

6. 确认 `/vio_health=VALID`、PX4 本地位置有效后发出起飞命令。深度点云暂时中断时仍可起飞并在 PX4 本地坐标悬停；VIO 或 PX4 本地定位失效时 `/takeoff` 会返回具体缺失项。无人机应先进入 `PRESTREAM`、`ARMING`、`TAKEOFF`，到约 2 m 后成为 `CRUISE`。**在 RViz 打 2D Goal Pose 前**，确认 `/obstacle_fresh=true`、Nav2 已激活，且飞机在先验地图上的位置与方向合理。导航目标经 Nav2 规划、控制和速度平滑后交给 PX4。观察 `/navigation_state`、`/flight_state`、`/plan`、`/cmd_vel_smoothed`。

   ```bash
   ros2 service call /takeoff std_srvs/srv/Trigger '{}'
   ros2 topic echo /flight_state
   ros2 topic echo /navigation_state
   # 完成后主动降落
   ros2 service call /land std_srvs/srv/Trigger '{}'
   ```

   RViz 的 2D Goal Pose 发布到 `/navigation_goal`，只能经过 `goal_manager` 进入 Nav2。也可用 `ros2 topic pub --once /navigation_goal geometry_msgs/msg/PoseStamped` 发点，默认 `header.frame_id` 必须为 `map`；纯 odom 模式才用 `odom`。`/navigation_state` 使用 transient local QoS，查看最近一次门控结果可运行 `ros2 topic echo --once /navigation_state --qos-durability transient_local`。目标方位不会强迫飞机先转向。默认地图模式会检查先验禁飞格和地图边界；纯 odom 模式的目标需位于 60 m 滚动规划窗口内。

### 可选：纯 odom 模式

如果暂时不使用先验地图，可在终端 B 的启动命令末尾加 `navigation_mode:=odom`，终端 C 同样加 `navigation_mode:=odom`。两个入口必须使用同一种模式。此时不启动地图服务和 `map→odom`，RViz Fixed Frame 为 `odom`，打点目标也使用 `odom`；无需手动地图对齐。全局规划窗口默认 60 m 且未观测区域视作可通行，局部代价地图仍用点云避障，因此不能预知视野外的墙。

默认地图模式的地图选择在 `src/bringup/params/global_config.yaml` 的 `map:` 一行；地图 YAML 和其 `image:` 指向的 PGM/PNG 放进 `src/bringup/map/`。当次启动可用 `map_file:=/absolute/path/to/field.yaml` 覆盖。内置 PGM 只适合仓库所附 RMUC 仿真场，真机必须换成实测场地地图。

## 真机：标定、连接和运行

目标飞控为 **MicoAir743v2-AIO-35A，PX4 1.14.3**。当前没有第二套独立定位传感器。PX4 对 OpenVINS 外部视觉的融合用于飞控本地位置；飞控估计与 OpenVINS 可能相关，二者不能当成互相独立的冗余定位。深度中断时取消当前导航目标并使用 PX4 本地位置悬停；VIO 或 PX4 本地位置失效时进入需要人工恢复的 `HOLD`。如果 PX4 自身丢失本地位置，PX4 的 failsafe 仍可能接管并降落。本工程无法在无定位备份时保证任何情况下都不会非撞击降落。

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

5. **无桨台架验证，再低风险试飞。** 默认使用先验 PGM 做全局规划；请换成实测场地地图。启动后检查 TF、/odom、/vio_health、/navigation_obstacles 和代价地图。确认 PX4 接受外部视觉且本地位置稳定后，才按场地规程做小范围起飞和本地悬停。移动飞机/转机头，确认 RViz 中 `odom→base_link` 的方向与真机一致；在地图目标测试前核对 `map→odom` 是否真的对应实测场地。全过程保留遥控人工接管。

   ```bash
   cd ~/one_drone
   source /opt/ros/humble/setup.bash
   source ~/catkin_ws_ov/install/setup.bash
   source ~/one_drone/install/setup.bash
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

   本地 VIO 定位正常后，在另一终端加载同样的三个 ROS 环境，并单独启动 Nav2：

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
ros2 topic echo --once /obstacle_fresh
ros2 topic hz /navigation_obstacles
ros2 topic echo --once /flight_state
ros2 topic echo --once /flight_hold_reason std_msgs/msg/String --qos-durability transient_local
ros2 topic echo --once /navigation_state
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
```

`HOLD` 时不会自动恢复导航；`/flight_hold_reason` 保留最近一次进入 `HOLD` 的原因。`/cmd_vel` 和 `/cmd_vel_smoothed` 有数据只证明 Nav2 在输出；`flight_bridge` 在 `HOLD` 时向 PX4 发位置保持设定值，不执行这些速度。默认地图模式不再使用 `/localization_ready` 门控；`map→odom` 若与实地不符，门控也不会替你发现。深度点云暂时中断时，`flight_bridge` 停止执行导航速度并使用 PX4 本地位置悬停，`goal_manager` 取消不安全的目标；点云恢复后需要重新打点。若曾发生 VIO 重置，原有目标已失效，需要确认定位恢复后重新打点；默认地图模式需要重新核对 `map→odom`。VIO 或 PX4 本地位置失效进入 `HOLD` 后排除原因，再调用 `/resume_navigation`。若 OpenVINS 的轨迹跳到几百米，停止试飞并录制左右目、IMU、`/uav1/odomimu`、`/vio_health` 和 PX4 本地位置/状态。默认地图模式中的黑色区域可能来自先验 PGM 或局部已观测障碍层，排查时分别看 RViz 的 `PriorMap`、`LocalCostmap` 和 `DepthObstacles`。可选纯 odom 模式只使用局部观测障碍。OpenVINS 的坐标原点任意，不能把仿真的 PX4 坐标直接当成地图坐标。

启动阶段的 `[init]: not enough feats to compute disp: 0,46 < 15` 表示初始化窗口前半段缺少可持续跟踪的特征；短暂出现后若 `/vio_health` 变为 `VALID`、`/uav1/odomimu` 连续发布，则初始化已完成。`[ZUPT]: There are no IMU data to check for zero velocity with!!` 是一次零速更新所需的**相机时间区间内**少于两条 IMU 样本，OpenVINS 会跳过这次零速更新；它本身不能证明整个 IMU 话题没有发布。`No IMU measurements to propagate with` 也是特定时间区间的样本不足。若持续出现或 `/vio_health` 不能变成 `VALID`，在算法运行时检查 `ros2 topic hz /uav1/imu0`（预期约 200 Hz）、`ros2 topic hz /uav1/cam0/image_raw` 和 `/uav1/cam1/image_raw`（各约 30 Hz），再用 `ros2 topic echo --once /uav1/imu0 --field header.stamp` 和相机的 `header.stamp` 核对是否处于同一仿真时间；同时确认终端 B 已 source 终端 A 生成的 `/tmp/one_drone_gz_env.sh`，且没有多套 Gazebo/PX4 残留。频率正常仍持续报错时，录制 `/clock`、双目和 IMU 供逐帧核对时间戳；不要靠关闭 ZUPT 掩盖时间同步问题。仿真退出后应由启动脚本回收 Gazebo、PX4 和 Agent；不要同时开启多套同名仿真。

## 版本与边界

`setup_env.sh` 固定 MicoAir PX4 1.14.3 提交、`px4_msgs` release/1.14 提交和 OpenVINS 提交，以减少消息协议及行为差异。Gazebo 物理与图像模型不是实物传感器的完整复制；仿真跑通只证明该链路在当前模型上工作。当前固定高度 2D 规划不会规划升降绕障；可选纯 odom 模式不会预知未观测障碍，两种模式都不支持移动障碍预判或 VIO 完全失效后的自主返航。多机、视觉目标识别和任务决策留待后续独立开发。
