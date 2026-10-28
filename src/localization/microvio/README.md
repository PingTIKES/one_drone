# microvio：MicoVIO / PX4 定位遥测接入

本包通过 MAVROS 读取 PX4 融合后的 MAVLink ODOMETRY。MicoVIO 接飞控 TELEM2，MAVROS 独占电脑到飞控的串口；本包只订阅 `/uas<target_system>/mavlink_source`、MAVROS 连接状态和时间同步状态，不打开串口、不发送飞行控制、不回灌定位。

总启动在 `bringup/params/launch.yaml` 中配置 `hardware.localization_source: microvio` 和 MAVROS 的 `fcu_url`，然后执行：

```bash
source /opt/ros/humble/setup.bash
source ~/one_drone/install/setup.bash
ros2 launch bringup startup.launch.py
```

总启动统一输出 `/odom`、`/vio_health`、`/vio_diagnostics`、`/vio_reset_event` 以及 `odom→base_link`。`perception_enabled: false` 关闭深度处理，不阻止规划或 OFFBOARD 轨迹执行。若只观察定位，设置 `flight_control: false`。

独立 `microvio.launch.py` 只启动适配节点，要求 MAVROS 已启动。默认独立话题为 `/microvio/odom`、`/microvio/health`、`/microvio/diagnostics`，便于台架观察。

## 坐标、时间与健康

- PX4 的世界 NED 转换为项目采用的 NWU：`[x, y, z] → [x, -y, -z]`；机体 FRD 转为 FLU。NWU 与常见 ENU 不同。
- 默认 `initial_yaw_zero: true`、`initial_position_zero: true`：水平定位有效后锁定初始机头方向和 XY 原点；空中初始化时保留相对地面的高度，优先采用下视测距记录离地高度。地面恒定位置状态只发布临时坐标供观察，不会锁死正式导航原点。临时坐标切换到正式坐标时发布一次重置事件，清除旧地图、轨迹和目标。后续源重置不重新归零。世界姿态/协方差使用固定旋转，机体系速度保持不变。
- 使用 MAVROS TimesyncStatus 提供的电脑与飞控时钟差，使用原始 ODOMETRY 的采样时刻；未同步、过期、重复、倒退的样本不生成新里程计或 TF。
- 原始遥测队列默认 20 帧，只解码估计器状态、里程计和测距；连接心跳独立使用 MAVROS State，避免被高频遥测挤掉。默认通过 MAVROS 在当前连接停发报文 30、105，保留报文 31 提供姿态与角速度，减少 RK3566 的串口遥测解析积压；`suppressed_mavlink_message_ids: []` 可保留全部遥测。此设置不写入持久飞控参数。
- `reset_counter` 变化、源时钟重启或串口异常会产生重置事件。总启动已将该事件接到 `/vio_reset_event`，供现有清地图、清轨迹和旧目标的流程使用。
- `quality=0` 表示未知，不等于质量差；`quality=-1` 才表示失败。PX4 融合 ODOMETRY 的质量字段可能为 0，不能将其误当成模组原始质量。
- MicoVIO 原始数据使用任意初始航向的 LOCAL_FRD，PX4 1.17.0 接受此航向后会主动将 `cs_yaw_align` 置为 false。这个标志不是本模式的初始化完成条件，也不要求旋转机体或与真北对齐。配置 `external_vision_reference: local_frd` 对应此模式；真正提供 NED 参考的设备才使用 `ned`。不要仅根据飞控融合输出的 frame_id 判断模组原始参考系。
- 健康消息根据遥测新鲜度和 PX4 `ESTIMATOR_STATUS` 计算。`VALID` 仅代表这些检查通过，**不代表独立验证了模组持续跟踪状态或绝对定位精度**。具体原因及缺失状态位见 diagnostics。
- 通过坐标、数值和采样时间检查的遥测仍会发布；恒定位置模式、未知协方差等原因如实反映在健康诊断中。`/microvio/navigation_ready` 周期发布正式原点和水平辅助是否就绪，过期样本不能使该状态保持为 true。
- PX4 常用 NaN 表示未知交叉协方差；目前输出对角近似，未知对角项使用配置的保守方差，而非填零。
- `/microvio/range` 转发下视测距，单位 m；飞行桥可将其用于离地初始化阶段的高度反馈。没有发布猜测的测距传感器安装 TF。

节点只通过 MAVROS 请求遥测频率，不发送解锁、切模式、起飞、设定值、参数写入或 VIO 回灌。

## 接入导航

总启动只运行一个 MAVROS，`target_system` 必须与飞控 MAV_SYS_ID 一致。本节点订阅同一目标系统的原始遥测，保留 ODOMETRY 重置计数、源时钟、质量和协方差信息。MAVROS 负责串口、MAVLink 心跳、时间同步与控制指令。切勿再独立占用同一串口。

## 协议依据

- [MicoVIO 官方手册](https://micoair.cn/zh/docs/sensors/visual-inertial-odometry-sensor/micovio)
- [PX4 1.17.0 外部视觉接入](https://docs.px4.io/v1.17/en/ros/external_position_estimation)
- [MAVLink ODOMETRY 定义](https://mavlink.io/en/messages/common.html#ODOMETRY)

PX4 对 LOCAL_FRD 的具体处理见 [v1.17.0 ev_yaw_control.cpp](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/ekf2/EKF/aid_sources/external_vision/ev_yaw_control.cpp)。
