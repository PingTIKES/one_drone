# microvio：MicoVIO / PX4 定位遥测接入

本包用于读取已接入 PX4 的 MicoVIO 定位链路。模组在 TELEM2 通过 MAVLink 向飞控发送原始 VIO；本包通过电脑到飞控的 USB MAVLink 口读取 **PX4 融合后的 ODOMETRY**，不直接连接 TELEM2，也不再次向飞控写入外部视觉。

```text
下视 MicoVIO ──TELEM2──→ PX4 EKF2 ──USB MAVLink──→ microvio
                                                    ├─ /microvio/odom
                                                    ├─ /microvio/health
                                                    ├─ /microvio/diagnostics
                                                    ├─ /microvio/reset_event
                                                    └─ /microvio/range
```

支持独立入口和总启动入口。总启动在 `bringup/params/launch.yaml` 的 `hardware` 下设置 `localization_source: microvio`，统一输出 `/odom`、`/vio_health`、`/vio_diagnostics`、`/vio_reset_event` 和 `odom→base_link`，互斥关闭 OpenVINS 与 VIO→PX4 回灌节点。无相机检查设置 `perception_enabled: false`，仅关闭深度处理，EGO 和打点规划仍启动；无深度时没有实时避障；同时保持 `startup.sim: false`、`startup.flight_control: false`。执行 `ros2 launch bringup startup.launch.py` 即可查看地图和定位。两种入口不能同时占用串口。

## 安装与启动

ROS 2 Humble；需要 `numpy`、`pyserial` 和 `pymavlink`。电脑已具备这些依赖；新设备缺少 pymavlink 时执行：

```bash
python3 -m pip install --user 'pymavlink>=2.4.41,<3'
cd ~/one_drone
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to microvio
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch microvio microvio.launch.py
```

配置统一放在 `config/params.yaml`，含中文注释，修改后重启。`device` 是**电脑连接飞控的端口**，不是模组的 TELEM2；默认使用本机 `/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00`。串口不能同时被 QGroundControl、MicroXRCEAgent 或另一读取程序独占。

若用户无串口访问权限，可为当前设备临时授权：

```bash
sudo setfacl -m u:ubuntu22:rw /dev/ttyACM0
```

重新插拔后可能需要再次授权。长期使用可由设备管理员配置 dialout 组权限并重新登录。

在同样 source 和 RMW 环境的新终端检查：

```bash
ros2 topic hz /microvio/odom
ros2 topic echo /microvio/diagnostics
ros2 run tf2_ros tf2_echo microvio_odom microvio_base_link
```

RViz 的 Fixed Frame 使用 `microvio_odom`，添加 Odometry 显示 `/microvio/odom` 和 TF 即可观察。默认独立话题和坐标系用于避免与 OpenVINS 的 `/odom`、`odom→base_link` 冲突。

## 坐标、时间与健康

- PX4 的世界 NED 转换为项目采用的 NWU：`[x, y, z] → [x, -y, -z]`；机体 FRD 转为 FLU。NWU 与常见 ENU 不同。
- 默认 `initial_yaw_zero: true`：首次接受的机头水平朝向定义为 ROS odom 的 +X，初始 yaw 为 0；之后转动正常跟随，保留真实 roll/pitch。位置和姿态及姿态协方差使用同一个固定旋转；机体系速度不旋转。原点不平移、Z 不清零。设置为 false 可保留原始 NWU 航向。变换仅在节点启动后锁定一次，不因断流、恢复或重置重新归零；原始 PX4 数据保持不变。`nav_msgs/Odometry` 的速度输出在机体系中，之后可交给现有 EGO 里程计适配器转换为世界速度。
- 主动 TIMESYNC 估算电脑和飞控时钟差，使用原始 ODOMETRY 的采样时刻；未同步、过期、重复、倒退的样本不生成新里程计或 TF。
- `reset_counter` 变化、源时钟重启或串口异常会产生重置事件。总启动已将该事件接到 `/vio_reset_event`，供现有清地图、清轨迹和旧目标的流程使用。
- `quality=0` 表示未知，不等于质量差；`quality=-1` 才表示失败。PX4 融合 ODOMETRY 的质量字段可能为 0，不能将其误当成模组原始质量。
- MicoVIO 原始数据使用任意初始航向的 LOCAL_FRD，PX4 1.14.3 接受此航向后会主动将 `cs_yaw_align` 置为 false。这个标志不是本模式的初始化完成条件，也不要求旋转机体或与真北对齐。配置 `external_vision_reference: local_frd` 对应此模式；真正提供 NED 参考的设备才使用 `ned`。不要仅根据飞控融合输出的 frame_id 判断模组原始参考系。
- 健康消息根据遥测新鲜度和 PX4 `ESTIMATOR_STATUS` 计算。`VALID` 仅代表这些检查通过，**不代表独立验证了模组持续跟踪状态或绝对定位精度**。具体原因及缺失状态位见 diagnostics。
- PX4 常用 NaN 表示未知交叉协方差；目前输出对角近似，未知对角项使用配置的保守方差，而非填零。
- `/microvio/range` 仅转发下视测距，单位 m。没有发布猜测的测距传感器安装 TF。

节点只发送 GCS 心跳、TIMESYNC，以及可选的遥测频率请求。不会发送解锁、切模式、起飞、设定值、参数写入或 VIO 回灌。

## 已验证设备与本次结果

2026-10-24 的只读检查：

| 项目 | 结果 |
| --- | --- |
| 飞控 | MicoAir743v2，PX4 1.14.3 |
| 模组端口 | TELEM2，921600，MAVLink Onboard，转发关闭 |
| 外部视觉融合配置 | `EKF2_EV_CTRL=13`：水平位置、速度、yaw |
| 飞控内部实际状态 | `cs_ev_pos/vel/yaw=true`；`cs_yaw_align=false` |
| 原始模块单次快照 | `vehicle_visual_odometry.quality=91`，FRD 世界 / FRD 机体速度 |
| ROS 里程计静置采样 | 10 秒 301 条，29.98 Hz，无重置事件 |
| 静置位置峰峰变化 | X 约 2.0 mm、Y 约 3.5 mm、Z 约 48.1 mm；不是绝对定位误差或精度标定 |
| 健康判据 | LOCAL_FRD 模式不要求全局 yaw_align；仍检查实际四元数、时间戳、位置/速度有效状态、故障和重置 |
| 高度状态 | `EKF2_HGT_REF=2`，但实际 `cs_rng_hgt=false`、`cs_baro_hgt=true`；仅设置参考源不保证测距当前有效或已参与融合 |
| DDS | `UXRCE_DDS_CFG=0`，未启用 |

这证明接线和遥测数据链路可用；没有进行摆动、闭环误差或飞行验证。没有修改飞控参数。模组安装偏移当前读取为 0，若实际不在机体参考点，需要按真实安装位置在 PX4 中配置。

## 后续接入导航需要完成的事

1. 确认实际原始坐标参考模式及测距参与高度融合的条件；LOCAL_FRD 无需等待全局 yaw_align=true。
2. 接通与当前固件匹配的 DDS 通道，或实现统一的 MAVLink 控制适配。本项目现有 flight_bridge 仍通过 DDS 控制，不能因本包能读 USB 就认为已有控制链路。TELEM2 已被模块使用，可评估 TELEM1 供 RK3566 的 DDS 通信。
3. 总启动已互斥选择定位并接入统一接口；飞行联调前确认只运行一个总启动实例，防止重复 TF 和串口争用。
4. 保留前视 D435i 的深度感知和准确的机体安装外参。MicoVIO 的下视 ToF 不能替代三维障碍物地图。

## 协议依据

- [MicoVIO 官方手册](https://micoair.cn/zh/docs/sensors/visual-inertial-odometry-sensor/micovio)
- [PX4 1.14 外部视觉接入](https://docs.px4.io/v1.14/en/ros/external_position_estimation)
- [PX4 1.14 DDS 接口](https://docs.px4.io/v1.14/en/middleware/uxrce_dds)
- [MAVLink ODOMETRY 定义](https://mavlink.io/en/messages/common.html#ODOMETRY)

PX4 对 LOCAL_FRD 的具体处理见 [v1.14.3 ev_yaw_control.cpp](https://github.com/PX4/PX4-Autopilot/blob/v1.14.3/src/modules/ekf2/EKF/ev_yaw_control.cpp#L141)。
