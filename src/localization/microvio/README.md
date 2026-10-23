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

本轮是独立的实机定位检查入口。没有启动起飞、巡航或飞控控制节点，没有替换 `bringup/startup.launch.py` 的默认 OpenVINS 链路。

## 安装与启动

ROS 2 Humble；需要 `numpy`、`pyserial` 和 `pymavlink`。电脑已具备这些依赖；新设备缺少 pymavlink 时执行：

```bash
python3 -m pip install --user 'pymavlink>=2.4.41,<3'
cd ~/one_drone
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select microvio
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
- 输出保持 PX4 本地原点，不擅自减去首次位置或首次 yaw。`nav_msgs/Odometry` 的速度输出在机体系中，之后可交给现有 EGO 里程计适配器转换为世界速度。
- 主动 TIMESYNC 估算电脑和飞控时钟差，使用原始 ODOMETRY 的采样时刻；未同步、过期、重复、倒退的样本不生成新里程计或 TF。
- `reset_counter` 变化、源时钟重启或串口异常会产生重置事件。未来接导航时，必须将该事件接入现有清地图、清轨迹和旧目标的流程。
- `quality=0` 表示未知，不等于质量差；`quality=-1` 才表示失败。PX4 融合 ODOMETRY 的质量字段可能为 0，不能将其误当成模组原始质量。
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
| 本次健康 | DEGRADED，ESTIMATOR_STATUS 缺姿态有效位 |
| 高度状态 | `EKF2_HGT_REF=2`，但实际 `cs_rng_hgt=false`、`cs_baro_hgt=true`；仅设置参考源不保证测距当前有效或已参与融合 |
| DDS | `UXRCE_DDS_CFG=0`，未启用 |

这证明接线和遥测数据链路可用；没有进行摆动、闭环误差或飞行验证。没有修改飞控参数。模组安装偏移当前读取为 0，若实际不在机体参考点，需要按真实安装位置在 PX4 中配置。

## 后续接入导航需要完成的事

1. 确认 yaw 对齐状态及测距参与高度融合的条件；不能用删除健康检查来替代排查。
2. 接通与当前固件匹配的 DDS 通道，或实现统一的 MAVLink 控制适配。本项目现有 flight_bridge 仍通过 DDS 控制，不能因本包能读 USB 就认为已有控制链路。TELEM2 已被模块使用，可评估 TELEM1 供 RK3566 的 DDS 通信。
3. 互斥启动 OpenVINS 与 microvio，只保留一个 `odom→base_link` 发布者；把 odom、健康和重置事件接入现有导航接口，停止旧的 VIO→PX4 回灌节点，防止融合反馈环路。
4. 保留前视 D435i 的深度感知和准确的机体安装外参。MicoVIO 的下视 ToF 不能替代三维障碍物地图。

## 协议依据

- [MicoVIO 官方手册](https://micoair.cn/zh/docs/sensors/visual-inertial-odometry-sensor/micovio)
- [PX4 1.14 外部视觉接入](https://docs.px4.io/v1.14/en/ros/external_position_estimation)
- [PX4 1.14 DDS 接口](https://docs.px4.io/v1.14/en/middleware/uxrce_dds)
- [MAVLink ODOMETRY 定义](https://mavlink.io/en/messages/common.html#ODOMETRY)
