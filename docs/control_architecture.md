# one_drone 单机控制链

定位由 OpenVINS 或 MicoVIO 提供，EGO 规划三维路径，`flight_bridge` 管理飞行状态。MAVROS 独占飞控串口或 UDP 连接，所有飞控设定值和模式请求通过同一个 MAVROS 实例发送。

## 模块职责

| 模块 | 职责 |
| --- | --- |
| `goal_manager` | 接收手动或决策目标，转换 map→odom，交给 EGO |
| `decision_manager` | OFFBOARD 导航就绪后按顺序或循环执行航点 |
| EGO / `traj_server` | 占用图碰撞检查、B 样条规划与位置/速度/加速度采样 |
| `trajectory_validator` | 检查坐标系、时间戳、数值、动态上限及跟踪误差 |
| `trajectory_controller` | 速度前馈、位置误差反馈、速度限幅与到点死区 |
| `yaw_manager` | 根据轨迹前瞻方向和前视 FOV 生成独立航向策略 |
| `bootstrap_takeoff` | 无水平 odom 起飞阶段的高度/垂直速度闭环 |
| `flight_bridge` | 起飞、导航、悬停、降落和 OFFBOARD 退出处理 |
| `mavros_adapter` | 内部 NED 与 MAVROS ENU 转换，发布设定值，异步请求模式和解锁 |

## 规划与控制

打点规划需要当前位置和运行中的规划器，与起飞、OFFBOARD 和深度是否接入独立。无深度时没有实时障碍信息；先验 PGM 用于显示和打点，未知区域沿用可通行策略。

OFFBOARD 中，有新鲜且数值有效的 EGO 指令和里程计时跟踪轨迹，没有时锁定当前位置；位置也不可用时发布零速度，不能保证实际定点。退出 OFFBOARD 后取消当前自动起飞和速度跟踪，不主动重新切入或解锁。定位重置清除地图、轨迹和旧目标；规划器身份变化也会使旧指令失效。

`yaw_control: true` 使用轨迹速度和加速度前瞻生成 yaw；超出前视 FOV 时限制平移比例，到点死区内停止转向。`false` 保持初始机头方向，平移独立执行。参数位于 `src/control/flight_bridge/config/params.yaml`，总开关位于 `src/bringup/params/launch.yaml`。

## 自动起飞

`/takeoff` 在启用 `bootstrap_takeoff` 时先预发姿态设定值，请求 OFFBOARD，确认后请求解锁；实际解锁后按高度和垂直速度反馈离地。该阶段不要求水平 odom、地图或深度，但需要姿态与垂直反馈。离地后水平定位连续可用，切换位置控制升至设定高度。完成起飞后发布导航就绪状态，决策开始执行航点。

姿态起飞的悬停推力和限幅需要对应实际机型。初始化等待超时、垂直反馈丢失或高度越界会终止起飞；已解锁时请求飞控降落。未启用姿态起飞时，`/takeoff` 使用有效本地位置计算相对目标高度。

## 观察接口

- `/flight_state`：当前飞行状态。
- `/flight_navigation_ready`：是否已完成起飞并具备导航里程计。
- `/flight_hold_reason`、`/flight_safety_status`：悬停原因、轨迹检查和 MAVROS 请求状态。
- `/mavros/setpoint_raw/local`、`/mavros/setpoint_raw/attitude`：实际发送给 MAVROS 的设定值。
- `/vio_health`、`/vio_diagnostics`、`/ego/depth_fresh`：定位和深度诊断，直接观察对应节点输出。

## 验证

```bash
cd ~/one_drone
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q \
  tests src/decision/decision_manager/test src/localization/microvio/test
```

集成检查位于 `tests/integration_*.py`，使用隔离 ROS 域和合成数据；MAVROS 检查连接本机模拟飞控，不操作实机。
