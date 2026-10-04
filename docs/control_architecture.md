# one_drone 控制链分层（FUEL / nexus-lite 方案第一阶段）

本阶段保留 OpenVINS → EGO → PX4。`flight_bridge` 仍是一个 ROS 2 进程；以下职责已拆成独立 Python 模块。只有 `px4_adapter` 发布 PX4 `OffboardControlMode`、`TrajectorySetpoint` 与 `VehicleCommand`，防止不同模块发出互相冲突的飞控指令。

| 模块 | 职责 | 不负责 |
| --- | --- | --- |
| `trajectory_validator` | 检查 EGO 样本的 odom 帧、时间戳、有限数值、动态上限、当前位置误差 | 障碍物碰撞判定；EGO 自身仍负责三维碰撞检查 |
| `trajectory_controller` | 轨迹速度前馈、位置误差反馈、速度限幅、到点死区 | PX4 消息、yaw、起飞状态 |
| `yaw_manager` | 根据当前/前瞻运动方向、相机水平 FOV 和 VIO 降级限制航向角速度与平移比例 | 改写 EGO XYZ 路径、估计视觉特征方向 |
| `flight_supervisor` | 起飞条件、HOLD 条件和自动恢复条件 | 直接发布 PX4 指令 |
| `px4_adapter` | 将已批准的 `FlightSetpoint` 转为 PX4 NED 设定值并记录审计话题 | 规划、安全决策 |

## 接口

`flight_interfaces/msg/FlightSetpoint` 是控制链内部提交给 `px4_adapter` 的类型，同时发布到 `/flight_setpoint` 便于录包。坐标为 PX4 本地 NED；`mode` 决定使用 `position` 还是 `velocity`，未使用的字段为 NaN。`trajectory_id` 来自 EGO；`confidence` 当前是控制模式标识（正常 1.0、降级 0.4、悬停 0.0），不能视为 VIO 的统计置信度，也不能替代安全门控。`SystemStatus` 发布到 `/system_status`，汇总飞行状态、VIO、PX4 位置、深度、地图及轨迹是否可用。原有 `/flight_state`、`/flight_hold_reason` 和 `/flight_safety_status` 保留。

控制链的继续条件保持原有语义：正常规划必须有新鲜 VIO、深度、EGO 地图与轨迹；VIO 降级且输入新鲜时限速；输入丢失时 HOLD 或本地位置悬停。VIO 重置仍清除旧轨迹并等待地图、轨迹重建。`trajectory_validator` 拒绝样本后，旧样本最多保持 `command_timeout`，之后不再用于速度控制。

## 本阶段 FOV 控制边界

仿真 D435i 左右目水平 FOV 为约 87°。`yaw_manager` 读取 EGO 当前速度与 0.3 秒前瞻加速度，计算预期运动方向相对机头的角度。超出半 FOV 时限制平移比例并让机头逐步跟上；到点死区内不转向。参数都在 `src/control/flight_bridge/config/params.yaml`。真机相机 FOV 应按标定数据核实。

本阶段尚无“某个朝向有多少已观测区域或视觉特征”的方向性地图。因此 FOV 限制只能减少盲飞，不能保证朝向有纹理，也不能阻止 EGO 把路径规划到完全无纹理的墙前。先验 region 墙体若接入，需同时进入规划占用图和 yaw 约束；仅改 yaw 不足以避墙。FUEL Frontier/Viewpoint 与任务命令在后续世界模型明确后再加入，不改变当前忽略未知区域的策略。

## 验证

```bash
cd ~/one_drone
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
PYTHONNOUSERSITE=1 python3 -m unittest discover -s tests -p 'test_*.py'
```

仿真飞行时检查 `/system_status` 与 `/flight_setpoint`。`/flight_setpoint` 是飞控已发布指令的审计镜像，订阅该话题不会触发控制。
