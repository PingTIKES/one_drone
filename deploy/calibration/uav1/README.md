# 当前真机标定

设备为 D435i，序列号 `135122071701`。2026-10-06 导入，使用 2026-10-05 录制的双红外 `640×480 Y8 @ 30 Hz`、`image_rect_raw` 图像和组合 IMU。

- `estimator_config.yaml`：真机 OpenVINS 运行参数，修改后重启生效。
- `kalibr_imucam_chain.yaml`：本次联合标定优化后的相机内参、剩余畸变、相机–IMU 外参和时间偏移。原始结果为 `camimu_d435i_640x480_y8_rect_ros1-camchain-imucam.yaml`。
- `kalibr_imu_chain.yaml`：同一设备此前噪声估计值，陀螺仪随机游走仍为暂定值；本次没有重新做噪声或 IMU 内部标定。
- `body.yaml`：用户确认的实际安装关系，当前 IMU 前方 4 cm、左右居中、上方 1.4 cm，按用户提供的新安装矩阵更新。此前三个方向的点云运动检查已通过，但对应此前安装；该检查不代表当前安装平移量经过精密测量。

算法内部 cam0/cam1 依次对应 `/camera/camera/infra1/image_rect_raw` 和 `/camera/camera/infra2/image_rect_raw`，IMU 对应 `/camera/camera/imu`。驱动使用 `gyro_fps:=200 accel_fps:=63 unite_imu_method:=2`。

当前 D435i 的硬件深度使用独立 `CameraInfo` 内参；`T_body_depth` 自动从 `body.yaml` 和 cam0 外参计算，因此无需重复填写。启动参数及完整流程见项目根目录 README 真机部分。

该目录通过 `calibration_dir` 直接读取，无需 colcon 编译。使用新相机、新图像模式或改变安装后，应更新对应参数。导入脚本拒绝覆盖已有目录；重新导入时输出到新目录，再检查并切换 `calibration_dir`。

2026-10-06 精度对照：仅将 `num_pts` 从 150 增加到 300（初始化后双目约每目 150 的目标预算），其他估计参数保持原值；改动后的精度与 RK3566 耗时尚未验证。重启算法生效，无需编译。
