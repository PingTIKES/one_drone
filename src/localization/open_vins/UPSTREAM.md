# OpenVINS 源码来源

- 上游：https://github.com/rpng/open_vins
- 固定提交：`69488123ed9362dd44b6f28e7f4680abbff1442b`（版本 2.7.0）
- 许可证：GPL-3.0，见本目录 `LICENSE`；原作者声明保留在源码中。
- 直接纳入本仓库管理，不是符号链接或 Git 子模块，普通 clone 即可获得源码。
- 纳入 `ov_core`、`ov_init`、`ov_msckf`、`ov_eval` 和上游 `config`；不纳入 `ov_data` 示例数据集、上游文档站点和容器文件。
- 本地补丁：`ov_msckf/src/core/VioManager.h` 的 `initialized()` 在静态初始化成功后即可发布估计（RM27_STATIC_VIO），与此前验证过的外部工作区一致。

项目启动使用的仿真参数仍在相邻 `vio_bridge/config/openvins_sim`，真机仍使用用户标定目录；这里的 `config` 是上游示例配置。

在项目根目录执行 `colcon build --symlink-install`，只需 source 本项目 `install/setup.bash`。后续修改估计器源码可用 `colcon build --symlink-install --packages-up-to ov_msckf`。
