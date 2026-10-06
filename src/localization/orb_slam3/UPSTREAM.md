# Bundled upstream sources

- ORB-SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3, commit `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`, GPL-3.0-or-later. Only core sources, required third-party code and compressed vocabulary retained.
- Pangolin: https://github.com/stevenlovegrove/Pangolin, v0.6 commit `dd801d244db3a8e27b7fe8020cd751404aa818fd`, license in vendor/Pangolin/LICENCE. GUI library only; viewer disabled at runtime.
- Local changes: read-only IMU state snapshot API; portable build outputs and flags; correct the upstream global-BA generation counter from bool to int for C++17. Skip the upstream null second-camera diagnostic for Rectified mode (both cameras use camera1 intrinsics). ROS 2 wrapper, calibration conversion and test settings are maintained in this package.
