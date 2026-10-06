#!/usr/bin/env python3
"""Convert measured Kalibr stereo/IMU calibration without substituting example values."""
from pathlib import Path
import argparse
import json
import cv2
import numpy as np
import yaml


def read(path):
    return yaml.safe_load('\n'.join(x for x in Path(path).read_text().splitlines()
                                    if not x.startswith('%YAML')))


def prepare(estimator_path, params_path, output_dir):
    source = Path(estimator_path)
    cfg = read(source)
    cams = read(source.parent / cfg['relative_config_imucam'])
    imu = read(source.parent / cfg['relative_config_imu'])['imu0']
    params = read(params_path)['orbslam']['ros__parameters']
    matrices = []
    for key in ['cam0', 'cam1']:
        c = cams[key]
        if c['camera_model'] != 'pinhole' or c['distortion_model'] != 'radtan':
            raise ValueError('ORB conversion currently requires measured pinhole/radtan stereo')
        fx, fy, cx, cy = c['intrinsics']
        matrices.append(np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.]]))
    if cams['cam0']['resolution'] != cams['cam1']['resolution']:
        raise ValueError('stereo image sizes differ')
    size = tuple(cams['cam0']['resolution'])
    tic0 = np.array(cams['cam0']['T_imu_cam'], dtype=float)
    tic1 = np.array(cams['cam1']['T_imu_cam'], dtype=float)
    t10 = np.linalg.inv(tic1) @ tic0
    d0 = np.array(cams['cam0']['distortion_coeffs'])
    d1 = np.array(cams['cam1']['distortion_coeffs'])
    r0, r1, p0, p1, _, _, _ = cv2.stereoRectify(
        matrices[0], d0, matrices[1], d1, size, t10[:3, :3], t10[:3, 3],
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    if p1[0, 3] >= 0 or abs(p1[1, 3]) > 1e-6:
        raise ValueError('requires left/right ordered horizontal stereo')
    rect_to_cam = np.eye(4)
    rect_to_cam[:3, :3] = r0.T
    # ORB T_b_c1 maps rectified camera -> inertial sensor, same direction as T_imu_cam.
    tic_rect = tic0 @ rect_to_cam
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    settings = output / 'orbslam.yaml'
    entries = {
        'File.version': '1.0', 'Camera.type': 'Rectified',
        'Camera1.fx': float(p0[0, 0]), 'Camera1.fy': float(p0[1, 1]),
        'Camera1.cx': float(p0[0, 2]), 'Camera1.cy': float(p0[1, 2]),
        'Stereo.b': float(-p1[0, 3] / p1[0, 0]),
        'Camera.width': int(size[0]), 'Camera.height': int(size[1]),
        'Camera.fps': int(params['camera_fps']), 'Camera.RGB': 0,
        'Stereo.ThDepth': float(params['close_depth_baselines']),
        'IMU.T_b_c1': tic_rect.astype(np.float32), 'IMU.InsertKFsWhenLost': 0,
        'IMU.NoiseGyro': float(imu['gyroscope_noise_density']),
        'IMU.NoiseAcc': float(imu['accelerometer_noise_density']),
        'IMU.GyroWalk': float(imu['gyroscope_random_walk']),
        'IMU.AccWalk': float(imu['accelerometer_random_walk']),
        'IMU.Frequency': float(imu['update_rate']),
        'ORBextractor.nFeatures': int(params['n_features']),
        'ORBextractor.scaleFactor': float(params['scale_factor']),
        'ORBextractor.nLevels': int(params['n_levels']),
        'ORBextractor.iniThFAST': int(params['ini_fast']),
        'ORBextractor.minThFAST': int(params['min_fast']),
        'loopClosing': int(params['loop_closing']),
        'Viewer.KeyFrameSize': 0.05, 'Viewer.KeyFrameLineWidth': 1.0,
        'Viewer.GraphLineWidth': 0.9, 'Viewer.PointSize': 2.0,
        'Viewer.CameraSize': 0.08, 'Viewer.CameraLineWidth': 3.0,
        'Viewer.ViewpointX': 0.0, 'Viewer.ViewpointY': -0.7,
        'Viewer.ViewpointZ': -1.8, 'Viewer.ViewpointF': 500.0,
    }
    # OpenCV can read dotted ORB keys but its Python writer rejects them.
    lines = ['%YAML:1.0', '---']
    for key, value in entries.items():
        if isinstance(value, np.ndarray):
            lines += [key + ': !!opencv-matrix', '  rows: 4', '  cols: 4',
                      '  dt: f', '  data: ' + json.dumps(value.ravel().tolist())]
        else:
            lines.append(key + ': ' + json.dumps(value))
    settings.write_text('\n'.join(lines) + '\n')
    rectify = output / 'rectification.yaml.gz'
    fs = cv2.FileStorage(str(rectify), cv2.FILE_STORAGE_WRITE)
    for label, k, d, r, p in [('left', matrices[0], d0, r0, p0),
                             ('right', matrices[1], d1, r1, p1)]:
        x, y = cv2.initUndistortRectifyMap(k, d, r, p, size, cv2.CV_32FC1)
        fs.write(label + '_map_x', x)
        fs.write(label + '_map_y', y)
    fs.release()
    return str(settings), str(rectify), float(cams['cam0']['timeshift_cam_imu'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--estimator', required=True)
    parser.add_argument('--params', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.estimator, args.params, args.output)))
