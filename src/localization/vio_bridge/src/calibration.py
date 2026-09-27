"""Validate OpenVINS/Kalibr configuration; never fabricate hardware calibration."""
from pathlib import Path
import numpy as np
import yaml


class OpenCVDumper(yaml.SafeDumper):
    pass


OpenCVDumper.add_representer(list,lambda dumper,value:dumper.represent_sequence('tag:yaml.org,2002:seq',value,flow_style=True))


def write_opencv_yaml(path,value):
    # FileStorage rejects PyYAML's default indentless block sequences.
    Path(path).write_text('%YAML:1.0\n'+yaml.dump(value,Dumper=OpenCVDumper,sort_keys=False,width=10000),encoding='utf-8')


def read_yaml(path):
    text = Path(path).read_text(encoding='utf-8')
    return yaml.safe_load('\n'.join(l for l in text.splitlines() if not l.startswith('%YAML')))


def transform(value):
    t = np.asarray(value,dtype=float)
    if t.shape != (4,4) or not np.isfinite(t).all(): raise ValueError('invalid 4x4 transform')
    if not np.allclose(t[3],[0,0,0,1]): raise ValueError('invalid homogeneous row')
    if not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(t[:3,:3]),1,atol=1e-5):
        raise ValueError('rotation must be right-handed orthonormal')
    return t


def validate_config(path):
    path = Path(path)
    cfg = read_yaml(path)
    cams = read_yaml(path.parent/cfg['relative_config_imucam'])
    imu = read_yaml(path.parent/cfg['relative_config_imu'])['imu0']
    if cfg.get('max_cameras') != 2 or not cfg.get('use_stereo'): raise ValueError('requires stereo VIO')
    for name in ('cam0','cam1'):
        cam = cams[name]
        transform(cam['T_imu_cam'])
        intr = np.asarray(cam['intrinsics'],float)
        if intr.shape != (4,) or not np.isfinite(intr).all() or min(intr[:2]) <= 0:
            raise ValueError(name+': uncalibrated intrinsics')
        if cam['camera_model'] != 'pinhole' or cam['distortion_model'] != 'radtan':
            raise ValueError('current software depth supports pinhole/radtan only')
        if len(cam['distortion_coeffs']) != 4 or not np.isfinite(cam['distortion_coeffs']).all():
            raise ValueError('requires four finite radtan coefficients')
        if len(cam['resolution']) != 2 or min(cam['resolution']) <= 0: raise ValueError('invalid resolution')
    if cams['cam0']['resolution'] != cams['cam1']['resolution']: raise ValueError('stereo resolutions differ')
    baseline = np.linalg.norm(np.array(cams['cam0']['T_imu_cam'])[:3,3]-np.array(cams['cam1']['T_imu_cam'])[:3,3])
    if not .005 < baseline < .5: raise ValueError('implausible baseline (metres expected)')
    for key in ('accelerometer_noise_density','accelerometer_random_walk','gyroscope_noise_density','gyroscope_random_walk','update_rate'):
        if not np.isfinite(imu[key]) or imu[key] <= 0: raise ValueError('invalid IMU '+key)
    return cfg,cams,imu
