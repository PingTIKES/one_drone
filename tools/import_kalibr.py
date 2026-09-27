#!/usr/bin/env python3
"""Import measured Kalibr results into a validated OpenVINS hardware bundle."""
import argparse
import copy
import tempfile
import shutil
from pathlib import Path
import sys
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/uav_localization'))
from uav_localization.calibration import read_yaml, transform, validate_config, write_opencv_yaml


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--camchain',required=True,help='Kalibr camchain-imucam YAML with T_cam_imu')
    p.add_argument('--imu',required=True,help='Measured IMU noise YAML, imu0 mapping or bare Kalibr mapping')
    p.add_argument('--body',required=True,help='YAML containing measured T_body_imu (body FLU <- camera IMU)')
    p.add_argument('--output',required=True)
    p.add_argument('--uav-id',type=int,default=1)
    a = p.parse_args()
    cameras = read_yaml(a.camchain)
    imu = read_yaml(a.imu)
    imu = imu.get('imu0',imu)
    body = read_yaml(a.body)
    t_body_imu = transform(body['T_body_imu'])
    template = ROOT/'src/uav_localization/config/openvins_sim'
    cfg = read_yaml(template/'estimator_config.yaml')
    cfg.update(num_opencv_threads=2,record_timing_information=True,
               record_timing_filepath=f'/tmp/uav{a.uav_id}_ov_timing.txt')
    out_cams = {}
    shifts = []
    for name in ('cam0','cam1'):
        src = cameras[name]
        c = {k: copy.deepcopy(src[k]) for k in ('camera_model','distortion_model','distortion_coeffs','intrinsics','resolution')}
        # Kalibr T_cam_imu is IMU -> camera. OpenVINS expects its INVERSE.
        c['T_imu_cam'] = np.linalg.inv(transform(src['T_cam_imu'])).tolist()
        shift = float(src['timeshift_cam_imu'])
        if not np.isfinite(shift) or abs(shift)>.5: raise ValueError('check timestamp units/time offset')
        c['timeshift_cam_imu'] = shift
        shifts.append(shift)
        c['cam_overlaps'] = [1 if name=='cam0' else 0]
        c['rostopic'] = f'/uav{a.uav_id}/{name}/image_raw'
        out_cams[name] = c
    if abs(shifts[0]-shifts[1])>.002:
        raise ValueError('OpenVINS uses one stereo time offset; fix camera synchronization first')
    noise = read_yaml(template/'kalibr_imu_chain.yaml')['imu0']
    for key in ('accelerometer_noise_density','accelerometer_random_walk','gyroscope_noise_density','gyroscope_random_walk','update_rate'):
        noise[key] = float(imu[key])
    noise['rostopic'] = f'/uav{a.uav_id}/imu0'
    out = Path(a.output)
    if out.exists():raise FileExistsError(f'refusing to overwrite {out}')
    mounting={'T_body_imu':t_body_imu.tolist(),'source':'measured Kalibr + body mounting'}
    if body.get('T_body_depth') is not None:mounting['T_body_depth']=transform(body['T_body_depth']).tolist()
    with tempfile.TemporaryDirectory() as directory:
        staged=Path(directory)
        for name,value in [('estimator_config.yaml',cfg),('kalibr_imucam_chain.yaml',out_cams),('kalibr_imu_chain.yaml',{'imu0':noise})]:
            write_opencv_yaml(staged/name,value)
        (staged/'body.yaml').write_text(yaml.safe_dump(mounting),encoding='utf-8')
        validate_config(staged/'estimator_config.yaml')
        shutil.copytree(staged,out)
    print('Validated:',out/'estimator_config.yaml')
    print('Use RAW images with this calibration; independent RGB requires separate calibration.')


if __name__ == '__main__': main()
