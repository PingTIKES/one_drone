"""Verify epipolar geometry and IMU-frame consistency using measured calibration."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('orb_calibration', ROOT / 'src/localization/orb_slam3/scripts/prepare_calibration.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

class OrbCalibrationTest(unittest.TestCase):
    def test_measured_geometry_and_timeshift(self):
        source = ROOT / 'deploy/calibration/uav1/estimator_config.yaml'
        cfg = mod.read(source)
        cameras = mod.read(source.parent / cfg['relative_config_imucam'])
        with tempfile.TemporaryDirectory() as d:
            settings, maps, offset = mod.prepare(source, ROOT / 'src/localization/orb_slam3/config/params.yaml', d)
            fs = cv2.FileStorage(settings, cv2.FILE_STORAGE_READ)
            ti = fs.getNode('IMU.T_b_c1').mat()
            krect = np.array([[fs.getNode('Camera1.fx').real(),0,fs.getNode('Camera1.cx').real()],
                              [0,fs.getNode('Camera1.fy').real(),fs.getNode('Camera1.cy').real()], [0,0,1.]])
            baseline = fs.getNode('Stereo.b').real()
            fs.release()
            ts = [np.array(cameras[c]['T_imu_cam']) for c in ['cam0','cam1']]
            t10 = np.linalg.inv(ts[1]) @ ts[0]
            ks = [np.array([[c['intrinsics'][0],0,c['intrinsics'][2]],
                            [0,c['intrinsics'][1],c['intrinsics'][3]],[0,0,1.]]) for c in cameras.values()]
            ds = [np.array(cameras[c]['distortion_coeffs']) for c in ['cam0','cam1']]
            r0,r1,*_ = cv2.stereoRectify(ks[0],ds[0],ks[1],ds[1],(640,480),t10[:3,:3],t10[:3,3],flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
            points = np.array([[.1,.1,1.5],[-.3,.2,3.],[.5,-.2,5.]])
            rect0 = points @ r0.T
            rect1 = (points @ t10[:3,:3].T + t10[:3,3]) @ r1.T
            uv0 = (rect0 @ krect.T); uv0 = uv0[:,:2] / uv0[:,2:]
            uv1 = (rect1 @ krect.T); uv1 = uv1[:,:2] / uv1[:,2:]
            np.testing.assert_allclose(uv0[:,1],uv1[:,1],atol=1e-6)
            self.assertTrue(np.all(uv0[:,0] > uv1[:,0]))
            np.testing.assert_allclose(rect0 @ ti[:3,:3].T + ti[:3,3],points @ ts[0][:3,:3].T + ts[0][:3,3],atol=1e-7)
            self.assertAlmostEqual(baseline,np.linalg.norm(t10[:3,3]),places=6)
            self.assertEqual(offset,cameras['cam0']['timeshift_cam_imu'])
            fs = cv2.FileStorage(maps,cv2.FILE_STORAGE_READ)
            self.assertEqual(fs.getNode('left_map_x').mat().shape,(480,640))
            fs.release()

    def test_reversed_stereo_rejected(self):
        source = ROOT / 'deploy/calibration/uav1/estimator_config.yaml'
        cfg = mod.read(source)
        cams = mod.read(source.parent / cfg['relative_config_imucam'])
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            cfg['relative_config_imucam']='cams.yaml';cfg['relative_config_imu']='imu.yaml'
            (root/'estimator.yaml').write_text(yaml.safe_dump(cfg))
            (root/'cams.yaml').write_text(yaml.safe_dump({'cam0':cams['cam1'],'cam1':cams['cam0']}))
            (root/'imu.yaml').write_text(yaml.safe_dump(mod.read(source.parent / mod.read(source)['relative_config_imu'])))
            with self.assertRaisesRegex(ValueError,'ordered horizontal'):
                mod.prepare(root/'estimator.yaml',ROOT/'src/localization/orb_slam3/config/params.yaml',root/'out')

if __name__ == '__main__': unittest.main()
