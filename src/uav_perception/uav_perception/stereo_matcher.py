"""Rectified stereo depth, metric scale comes exclusively from calibration."""
import cv2
import numpy as np
from uav_localization.calibration import validate_config, transform


class StereoMatcher:
    def __init__(self, config, t_body_imu, disparities=96, texture_std_min=1.0):
        _, cams, _ = validate_config(config)
        t0,t1 = (transform(cams[c]['T_imu_cam']) for c in ('cam0','cam1'))
        # Choose left camera geometrically, not by an assumed device stream name.
        right_in_zero = (np.linalg.inv(t0)@t1)[:3,3]
        self.order = ('cam0','cam1') if right_in_zero[0]>0 else ('cam1','cam0')
        left,right = [cams[k] for k in self.order]
        tl,tr = [transform(c['T_imu_cam']) for c in (left,right)]
        relative = np.linalg.inv(tr)@tl
        self.size = tuple(left['resolution'])
        def k(c):
            fx,fy,cx,cy = c['intrinsics']
            return np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
        kl,kr = k(left),k(right)
        dl,dr = np.array(left['distortion_coeffs']),np.array(right['distortion_coeffs'])
        r1,r2,p1,p2,q,_,_ = cv2.stereoRectify(kl,dl,kr,dr,self.size,relative[:3,:3],relative[:3,3],flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
        self.baseline = -p2[0,3]/p2[0,0]
        if self.baseline<=0 or abs(p2[1,3])>1e-5: raise ValueError('requires horizontal stereo pair')
        self.p = p1
        self.body_optical = transform(t_body_imu)@tl
        self.body_optical[:3,:3] = self.body_optical[:3,:3]@r1.T
        self.maps = [cv2.initUndistortRectifyMap(k,d,r,p,self.size,cv2.CV_32FC1) for k,d,r,p in ((kl,dl,r1,p1),(kr,dr,r2,p2))]
        if disparities<=0 or disparities%16: raise ValueError('disparities must be positive multiple of 16')
        if texture_std_min < 0: raise ValueError('texture_std_min must be nonnegative')
        self.texture_std_min = float(texture_std_min)
        self.matcher = cv2.StereoSGBM_create(minDisparity=0,numDisparities=disparities,blockSize=5,
                  P1=8*25,P2=32*25,disp12MaxDiff=1,uniquenessRatio=15,speckleWindowSize=80,
                  speckleRange=2,mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    def depth(self, image0,image1):
        images = {'cam0':image0,'cam1':image1}
        pair = []
        for key,(mx,my) in zip(self.order,self.maps):
            image = images[key]
            if image.shape != self.size[::-1]: raise ValueError('image resolution differs from calibration')
            pair.append(cv2.remap(image,mx,my,cv2.INTER_LINEAR))
        disparity = self.matcher.compute(*pair).astype(np.float32)/16.
        depth = np.full(disparity.shape,np.nan,np.float32)
        valid = disparity>0
        if self.texture_std_min:
            # Horizontal disparity is unobservable in a flat image row. Gazebo's
            # nearly constant infrared background can otherwise create a crisp
            # but false near-depth band across hundreds of pixels.
            image = pair[0].astype(np.float32)
            mean = cv2.boxFilter(image,-1,(7,1))
            variance = cv2.boxFilter(image*image,-1,(7,1))-mean*mean
            valid &= variance >= self.texture_std_min**2
        depth[valid] = self.p[0,0]*self.baseline/disparity[valid]
        return depth
