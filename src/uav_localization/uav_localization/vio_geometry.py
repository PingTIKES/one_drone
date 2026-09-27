"""OpenVINS global-up / IMU -> PX4 arbitrary-heading FRD world / FRD body.

OpenVINS odomimu uses Hamilton IMU->global orientation, body-frame twist,
and local IMU orientation covariance. The global yaw is not true North.
"""
import numpy as np
from uav_localization.calibration import transform


def rotation(q):
    q = np.array(q,float)
    if not np.isfinite(q).all() or not .9 < np.linalg.norm(q) < 1.1: raise ValueError('invalid quaternion')
    w,x,y,z = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def quaternion(r):
    # Eigenvector extraction avoids singular special cases at 180 degrees.
    k = np.array([[r[0,0]-r[1,1]-r[2,2],r[0,1]+r[1,0],r[0,2]+r[2,0],r[2,1]-r[1,2]],
                  [r[0,1]+r[1,0],r[1,1]-r[0,0]-r[2,2],r[1,2]+r[2,1],r[0,2]-r[2,0]],
                  [r[0,2]+r[2,0],r[1,2]+r[2,1],r[2,2]-r[0,0]-r[1,1],r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1],np.trace(r)]])/3
    _,v = np.linalg.eigh(k)
    q = v[:, -1][[3,0,1,2]]
    return q if q[0]>=0 else -q


def convert(position, q, velocity, omega, pose_cov, twist_cov, t_body_imu):
    t = transform(t_body_imu)
    r_bi, p_bi = t[:3,:3],t[:3,3]  # body FLU <- IMU
    r_wi = rotation(q)
    r_wb = r_wi @ r_bi.T
    flip = np.diag([1.,-1.,-1.])
    p = flip @ (np.asarray(position)-r_wb@p_bi)
    w_b = r_bi @ omega
    v_b = r_bi @ velocity - np.cross(w_b,p_bi)
    r_fb = flip @ r_wb @ flip
    pc, vc = np.asarray(pose_cov).reshape(6,6),np.asarray(twist_cov).reshape(6,6)
    for cov in (pc,vc):
        if not np.isfinite(cov).all() or not np.allclose(cov,cov.T,atol=1e-5) or np.linalg.eigvalsh(cov).min() < -1e-7:
            raise ValueError('invalid covariance')
    # Conservative variance inflation for the IMU lever arm (unmodelled cross terms).
    lever = np.linalg.norm(p_bi)
    pv = (2 if lever else 1)*np.diag(flip@pc[:3,:3]@flip) + 2*lever*lever*np.trace(pc[3:,3:])
    ov = np.diag(flip@r_bi@pc[3:,3:]@r_bi.T@flip)
    vv = (2 if lever else 1)*np.diag(flip@r_bi@vc[:3,:3]@r_bi.T@flip) + 2*lever*lever*np.trace(vc[3:,3:])
    return p,quaternion(r_fb),flip@v_b,flip@w_b,np.maximum(pv,1e-6),np.maximum(ov,1e-6),np.maximum(vv,1e-6)
