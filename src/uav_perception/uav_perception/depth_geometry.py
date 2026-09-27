"""Depth decoding independent of ROS, including row padding and byte order."""
import numpy as np


def decode_depth(data, height, width, step, encoding, bigendian=False, scale=.001):
    if encoding not in ('32FC1', '16UC1'):
        raise ValueError('depth must be 32FC1 metres or scaled 16UC1')
    dtype = np.dtype(('>' if bigendian else '<') + ('f4' if encoding == '32FC1' else 'u2'))
    if min(height,width) <= 0 or step < width*dtype.itemsize or len(data) < height*step:
        raise ValueError('invalid depth dimensions/stride/buffer')
    if not np.isfinite(scale) or scale <= 0: raise ValueError('invalid depth scale')
    image = np.ndarray((height,width), dtype=dtype, buffer=data, strides=(step,dtype.itemsize))
    metres = image.astype(np.float32)
    if encoding == '16UC1': metres *= scale
    metres[(metres <= 0) | ~np.isfinite(metres)] = np.nan
    return metres
