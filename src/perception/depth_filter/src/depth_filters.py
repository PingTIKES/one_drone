"""Validity-aware spatial, temporal and small-hole depth filtering."""
import cv2
import numpy as np


class DepthFilters:
    def __init__(self, spatial_kernel=3, temporal_alpha=.65,
                 temporal_max_delta=.4, hole_min_neighbors=5):
        if spatial_kernel < 1 or spatial_kernel % 2 == 0:
            raise ValueError('spatial_kernel must be a positive odd integer')
        if not 0 < temporal_alpha <= 1:
            raise ValueError('temporal_alpha must be in (0, 1]')
        if temporal_max_delta <= 0 or not 0 <= hole_min_neighbors <= spatial_kernel ** 2:
            raise ValueError('invalid temporal or hole-fill parameter')
        self.kernel = int(spatial_kernel)
        self.alpha = float(temporal_alpha)
        self.max_delta = float(temporal_max_delta)
        self.min_neighbors = int(hole_min_neighbors)
        self.previous = None

    def apply(self, depth):
        depth = np.asarray(depth, dtype=np.float32)
        if depth.ndim != 2:
            raise ValueError('depth image must be two-dimensional')
        valid = np.isfinite(depth) & (depth > 0)
        values = np.where(valid, depth, 0.0)
        if self.kernel > 1:
            median = cv2.medianBlur(values, self.kernel)
            neighbor_count = cv2.boxFilter(
                valid.astype(np.float32), -1, (self.kernel, self.kernel),
                normalize=False, borderType=cv2.BORDER_REPLICATE)
            # Preserve measured pixels and fill only small, well-supported holes.
            fill = ~valid & (neighbor_count >= self.min_neighbors) & (median > 0)
            values[fill] = median[fill]
            valid |= fill
        if self.previous is not None and self.previous.shape == values.shape:
            previous_valid = np.isfinite(self.previous) & (self.previous > 0)
            stable = (valid & previous_valid &
                      (np.abs(values-self.previous) <= self.max_delta))
            values[stable] = (self.alpha * values[stable] +
                              (1.0-self.alpha) * self.previous[stable])
        result = np.where(valid, values, np.nan).astype(np.float32)
        self.previous = result.copy()
        return result
