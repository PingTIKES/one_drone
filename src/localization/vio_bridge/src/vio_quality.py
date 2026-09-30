"""Low-cost quality signals around OpenVINS without changing its estimator."""
import math

import numpy as np


def normalized_covariance(position, orientation, velocity, limits):
    """Return the largest covariance-to-hard-limit ratio."""
    values = tuple(float(v) for group in (position, orientation, velocity)
                   for v in group)
    bounds = (float(limits[0]),) * 3 + (float(limits[1]),) * 3 + (float(limits[2]),) * 3
    if (not all(math.isfinite(v) and v >= 0 for v in values) or
            not all(math.isfinite(v) and v > 0 for v in bounds)):
        raise ValueError('invalid covariance or limit')
    return max(v / bound for v, bound in zip(values, bounds))


def feature_confidence(count, bad_count, good_count):
    """Map a measured image feature count to [0, 1]."""
    if not all(math.isfinite(float(v)) for v in (count, bad_count, good_count)):
        raise ValueError('nonfinite feature threshold')
    if count < 0 or bad_count < 0 or good_count <= bad_count:
        raise ValueError('invalid feature threshold')
    return min(1.0, max(0.0, (count - bad_count) / (good_count - bad_count)))


def quality_state(covariance_ratio, feature_count, feature_bad, feature_good,
                  acceleration, acceleration_warn, angular_rate, angular_rate_warn):
    """Return confidence and soft-degradation reasons for one accepted pose."""
    values = (covariance_ratio, acceleration, acceleration_warn,
              angular_rate, angular_rate_warn)
    if (not all(math.isfinite(float(v)) for v in values) or
            covariance_ratio < 0 or acceleration < 0 or angular_rate < 0 or
            acceleration_warn <= 0 or angular_rate_warn <= 0):
        raise ValueError('invalid VIO quality input')
    reasons = []
    covariance_confidence = min(1.0, max(0.0, 1.0 - covariance_ratio))
    if covariance_ratio >= 0.5:
        reasons.append('COVARIANCE_HIGH')
    confidence = covariance_confidence
    if feature_count is not None:
        feature_score = feature_confidence(feature_count, feature_bad, feature_good)
        confidence = min(confidence, feature_score)
        if feature_count < feature_good:
            reasons.append('FEATURES_LOW')
    if acceleration >= acceleration_warn:
        reasons.append('VELOCITY_CHANGE_HIGH')
        confidence = min(confidence, acceleration_warn / max(acceleration, 1e-9))
    if angular_rate >= angular_rate_warn:
        reasons.append('ANGULAR_RATE_HIGH')
        confidence = min(confidence, angular_rate_warn / max(angular_rate, 1e-9))
    return confidence, reasons
