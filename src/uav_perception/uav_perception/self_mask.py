"""Known x500 body geometry in base_link FLU metres, for sim depth self filtering.

The body box and rotor centres come from PX4 1.14.3's Gazebo x500 model.sdf.
Keep this mask disabled for real airframes until their geometry is measured.
"""

import numpy as np


def x500_external_mask(points):
    """Return True for points outside the x500 body and swept propeller disks."""
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    x, y, z = points.T
    # Body collision: 0.353553 m square, centred at z=0.007, height 0.05 m.
    # Small margins absorb depth quantisation, without creating a forward blind zone.
    on_body = ((np.abs(x) <= .187) & (np.abs(y) <= .187) &
               (z >= -.028) & (z <= .042))
    on_rotor = np.zeros(len(points), dtype=bool)
    for cx in (-.174, .174):
        for cy in (-.174, .174):
            on_rotor |= (((x - cx)**2 + (y - cy)**2 <= .15**2) &
                         (z >= .04) & (z <= .08))
    return ~(on_body | on_rotor)
