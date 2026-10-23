"""Waypoint configuration and progression, independent of ROS."""
import math


def load_decisions(data):
    decisions = data.get('decisions') if isinstance(data, dict) else None
    if not isinstance(decisions, dict) or not decisions:
        raise ValueError('decisions must be a nonempty mapping')
    result = {}
    for name, item in decisions.items():
        if not isinstance(item, dict):
            raise ValueError(f'{name}: expected a mapping')
        mode = item.get('mode', 'once')
        frame = item.get('frame_id', 'map')
        points = item.get('waypoints', [])
        if mode not in ('once', 'loop') or frame not in ('map', 'odom'):
            raise ValueError(f'{name}: mode must be once/loop; frame_id must be map/odom')
        if not isinstance(points, list):
            raise ValueError(f'{name}: waypoints must be a list')
        for point in points:
            if (not isinstance(point, list) or len(point) != 3 or
                    any(isinstance(v, bool) or not isinstance(v, (float, int)) or
                        not math.isfinite(v) for v in point)):
                raise ValueError(f'{name}: each waypoint must contain three finite numbers')
        result[name] = dict(mode=mode, frame_id=frame, waypoints=points)
    return result


class Mission:
    def __init__(self, definition):
        self.definition = definition
        self.index = 0
        self.laps = 0
        self.complete = False
        self.arrival_since = None

    @property
    def waypoint(self):
        return self.definition['waypoints'][self.index]

    def observe(self, position, target, speed, now, xy, z, max_speed, dwell):
        inside = (math.hypot(position[0] - target[0], position[1] - target[1]) <= xy
                  and abs(position[2] - target[2]) <= z and speed <= max_speed)
        if not inside:
            self.arrival_since = None
            return False
        if self.arrival_since is None or now < self.arrival_since:
            self.arrival_since = now
        return now - self.arrival_since >= dwell

    def advance(self):
        self.arrival_since = None
        self.index += 1
        if self.index == len(self.definition['waypoints']):
            self.laps += 1
            if self.definition['mode'] == 'loop':
                self.index = 0
            else:
                self.complete = True
        return not self.complete
