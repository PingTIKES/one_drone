import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('mission', Path(__file__).parents[1] / 'src/mission.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
Mission, load_decisions = module.Mission, module.load_decisions


def mission(mode):
    return Mission(load_decisions({'decisions': {'test': {
        'mode': mode, 'waypoints': [[1, 2, 3], [4, 5, 6]]}}})['test'])


def test_once_finishes_only_after_last_point():
    m = mission('once')
    assert m.waypoint == [1, 2, 3]
    assert m.advance()
    assert m.waypoint == [4, 5, 6]
    assert not m.advance()
    assert m.complete and m.laps == 1


def test_loop_returns_to_first_without_completing():
    m = mission('loop')
    for _ in range(6):
        assert m.advance()
    assert m.index == 0 and m.laps == 3 and not m.complete


def test_arrival_requires_stable_low_speed_and_height():
    m = mission('once')
    def observe(t, pos=(0, 0, 2), speed=0):
        return m.observe(pos, (0, 0, 2), speed, t, .25, .2, .2, .5)
    assert not observe(0)
    assert not observe(.4)
    assert not observe(.5, speed=1)
    assert not observe(.6)
    assert not observe(1.0)
    assert observe(1.2)
    assert not observe(1.3, pos=(0, 0, 2.3))
    assert not observe(1.4)


@pytest.mark.parametrize('point', [[1, 2], [1, 2, float('nan')], [1, True, 2], [1, 2, '3']])
def test_rejects_bad_coordinates(point):
    with pytest.raises(ValueError):
        load_decisions({'decisions': {'test': {'waypoints': [point]}}})


def test_empty_route_is_valid_but_not_executed():
    assert load_decisions({'decisions': {'test': {'mode': 'loop', 'waypoints': []}}})['test']['waypoints'] == []
