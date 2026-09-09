#!/usr/bin/python3
"""Read-only checks; no ROS master, UDP sender or simulator is started."""
import math
from pathlib import Path

from rddf_lanelet_route_node import build_route, map_speed_envelope


def main():
    points = [(float(i), 0., 0.) for i in range(1000)]
    flags = [True] * 1000
    assert map_speed_envelope(points, flags, 100, 100)[1:] == (100 / 3.6, 100 / 3.6)
    assert map_speed_envelope(points, [False] * 1000, 100, 100)[1:] == (60 / 3.6, 59 / 3.6)
    flags[250:500] = [False] * 250
    values = [map_speed_envelope(points, flags, i, 100)[2] for i in (70, 100, 150, 200, 230, 250)]
    assert all(a >= b for a, b in zip(values, values[1:])), values
    assert values[-2] == values[-1] == 59 / 3.6
    assert abs(values[1] ** 2 - ((59 / 3.6) ** 2 + 2 * 1.7 * 130)) < 1e-9
    assert map_speed_envelope(points, flags, 500, 100)[1] == 60 / 3.6
    assert map_speed_envelope(points, flags, 505, 100)[1] == 100 / 3.6
    assert map_speed_envelope(points, flags, 245, 100)[1] == 60 / 3.6
    # Seam lookup wraps, so an upcoming low-speed section is not missed.
    circle = [(math.cos(i * math.tau / 1000) * 160, math.sin(i * math.tau / 1000) * 160, 0.) for i in range(1000)]
    wrapped = [False] * 100 + [True] * 900
    assert map_speed_envelope(circle, wrapped, 990, 100)[2] == 59 / 3.6
    for invalid in (float('nan'), float('inf'), 59, 101):
        try:
            map_speed_envelope(points, flags, 0, invalid)
        except ValueError:
            pass
        else:
            raise AssertionError('invalid speed accepted')
    package = Path(__file__).resolve().parents[1]
    route, sequence, count, stats = build_route(
        package / 'map/2026_molit_comp_global_path.txt',
        package / 'map/lanelet2_map.osm', package / 'raw_mgeo')
    points, flags = route[:-1], stats['high_speed_by_point'][:-1]
    assert len(points) == len(flags) and any(flags) and not all(flags)
    peak = 0.
    for i in range(len(points)):
        high, limit, target = map_speed_envelope(points, flags, i, 100)
        assert 0 <= target <= limit <= 100 / 3.6
        if not flags[i]:
            assert limit <= 60 / 3.6 and target <= 59 / 3.6
        peak = max(peak, target * 3.6)
    print('PASS: ordinary/high-speed/advance-braking/body-margin/wrap/invalid-input checks')
    print(f'PASS: {len(points)} actual route points, {len(sequence)}/{count} Lanelets; map-only peak={peak:.3f} km/h')
    print('NOT a simulation result: curvature and physical tracking can lower actual speed.')


if __name__ == '__main__':
    main()
