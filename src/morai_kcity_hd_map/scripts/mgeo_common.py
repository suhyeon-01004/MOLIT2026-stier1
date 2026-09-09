#!/usr/bin/python3
"""Small geometry helpers shared by the converter and RViz viewer."""

import json
import math
from pathlib import Path


def load_mgeo(root):
    root = Path(root)
    names = (
        "global_info",
        "link_set",
        "lane_boundary_set",
        "traffic_light_set",
        "singlecrosswalk_set",
        "crosswalk_set",
        "intersection_controller_set",
        "intersection_controller_data",
        "node_set",
        "lane_node_set",
    )
    return {
        name: json.loads((root / f"{name}.json").read_text())
        for name in names
    }


def clean_points(points):
    out = []
    for point in points:
        value = tuple(float(v) for v in point[:3])
        if len(value) == 2:
            value += (0.0,)
        if not out or distance(out[-1], value) > 1e-6:
            out.append(value)
    return out


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def polyline_length(points):
    return sum(distance(a, b) for a, b in zip(points, points[1:]))


def interpolate(a, b, ratio):
    return tuple(a[i] + (b[i] - a[i]) * ratio for i in range(3))


def resample(points, count):
    points = clean_points(points)
    if len(points) < 2:
        return points
    lengths = [0.0]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1] + distance(a, b))
    if lengths[-1] <= 1e-9:
        return [points[0]] * count
    out = []
    segment = 0
    for index in range(count):
        target = lengths[-1] * index / (count - 1)
        while (
            segment + 1 < len(lengths) - 1
            and lengths[segment + 1] < target
        ):
            segment += 1
        span = lengths[segment + 1] - lengths[segment]
        ratio = (
            0.0
            if span <= 1e-9
            else (target - lengths[segment]) / span
        )
        out.append(interpolate(points[segment], points[segment + 1], ratio))
    return out


def offset_polyline(points, offset):
    points = clean_points(points)
    out = []
    for index, point in enumerate(points):
        before = points[max(0, index - 1)]
        after = points[min(len(points) - 1, index + 1)]
        dx, dy = after[0] - before[0], after[1] - before[1]
        norm = math.hypot(dx, dy) or 1.0
        out.append(
            (
                point[0] - dy * offset / norm,
                point[1] + dx * offset / norm,
                point[2],
            )
        )
    return out


def _project(point, reference):
    best_distance = float("inf")
    best_s = 0.0
    walked = 0.0
    for a, b in zip(reference, reference[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        ratio = (
            0.0
            if length2 == 0
            else max(
                0.0,
                min(
                    1.0,
                    ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy)
                    / length2,
                ),
            )
        )
        x, y = a[0] + ratio * dx, a[1] + ratio * dy
        current = math.hypot(point[0] - x, point[1] - y)
        if current < best_distance:
            best_distance = current
            best_s = walked + math.sqrt(length2) * ratio
        walked += math.sqrt(length2)
    return best_s, best_distance


def _project_s(point, reference):
    return _project(point, reference)[0]


def _connected_boundary(point_sets, reference, tolerance=0.05):
    """Prefer an endpoint-connected trail when projection order is ambiguous."""
    if len(point_sets) < 2:
        return None
    candidates = []

    def walk(out, remaining):
        if not remaining:
            sample_count = min(64, max(2, len(reference)))
            samples = resample(out, sample_count)
            reference_samples = resample(reference, sample_count)
            score = sum(
                distance(a, b) for a, b in zip(samples, reference_samples)
            ) / sample_count
            candidates.append((score, out))
            return
        for position, points in enumerate(remaining):
            rest = remaining[:position] + remaining[position + 1 :]
            if distance(out[-1], points[0]) <= tolerance:
                walk(out + points[1:], rest)
            if distance(out[-1], points[-1]) <= tolerance:
                walk(out + list(reversed(points[:-1])), rest)

    for position, points in enumerate(point_sets):
        rest = point_sets[:position] + point_sets[position + 1 :]
        walk(list(points), rest)
        walk(list(reversed(points)), rest)
    return min(candidates, key=lambda item: item[0])[1] if candidates else None


def stitch_boundary(boundary_ids, boundaries, reference):
    reference = clean_points(reference)
    point_sets = []
    for boundary_id in boundary_ids:
        boundary = boundaries.get(boundary_id)
        if not boundary:
            continue
        points = clean_points(boundary["points"])
        if len(points) >= 2:
            point_sets.append(points)
    connected = _connected_boundary(point_sets, reference)
    if connected:
        return clean_points(connected)
    pieces = []
    for points in point_sets:
        start_s = _project_s(points[0], reference)
        end_s = _project_s(points[-1], reference)
        if end_s < start_s:
            points.reverse()
            start_s, end_s = end_s, start_s
        pieces.append((start_s, end_s, points))
    pieces.sort(key=lambda item: (item[0], item[1]))
    out = []
    for _, _, points in pieces:
        if out and distance(out[-1], points[0]) < 0.05:
            out.extend(points[1:])
        else:
            out.extend(points)
    return clean_points(out)


def _usable_boundary(points, center, width):
    if len(points) < 2:
        return False
    projected = [_project(point, center) for point in points]
    near = [
        (along, lateral)
        for along, lateral in projected
        if lateral <= max(10.0, width * 3.0)
    ]
    if len(near) < 2:
        return False
    center_length = polyline_length(center)
    coverage = (
        max(value[0] for value in near) - min(value[0] for value in near)
    ) / max(center_length, 1e-9)
    lateral = sorted(value[1] for value in projected)
    length_ratio = polyline_length(points) / max(center_length, 1e-9)
    endpoint_limit = max(15.0, width * 4.0)
    return (
        coverage >= 0.6
        and 0.7 <= length_ratio <= 1.5
        and distance(points[0], center[0]) <= endpoint_limit
        and distance(points[-1], center[-1]) <= endpoint_limit
        and lateral[int(0.95 * (len(lateral) - 1))]
        <= max(12.0, width * 3.0)
    )


def _crosses(a, b, c, d, epsilon=1e-8):
    if (
        max(a[0], b[0]) < min(c[0], d[0])
        or max(c[0], d[0]) < min(a[0], b[0])
        or max(a[1], b[1]) < min(c[1], d[1])
        or max(c[1], d[1]) < min(a[1], b[1])
    ):
        return False

    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (
            q[1] - p[1]
        ) * (r[0] - p[0])

    values = side(a, b, c), side(a, b, d), side(c, d, a), side(c, d, b)
    return (
        values[0] * values[1] < -epsilon
        and values[2] * values[3] < -epsilon
    )


def self_intersects(points):
    segments = list(zip(points, points[1:]))
    return any(
        _crosses(a, b, c, d)
        for index, (a, b) in enumerate(segments)
        for c, d in segments[index + 2 :]
    )


def polylines_cross(first, second):
    return any(
        _crosses(a, b, c, d)
        for a, b in zip(first, first[1:])
        for c, d in zip(second, second[1:])
    )


def end_connectors_cross(left, right):
    connectors = ((left[0], right[0]), (left[-1], right[-1]))
    return any(
        _crosses(a, b, c, d)
        for a, b in connectors
        for boundary in (left, right)
        for c, d in zip(boundary, boundary[1:])
    )


def lane_bounds(link, boundaries):
    center = clean_points(link["points"])
    width_start = float(link.get("width_start") or 3.5)
    width_end = float(link.get("width_end") or width_start)
    width = (width_start + width_end) / 2.0
    left = stitch_boundary(link.get("lane_mark_left", []), boundaries, center)
    right = stitch_boundary(link.get("lane_mark_right", []), boundaries, center)
    left_real = _usable_boundary(left, center, width)
    right_real = _usable_boundary(right, center, width)
    if not left_real:
        left = offset_polyline(center, width / 2.0)
    if not right_real:
        right = offset_polyline(center, -width / 2.0)
    if end_connectors_cross(left, right):
        left_gap = max(
            distance(left[0], center[0]), distance(left[-1], center[-1])
        )
        right_gap = max(
            distance(right[0], center[0]), distance(right[-1], center[-1])
        )
        if left_gap >= right_gap:
            left, left_real = offset_polyline(center, width / 2.0), False
        else:
            right, right_real = offset_polyline(center, -width / 2.0), False
    return left, right, left_real, right_real


def dashed_segments(points, dash, gap):
    points = clean_points(points)
    dash = float(dash) if dash and dash > 0 else 3.0
    gap = float(gap) if gap and gap > 0 else 3.0
    total = polyline_length(points)
    if total <= 1e-9:
        return []
    samples = resample(points, max(2, int(total / 0.25) + 1))
    out = []
    walked = 0.0
    for a, b in zip(samples, samples[1:]):
        segment = distance(a, b)
        if (walked % (dash + gap)) < dash:
            out.append((a, b))
        walked += segment
    return out


def triangle_strip(left, right, count=24):
    left, right = resample(left, count), resample(right, count)
    triangles = []
    for l0, l1, r0, r1 in zip(left, left[1:], right, right[1:]):
        triangles.extend((l0, r0, l1, l1, r0, r1))
    return triangles


def polygon_outline(points):
    """Return LINE_LIST points for one closed polygon."""
    points = clean_points(points)
    if len(points) > 1 and distance(points[0], points[-1]) <= 1e-6:
        points = points[:-1]
    if len(points) < 2:
        return []
    closed = points + [points[0]]
    return [value for pair in zip(closed, closed[1:]) for value in pair]


def _signed_area(points):
    return 0.5 * sum(
        a[0] * b[1] - b[0] * a[1]
        for a, b in zip(points, points[1:] + points[:1])
    )


def _turn(a, b, c):
    return (b[0] - a[0]) * (c[1] - b[1]) - (
        b[1] - a[1]
    ) * (c[0] - b[0])


def _inside_triangle(point, a, b, c, orientation, epsilon=1e-9):
    return all(
        orientation * _turn(first, second, point) >= -epsilon
        for first, second in ((a, b), (b, c), (c, a))
    )


def triangulate_polygon(points):
    """Triangulate a simple concave MGeo polygon with ear clipping."""
    vertices = clean_points(points)
    if len(vertices) > 1 and distance(vertices[0], vertices[-1]) <= 1e-6:
        vertices = vertices[:-1]
    changed = True
    while changed and len(vertices) > 3:
        changed = False
        for index in range(len(vertices)):
            if abs(
                _turn(
                    vertices[index - 1],
                    vertices[index],
                    vertices[(index + 1) % len(vertices)],
                )
            ) <= 1e-9:
                vertices.pop(index)
                changed = True
                break
    if len(vertices) < 3:
        return []
    orientation = 1.0 if _signed_area(vertices) > 0.0 else -1.0
    remaining = list(range(len(vertices)))
    triangles = []
    while len(remaining) > 3:
        ear_found = False
        for position, current in enumerate(remaining):
            previous = remaining[position - 1]
            following = remaining[(position + 1) % len(remaining)]
            a, b, c = vertices[previous], vertices[current], vertices[following]
            if orientation * _turn(a, b, c) <= 1e-9:
                continue
            if any(
                _inside_triangle(vertices[index], a, b, c, orientation)
                for index in remaining
                if index not in (previous, current, following)
            ):
                continue
            # RViz lights TRIANGLE_LIST faces according to vertex winding.
            # Always emit +Z normals so white road paint does not render black.
            triangles.extend(
                (a, b, c) if orientation > 0.0 else (a, c, b)
            )
            remaining.pop(position)
            ear_found = True
            break
        if not ear_found:
            return []
    final = [vertices[index] for index in remaining]
    if _turn(*final) < 0.0:
        final[1], final[2] = final[2], final[1]
    triangles.extend(final)
    return triangles


def is_high_speed(link):
    return (
        not link.get("lazy_init")
        and float(link.get("max_speed") or 0) >= 80.0
    )
