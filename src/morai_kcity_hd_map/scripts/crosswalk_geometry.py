#!/usr/bin/python3
"""Geometry for drawing recognizable zebra crossings from MGeo outlines."""

import math

from mgeo_common import clean_points, distance


def _nearest_direction(point, polyline):
    best = None
    for a, b in zip(polyline, polyline[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        if length2 <= 1e-9:
            continue
        ratio = max(
            0.0,
            min(
                1.0,
                ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy)
                / length2,
            ),
        )
        x, y = a[0] + ratio * dx, a[1] + ratio * dy
        candidate = ((point[0] - x) ** 2 + (point[1] - y) ** 2, dx, dy)
        if best is None or candidate[0] < best[0]:
            best = candidate
    if best is None:
        return None
    norm = math.hypot(best[1], best[2])
    return best[1] / norm, best[2] / norm


def crosswalk_stripes(points, road_points=None, spacing=0.75, width=0.42):
    """Return clipped zebra bars, using the linked road to resolve their axis."""
    polygon = clean_points(points)
    if len(polygon) > 1 and distance(polygon[0], polygon[-1]) <= 1e-6:
        polygon = polygon[:-1]
    if len(polygon) < 3:
        return [], width

    cx = sum(point[0] for point in polygon) / len(polygon)
    cy = sum(point[1] for point in polygon) / len(polygon)
    z = sum(point[2] for point in polygon) / len(polygon)
    centered = [(point[0] - cx, point[1] - cy) for point in polygon]

    # PCA visibly tilts outlines with unevenly distributed intermediate points.
    # A minimum-area bounding box always uses an actual polygon-edge direction.
    best = None
    closed = centered + [centered[0]]
    for a, b in zip(closed, closed[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        norm = math.hypot(dx, dy)
        if norm <= 1e-9:
            continue
        ex, ey = dx / norm, dy / norm
        nx, ny = -ey, ex
        along = [x * ex + y * ey for x, y in centered]
        across = [x * nx + y * ny for x, y in centered]
        span_along = max(along) - min(along)
        span_across = max(across) - min(across)
        candidate = (
            span_along * span_across,
            ex,
            ey,
            span_along,
            span_across,
        )
        if best is None or candidate[0] < best[0]:
            best = candidate
    if best is None:
        return [], width

    _, ex, ey, span_along, span_across = best
    road_direction = _nearest_direction(
        (cx, cy), clean_points(road_points or [])
    )
    if road_direction is not None:
        rx, ry = road_direction
        if abs(ex * rx + ey * ry) >= abs(-ey * rx + ex * ry):
            ux, uy = ex, ey
        else:
            ux, uy = -ey, ex
    elif span_along <= span_across:
        ux, uy = ex, ey
    else:
        ux, uy = -ey, ex
    vx, vy = -uy, ux

    local = [
        (x * ux + y * uy, x * vx + y * vy) for x, y in centered
    ]
    minimum = min(value[1] for value in local)
    maximum = max(value[1] for value in local)
    count = max(2, int((maximum - minimum) / spacing))
    step = (maximum - minimum) / count
    stripe_width = min(width, step * 0.62)
    segments = []
    for index in range(count):
        transverse = minimum + (index + 0.5) * step
        intersections = []
        closed = local + [local[0]]
        for a, b in zip(closed, closed[1:]):
            if abs(b[1] - a[1]) <= 1e-9:
                continue
            if not min(a[1], b[1]) <= transverse < max(a[1], b[1]):
                continue
            ratio = (transverse - a[1]) / (b[1] - a[1])
            intersections.append(a[0] + ratio * (b[0] - a[0]))
        intersections.sort()
        for start, end in zip(intersections[::2], intersections[1::2]):
            if end - start <= 0.05:
                continue
            segments.extend(
                (
                    (
                        cx + ux * start + vx * transverse,
                        cy + uy * start + vy * transverse,
                        z,
                    ),
                    (
                        cx + ux * end + vx * transverse,
                        cy + uy * end + vy * transverse,
                        z,
                    ),
                )
            )
    return segments, stripe_width
