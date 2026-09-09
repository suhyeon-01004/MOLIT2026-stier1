#!/usr/bin/python3
"""Build and publish a Lanelet2 route selected by the competition RDDF."""

import argparse
import json
import math
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

from lanelet2.core import Lanelet, LaneletMap, LineString3d, Point3d
from mgeo_common import clean_points, polyline_length, resample


def element_tags(element):
    return {tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")}


def same_point(first, second, tolerance=1e-7):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second))) <= tolerance


def clean_consecutive(points):
    result = []
    for point in points:
        if not result or not same_point(result[-1], point):
            result.append(point)
    return result


def load_rddf(path):
    points = []
    for number, line in enumerate(Path(path).read_text().splitlines(), 1):
        if not line.strip():
            continue
        values = line.split()
        if len(values) != 3:
            raise ValueError(f"invalid RDDF record at line {number}")
        point = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in point):
            raise ValueError(f"non-finite RDDF record at line {number}")
        points.append(point)
    if len(points) < 3:
        raise ValueError("RDDF must contain at least three points")
    return points


def load_links(raw_mgeo):
    path = Path(raw_mgeo) / "link_set.json"
    links = json.loads(path.read_text())
    return {link["idx"]: link for link in links if not link.get("lazy_init")}


def load_lanelet2_map(osm_path, links):
    root = ET.parse(osm_path).getroot()
    point_by_id = {}
    max_id = 0
    for node in root.findall("node"):
        osm_id = int(node.attrib["id"])
        max_id = max(max_id, osm_id)
        tags = element_tags(node)
        point_by_id[node.attrib["id"]] = Point3d(
            osm_id,
            float(tags["local_x"]),
            float(tags["local_y"]),
            float(tags.get("ele", 0.0)),
        )

    way_by_id = {}
    for way in root.findall("way"):
        osm_id = int(way.attrib["id"])
        max_id = max(max_id, osm_id)
        line = LineString3d(
            osm_id,
            [point_by_id[node.attrib["ref"]] for node in way.findall("nd")],
        )
        for key, value in element_tags(way).items():
            line.attributes[key] = value
        way_by_id[way.attrib["id"]] = line

    lanelet_map = LaneletMap()
    lanelet_by_source = {}
    next_id = max_id + 1
    for relation in root.findall("relation"):
        tags = element_tags(relation)
        if tags.get("type") != "lanelet":
            continue
        members = {
            member.attrib["role"]: member.attrib["ref"]
            for member in relation.findall("member")
            if member.attrib["type"] == "way"
        }
        lanelet = Lanelet(
            int(relation.attrib["id"]),
            way_by_id[members["left"]],
            way_by_id[members["right"]],
        )
        for key, value in tags.items():
            lanelet.attributes[key] = value
        source_id = tags["source_link_id"]
        center_points = []
        for point in links[source_id]["points"]:
            center_points.append(Point3d(next_id, *map(float, point[:3])))
            next_id += 1
        lanelet.centerline = LineString3d(next_id, center_points)
        next_id += 1
        lanelet_map.add(lanelet)
        lanelet_by_source[source_id] = lanelet
    return lanelet_map, lanelet_by_source


def route_lanelet_ids(rddf, links):
    owners = defaultdict(set)
    for link_id, link in links.items():
        for point in link["points"]:
            owners[tuple(round(float(value), 8) for value in point[:3])].add(link_id)

    candidates = []
    for index, point in enumerate(rddf):
        matches = owners.get(tuple(round(value, 8) for value in point))
        if not matches:
            raise ValueError(f"RDDF point {index} is not a Lanelet centerline point")
        candidates.append(matches)

    selected = []
    for index, matches in enumerate(candidates):
        if len(matches) == 1:
            choice = next(iter(matches))
        elif selected and selected[-1] in matches:
            choice = selected[-1]
        else:
            choice = None
            for future in candidates[index + 1 :]:
                if len(future) == 1:
                    candidate = next(iter(future))
                    if candidate in matches:
                        choice = candidate
                    break
            choice = choice or sorted(matches)[0]
        selected.append(choice)

    sequence = []
    for link_id in selected:
        if not sequence or sequence[-1] != link_id:
            sequence.append(link_id)
    return sequence


def validate_topology(sequence, links):
    for current_id, next_id in zip(sequence, sequence[1:] + sequence[:1]):
        current, following = links[current_id], links[next_id]
        if current["to_node_idx"] != following["from_node_idx"]:
            raise ValueError(f"disconnected Lanelets: {current_id} -> {next_id}")


def stitch_centerlines(sequence, links):
    points = []
    for link_id in sequence:
        centerline = [tuple(map(float, point[:3])) for point in links[link_id]["points"]]
        if points and same_point(points[-1], centerline[0]):
            centerline = centerline[1:]
        points.extend(centerline)
    if not same_point(points[-1], points[0]):
        points.append(points[0])
    return points


def _lanelet_points(line):
    return [(float(point.x), float(point.y), float(point.z)) for point in line]


def _oriented(points, reference):
    points = clean_points(points)
    start = reference[0]
    forward = math.hypot(points[0][0] - start[0], points[0][1] - start[1])
    reverse = math.hypot(points[-1][0] - start[0], points[-1][1] - start[1])
    return list(reversed(points)) if reverse < forward else points


def _signed_boundary_offsets(center, left, right):
    offsets = []
    for index, (point, left_point, right_point) in enumerate(
        zip(center, left, right)
    ):
        before = center[max(0, index - 1)]
        after = center[min(len(center) - 1, index + 1)]
        dx, dy = after[0] - before[0], after[1] - before[1]
        norm = math.hypot(dx, dy) or 1.0
        normal_x, normal_y = -dy / norm, dx / norm
        left_offset = (
            (left_point[0] - point[0]) * normal_x
            + (left_point[1] - point[1]) * normal_y
        )
        right_offset = (
            (right_point[0] - point[0]) * normal_x
            + (right_point[1] - point[1]) * normal_y
        )
        offsets.append((left_offset, right_offset))
    return offsets


def _percentile(values, quantile):
    values = sorted(values)
    return values[int((len(values) - 1) * quantile)]


def _maximum_curvature(points):
    values = []
    for first, middle, last in zip(points, points[1:], points[2:]):
        first_length = math.hypot(
            middle[0] - first[0], middle[1] - first[1]
        )
        second_length = math.hypot(
            last[0] - middle[0], last[1] - middle[1]
        )
        chord = math.hypot(last[0] - first[0], last[1] - first[1])
        twice_area = abs(
            (middle[0] - first[0]) * (last[1] - first[1])
            - (middle[1] - first[1]) * (last[0] - first[0])
        )
        denominator = first_length * second_length * chord
        if denominator > 1e-9:
            values.append(2.0 * twice_area / denominator)
    return max(values) if values else 0.0


def boundary_centered_route(
    sequence,
    links,
    lanelets,
    path_spacing_m=0.5,
    vehicle_width_m=1.892,
    minimum_clearance_m=0.15,
    maximum_correction_m=0.75,
    smoothing_window_points=31,
):
    """Create a smooth route centered between the selected Lanelet2 bounds."""
    if path_spacing_m <= 0.0 or vehicle_width_m <= 0.0:
        raise ValueError("path spacing and vehicle width must be positive")
    if minimum_clearance_m < 0.0 or maximum_correction_m <= 0.0:
        raise ValueError("clearance/correction parameters are invalid")
    window = max(1, int(smoothing_window_points))
    window += 1 if window % 2 == 0 else 0

    raw, targets, limits, sources = [], [], [], []
    fallback_links = []
    half_width = vehicle_width_m / 2.0
    for link_id in sequence:
        original = clean_points(links[link_id]["points"])
        count = max(
            2, int(math.ceil(polyline_length(original) / path_spacing_m)) + 1
        )
        center = resample(original, count)
        lanelet = lanelets[link_id]
        left = resample(_oriented(_lanelet_points(lanelet.leftBound), center), count)
        right = resample(
            _oriented(_lanelet_points(lanelet.rightBound), center), count
        )
        boundary_offsets = _signed_boundary_offsets(center, left, right)
        valid = [
            left_offset > right_offset
            and left_offset - right_offset
            >= vehicle_width_m + 2.0 * minimum_clearance_m
            and abs((left_offset + right_offset) / 2.0)
            <= maximum_correction_m
            for left_offset, right_offset in boundary_offsets
        ]
        link_valid = all(valid)
        if not link_valid:
            fallback_links.append(link_id)

        skip = 1 if raw and same_point(raw[-1], center[0]) else 0
        for point, (left_offset, right_offset) in zip(
            center[skip:], boundary_offsets[skip:]
        ):
            raw.append(point)
            sources.append(link_id)
            if link_valid:
                targets.append((left_offset + right_offset) / 2.0)
                limits.append(
                    (
                        right_offset + half_width + minimum_clearance_m,
                        left_offset - half_width - minimum_clearance_m,
                        left_offset,
                        right_offset,
                    )
                )
            else:
                targets.append(0.0)
                limits.append(None)

    if not same_point(raw[-1], raw[0]):
        raw.append(raw[0])
        targets.append(targets[0])
        limits.append(limits[0])
        sources.append(sources[0])

    core, core_targets, core_limits = raw[:-1], targets[:-1], limits[:-1]
    half_window = window // 2
    smoothed = []
    for index in range(len(core)):
        weighted = [
            (
                half_window + 1 - abs(delta),
                core_targets[(index + delta) % len(core)],
            )
            for delta in range(-half_window, half_window + 1)
        ]
        offset = sum(weight * value for weight, value in weighted) / sum(
            weight for weight, _ in weighted
        )
        offset = max(-maximum_correction_m, min(maximum_correction_m, offset))
        limit = core_limits[index]
        if limit is None:
            offset = 0.0
        else:
            offset = max(limit[0], min(limit[1], offset))
        smoothed.append(offset)

    corrected = []
    for index, (point, offset) in enumerate(zip(core, smoothed)):
        before, after = core[index - 1], core[(index + 1) % len(core)]
        dx, dy = after[0] - before[0], after[1] - before[1]
        norm = math.hypot(dx, dy) or 1.0
        corrected.append(
            (point[0] - dy * offset / norm, point[1] + dx * offset / norm, point[2])
        )
    corrected.append(corrected[0])

    clearances = [
        min(left_offset - offset - half_width, offset - right_offset - half_width)
        for offset, limit in zip(smoothed, core_limits)
        if limit is not None
        for left_offset, right_offset in [(limit[2], limit[3])]
    ]
    minimum_verified_clearance = min(clearances)
    if minimum_verified_clearance + 1e-9 < minimum_clearance_m:
        raise ValueError(
            "corrected route violates minimum boundary clearance: "
            f"{minimum_verified_clearance:.3f} m"
        )
    if not all(math.isfinite(value) for point in corrected for value in point):
        raise ValueError("corrected route contains a non-finite coordinate")

    raw_curvature = _maximum_curvature(raw)
    corrected_curvature = _maximum_curvature(corrected)
    if corrected_curvature > raw_curvature + 0.01:
        raise ValueError(
            "boundary correction introduced excessive curvature: "
            f"{corrected_curvature:.3f} > {raw_curvature:.3f} 1/m"
        )
    absolute_corrections = [abs(value) for value in smoothed]
    stats = {
        "fallback_links": fallback_links,
        "minimum_clearance_m": minimum_verified_clearance,
        "maximum_correction_m": max(absolute_corrections),
        "p99_correction_m": _percentile(absolute_corrections, 0.99),
        "raw_maximum_curvature": raw_curvature,
        "corrected_maximum_curvature": corrected_curvature,
        "source_by_point": sources,
    }
    return corrected, stats


def build_route(
    rddf_path,
    osm_path,
    raw_mgeo,
    path_spacing_m=0.5,
    vehicle_width_m=1.892,
    minimum_clearance_m=0.15,
    maximum_correction_m=0.75,
    smoothing_window_points=31,
):
    rddf = load_rddf(rddf_path)
    links = load_links(raw_mgeo)
    lanelet_map, lanelet_by_source = load_lanelet2_map(osm_path, links)
    sequence = route_lanelet_ids(rddf, links)
    missing = [link_id for link_id in sequence if link_id not in lanelet_by_source]
    if missing:
        raise ValueError(f"RDDF references missing Lanelets: {missing}")
    validate_topology(sequence, links)
    raw_route = stitch_centerlines(sequence, links)

    reference = clean_consecutive(rddf)
    if len(reference) != len(raw_route):
        raise ValueError(
            f"RDDF/Lanelet point count differs: {len(reference)} != {len(raw_route)}"
        )
    errors = [
        math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))
        for first, second in zip(reference, raw_route)
    ]
    if max(errors) > 1e-6:
        raise ValueError(f"RDDF/Lanelet maximum error is {max(errors):.6f} m")
    route, stats = boundary_centered_route(
        sequence,
        links,
        lanelet_by_source,
        path_spacing_m,
        vehicle_width_m,
        minimum_clearance_m,
        maximum_correction_m,
        smoothing_window_points,
    )
    stats["maximum_rddf_error_m"] = max(errors)
    return route, sequence, len(lanelet_map.laneletLayer), stats


def path_length(points):
    return sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(points, points[1:])
    )


def validate_only(args):
    route, sequence, lanelet_count, stats = build_route(
        args.rddf,
        args.lanelet_map,
        args.raw_mgeo,
        args.path_spacing_m,
        args.vehicle_width_m,
        args.minimum_clearance_m,
        args.maximum_correction_m,
        args.smoothing_window_points,
    )
    print(
        f"OK: {len(route)} route points, {len(sequence)} route Lanelets / "
        f"{lanelet_count} map Lanelets, {path_length(route):.1f} m closed loop, "
        f"max RDDF error={stats['maximum_rddf_error_m']:.9f} m"
    )
    print(
        "boundary correction: "
        f"max={stats['maximum_correction_m']:.3f} m, "
        f"p99={stats['p99_correction_m']:.3f} m, "
        f"minimum verified clearance={stats['minimum_clearance_m']:.3f} m, "
        f"curvature={stats['raw_maximum_curvature']:.3f}->"
        f"{stats['corrected_maximum_curvature']:.3f} 1/m, "
        f"fallback Lanelets={stats['fallback_links']}"
    )
    print("route: " + " -> ".join(sequence))


def quaternion_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion

    return Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def make_path(points, frame_id, stamp):
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as PathMessage

    message = PathMessage()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    for index, point in enumerate(points):
        if index + 1 < len(points) and not same_point(point, points[index + 1]):
            dx = points[index + 1][0] - point[0]
            dy = points[index + 1][1] - point[1]
        elif index > 0:
            dx = point[0] - points[index - 1][0]
            dy = point[1] - points[index - 1][1]
        else:
            dx, dy = 1.0, 0.0
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
        pose.pose.orientation = quaternion_from_yaw(math.atan2(dy, dx))
        message.poses.append(pose)
    return message


class RoutePublisher:
    def __init__(self, route, sequence):
        import rospy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path as PathMessage

        self.rospy = rospy
        self.points = route[:-1] if same_point(route[0], route[-1]) else route
        self.sequence = sequence
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.local_length = float(rospy.get_param("~local_path_length_m", 100.0))
        self.backward_points = int(rospy.get_param("~search_backward_points", 80))
        self.forward_points = int(rospy.get_param("~search_forward_points", 500))
        self.reacquire_distance = float(rospy.get_param("~reacquire_distance_m", 10.0))
        self.previous_index = None
        self.global_publisher = rospy.Publisher(
            rospy.get_param("~global_path_topic", "/global_path"),
            PathMessage,
            queue_size=1,
            latch=True,
        )
        self.local_publisher = rospy.Publisher(
            rospy.get_param("~local_path_topic", "/local_path"),
            PathMessage,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~localization_topic", "/localization/pose"),
            PoseStamped,
            self.pose_callback,
            queue_size=10,
        )
        self.global_publisher.publish(
            make_path(self.points + [self.points[0]], self.frame_id, rospy.Time.now())
        )

    def nearest(self, x, y):
        size = len(self.points)
        if self.previous_index is None:
            candidates = range(size)
        else:
            candidates = (
                (self.previous_index + offset) % size
                for offset in range(-self.backward_points, self.forward_points + 1)
            )
        best = min(
            candidates,
            key=lambda index: (self.points[index][0] - x) ** 2
            + (self.points[index][1] - y) ** 2,
        )
        distance = math.hypot(self.points[best][0] - x, self.points[best][1] - y)
        if self.previous_index is not None and distance > self.reacquire_distance:
            best = min(
                range(size),
                key=lambda index: (self.points[index][0] - x) ** 2
                + (self.points[index][1] - y) ** 2,
            )
        return best

    def local_points(self, start):
        result = [self.points[start]]
        total = 0.0
        index = start
        while total < self.local_length and len(result) <= len(self.points):
            next_index = (index + 1) % len(self.points)
            total += math.hypot(
                self.points[next_index][0] - self.points[index][0],
                self.points[next_index][1] - self.points[index][1],
            )
            result.append(self.points[next_index])
            index = next_index
        return result

    def pose_callback(self, pose):
        if pose.header.frame_id and pose.header.frame_id != self.frame_id:
            self.rospy.logwarn_throttle(
                5.0, f"route publisher ignored pose frame '{pose.header.frame_id}'"
            )
            return
        position = pose.pose.position
        self.previous_index = self.nearest(position.x, position.y)
        points = self.local_points(self.previous_index)
        stamp = pose.header.stamp or self.rospy.Time.now()
        self.local_publisher.publish(make_path(points, self.frame_id, stamp))


def ros_main():
    import rospy

    rospy.init_node("rddf_lanelet_route")
    package = Path(__file__).resolve().parents[1]
    rddf_path = rospy.get_param("~rddf_file", str(package / "map" / "2026_molit_comp_global_path.txt"))
    osm_path = rospy.get_param("~lanelet_map", str(package / "map" / "lanelet2_map.osm"))
    raw_mgeo = rospy.get_param("~raw_mgeo", str(package / "raw_mgeo"))
    route, sequence, lanelet_count, stats = build_route(
        rddf_path,
        osm_path,
        raw_mgeo,
        float(rospy.get_param("~path_spacing_m", 0.5)),
        float(rospy.get_param("~vehicle_width_m", 1.892)),
        float(rospy.get_param("~minimum_boundary_clearance_m", 0.15)),
        float(rospy.get_param("~maximum_centerline_correction_m", 0.75)),
        int(rospy.get_param("~centerline_smoothing_window_points", 31)),
    )
    RoutePublisher(route, sequence)
    rospy.loginfo(
        "RDDF-selected boundary-centered Lanelet2 route: %d points, %d/%d Lanelets, "
        "%.1f m, max RDDF error %.9f m, correction max/p99 %.3f/%.3f m, "
        "minimum verified clearance %.3f m, fallback %s",
        len(route), len(sequence), lanelet_count, path_length(route),
        stats["maximum_rddf_error_m"], stats["maximum_correction_m"],
        stats["p99_correction_m"], stats["minimum_clearance_m"],
        stats["fallback_links"],
    )
    rospy.spin()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--rddf")
    parser.add_argument("--lanelet-map")
    parser.add_argument("--raw-mgeo")
    parser.add_argument("--path-spacing-m", type=float, default=0.5)
    parser.add_argument("--vehicle-width-m", type=float, default=1.892)
    parser.add_argument("--minimum-clearance-m", type=float, default=0.15)
    parser.add_argument("--maximum-correction-m", type=float, default=0.75)
    parser.add_argument("--smoothing-window-points", type=int, default=31)
    args, _ = parser.parse_known_args()
    if args.validate_only:
        if not all((args.rddf, args.lanelet_map, args.raw_mgeo)):
            parser.error("--validate-only requires --rddf, --lanelet-map, and --raw-mgeo")
        validate_only(args)
    else:
        ros_main()


if __name__ == "__main__":
    main()
