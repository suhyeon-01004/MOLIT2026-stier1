#!/usr/bin/python3
"""Publish the original K-City MGeo features as clean RViz layers."""

import argparse
from pathlib import Path

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from crosswalk_geometry import crosswalk_stripes
from mgeo_common import (
    clean_points,
    dashed_segments,
    is_high_speed,
    is_pedestrian_crosswalk,
    load_mgeo,
    offset_polyline,
    polygon_outline,
)


COLORS = {
    "white": (0.95, 0.95, 0.95, 1.0),
    "yellow": (1.0, 0.75, 0.05, 1.0),
    "red": (1.0, 0.08, 0.08, 1.0),
    "green": (0.1, 1.0, 0.2, 0.9),
}


def point(value, z_offset=0.0):
    return Point(x=value[0], y=value[1], z=value[2] + z_offset)


def color(value):
    return ColorRGBA(*value)


class Markers:
    def __init__(self):
        self.array = MarkerArray()
        self.next_id = 0

    def add(self, namespace, marker_type, points, rgba, scale=0.15,
            z_offset=0.0):
        if not points:
            return None
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = self.next_id
        self.next_id += 1
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color = color(rgba)
        marker.points = [point(item, z_offset) for item in points]
        self.array.markers.append(marker)
        return marker


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--input",
        default=str(Path(__file__).resolve().parents[1] / "raw_mgeo"),
    )
    args, _ = parser.parse_known_args(rospy.myargv()[1:])
    rospy.init_node("kcity_hd_map_visualizer")
    data = load_mgeo(args.input)
    boundaries = {item["idx"]: item for item in data["lane_boundary_set"]}
    links = [item for item in data["link_set"] if not item.get("lazy_init")]
    links_by_id = {item["idx"]: item for item in links}
    markers = Markers()

    line_groups = {}
    for boundary in boundaries.values():
        points = clean_points(boundary["points"])
        if len(points) < 2:
            continue
        lane_type = int(boundary.get("lane_type", [0])[0])
        if lane_type == 530:
            key = ("stop_lines", "red", 0.5, 0.16)
            segments = list(zip(points, points[1:]))
        else:
            lane_color = boundary.get("lane_color", ["white"])[0]
            shape = boundary.get("lane_shape", ["solid"])[0]
            dashed = "broken" in shape and "solid" not in shape
            key = (
                "lane_boundary_dashed" if dashed else "lane_boundary_solid",
                lane_color,
                max(0.08, float(boundary.get("lane_width") or 0.15)),
                0.10,
            )
            segments = (
                dashed_segments(
                    points,
                    boundary.get("dash_interval_L1"),
                    boundary.get("dash_interval_L2"),
                )
                if dashed
                else list(zip(points, points[1:]))
            )
        line_groups.setdefault(key, []).extend(
            value for pair in segments for value in pair
        )
    for (namespace, lane_color, width, z_offset), points in line_groups.items():
        markers.add(
            f"{namespace}_{lane_color}",
            Marker.LINE_LIST,
            points,
            COLORS.get(lane_color, COLORS["white"]),
            width,
            z_offset,
        )

    regular_centers, high_centers = [], []
    for link in links:
        center = clean_points(link["points"])
        target = high_centers if is_high_speed(link) else regular_centers
        target.extend(value for pair in zip(center, center[1:]) for value in pair)
        middle = center[len(center) // 2]
        for side, allowed in (
            (1, link.get("can_move_left_lane")),
            (-1, link.get("can_move_right_lane")),
        ):
            if allowed:
                sample = [
                    center[max(0, len(center) // 2 - 1)],
                    middle,
                    center[min(len(center) - 1, len(center) // 2 + 1)],
                ]
                destination = offset_polyline(sample, side * 2.0)[1]
                marker = markers.add(
                    "lane_change_allowed",
                    Marker.ARROW,
                    [middle, destination],
                    COLORS["green"],
                    0.18,
                    0.12,
                )
                marker.scale.y = marker.scale.z = 0.4

    # Filled lane polygons were deliberately removed. Hundreds of overlapping
    # transparent link triangles caused dark wedges and blue/magenta boxes in
    # RViz. Boundaries and centerlines carry the same map semantics cleanly.
    markers.add(
        "regular_centerlines_60kmh",
        Marker.LINE_LIST,
        regular_centers,
        (0.2, 0.6, 1.0, 0.85),
        0.08,
        0.06,
    )
    markers.add(
        "high_speed_centerlines_unlimited",
        Marker.LINE_LIST,
        high_centers,
        (1.0, 0.1, 0.75, 1.0),
        0.16,
        0.07,
    )

    crosswalk_lines, other_marking_lines = [], []
    crosswalk_polygons = {marking["idx"]: clean_points(marking["points"])
                          for marking in data["singlecrosswalk_set"]
                          if is_pedestrian_crosswalk(marking)}
    crosswalk_count = 0
    for marking in data["singlecrosswalk_set"]:
        points = clean_points(marking["points"])
        if is_pedestrian_crosswalk(marking):
            road = next(
                (
                    links_by_id.get(link_id)
                    for link_id in marking.get("link_id_list", [])
                    if link_id in links_by_id
                ),
                None,
            )
            stripes, _ = crosswalk_stripes(
                points, road["points"] if road else None,
                exclude_polygons=[polygon for idx, polygon in crosswalk_polygons.items()
                                  if idx != marking["idx"]],
            )
            if not stripes:
                rospy.logwarn("could not draw crosswalk %s", marking["idx"])
                continue
            crosswalk_lines.extend(stripes)
            crosswalk_count += 1
        else:
            other_marking_lines.extend(polygon_outline(points))
    markers.add(
        "crosswalks",
        Marker.LINE_LIST,
        crosswalk_lines,
        (1.0, 1.0, 1.0, 1.0),
        0.42,
        0.20,
    )
    markers.add(
        "other_surface_markings",
        Marker.LINE_LIST,
        other_marking_lines,
        (0.65, 0.9, 1.0, 1.0),
        0.12,
        0.15,
    )

    light_points = [
        tuple(float(value) for value in light["point"])
        for light in data["traffic_light_set"]
    ]
    light_marker = markers.add(
        "traffic_lights",
        Marker.SPHERE_LIST,
        light_points,
        (1.0, 0.35, 0.05, 1.0),
        0.7,
        0.25,
    )
    light_marker.scale.y = light_marker.scale.z = 0.7

    publisher = rospy.Publisher(
        "/kcity_hd_map/markers", MarkerArray, queue_size=1, latch=True
    )
    rospy.sleep(0.5)
    publisher.publish(markers.array)
    rospy.loginfo(
        "K-City HD map: %d markers, %d links, %d boundaries, %d crosswalk polygons",
        len(markers.array.markers),
        len(links),
        len(boundaries),
        crosswalk_count,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
