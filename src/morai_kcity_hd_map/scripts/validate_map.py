#!/usr/bin/python3
"""One runnable integrity check for the generated map and RViz geometry."""

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from crosswalk_geometry import crosswalk_stripes
from mgeo_common import (
    clean_points,
    end_connectors_cross,
    is_high_speed,
    is_pedestrian_crosswalk,
    load_mgeo,
    polylines_cross,
    self_intersects,
    triangulate_polygon,
)


def tags(element):
    return {
        tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")
    }


def polygon_area(points):
    if len(points) > 1 and points[0] == points[-1]:
        points = points[:-1]
    return abs(
        0.5
        * sum(
            a[0] * b[1] - b[0] * a[1]
            for a, b in zip(points, points[1:] + points[:1])
        )
    )


def triangle_area(points):
    return sum(
        abs(
            (b[0] - a[0]) * (c[1] - a[1])
            - (b[1] - a[1]) * (c[0] - a[0])
        )
        / 2.0
        for a, b, c in zip(points[::3], points[1::3], points[2::3])
    )


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default=str(root / "raw_mgeo"))
    parser.add_argument(
        "--map", default=str(root / "map" / "lanelet2_map.osm")
    )
    args = parser.parse_args()
    data = load_mgeo(args.input)
    tree = ET.parse(args.map)
    osm = tree.getroot()
    ids = {
        element.attrib["id"]
        for element in osm
        if element.tag in {"node", "way", "relation"}
    }
    for way in osm.findall("way"):
        assert len(way.findall("nd")) >= 2
        assert all(nd.attrib["ref"] in ids for nd in way.findall("nd"))
    for relation in osm.findall("relation"):
        assert all(
            member.attrib["ref"] in ids
            for member in relation.findall("member")
        )
    lanelets = [
        relation
        for relation in osm.findall("relation")
        if tags(relation).get("type") == "lanelet"
    ]
    high = [
        lanelet
        for lanelet in lanelets
        if tags(lanelet).get("contest_zone") == "high_speed"
    ]
    expected_links = [
        link for link in data["link_set"] if not link.get("lazy_init")
    ]
    links_by_id = {link["idx"]: link for link in expected_links}
    assert len(lanelets) == len(expected_links) == 634
    assert len(high) == sum(is_high_speed(link) for link in expected_links) == 53
    assert all(
        tags(lanelet).get("speed_limit") == "60 km/h"
        for lanelet in lanelets
        if lanelet not in high
    )

    nodes = {node.attrib["id"]: tags(node) for node in osm.findall("node")}
    ways = {way.attrib["id"]: way for way in osm.findall("way")}

    def points(way_id):
        return [
            (
                float(nodes[nd.attrib["ref"]]["local_x"]),
                float(nodes[nd.attrib["ref"]]["local_y"]),
            )
            for nd in ways[way_id].findall("nd")
        ]

    for lanelet in lanelets:
        source_id = tags(lanelet)["source_link_id"]
        members = {
            member.attrib["role"]: member.attrib["ref"]
            for member in lanelet.findall("member")
        }
        left, right = points(members["left"]), points(members["right"])
        assert not self_intersects(left), (
            f"{source_id}: left boundary self-intersects"
        )
        assert not self_intersects(right), (
            f"{source_id}: right boundary self-intersects"
        )
        assert not polylines_cross(left, right), (
            f"{source_id}: left/right boundaries cross"
        )
        assert not end_connectors_cross(left, right), (
            f"{source_id}: lanelet end connector crosses a boundary"
        )

    crosswalks = [
        marking
        for marking in data["singlecrosswalk_set"]
        if is_pedestrian_crosswalk(marking)
    ]
    assert len(crosswalks) == 77
    assert sum(str(marking.get("sign_type")) == "533" for marking in crosswalks) == 17
    assert not is_pedestrian_crosswalk({"sign_type": "534"})
    assert not is_pedestrian_crosswalk({"sign_type": "544"})
    assert not is_pedestrian_crosswalk({})
    crosswalk_areas = [relation for relation in osm.findall("relation")
                      if tags(relation).get("subtype") == "crosswalk"]
    assert len(crosswalk_areas) == len(crosswalks)
    assert {tags(area)["source_id"] for area in crosswalk_areas} == {item["idx"] for item in crosswalks}
    clipped_crosswalks = []
    for crosswalk in crosswalks:
        polygon = clean_points(crosswalk["points"])
        triangles = triangulate_polygon(polygon)
        assert triangles, f"{crosswalk['idx']}: triangulation failed"
        assert len(triangles) % 3 == 0
        assert all(
            (b[0] - a[0]) * (c[1] - a[1])
            - (b[1] - a[1]) * (c[0] - a[0]) > 0.0
            for a, b, c in zip(
                triangles[::3], triangles[1::3], triangles[2::3]
            )
        ), f"{crosswalk['idx']}: triangle normal does not face +Z"
        road = next(
            (
                links_by_id[link_id]
                for link_id in crosswalk.get("link_id_list", [])
                if link_id in links_by_id
            ),
            None,
        )
        stripes, stripe_width = crosswalk_stripes(
            polygon, road["points"] if road else None
        )
        clipped, _ = crosswalk_stripes(
            polygon, road["points"] if road else None,
            exclude_polygons=[item["points"] for item in crosswalks if item["idx"] != crosswalk["idx"]],
        )
        assert len(clipped) >= 4, f"{crosswalk['idx']}: clipping removed crossing"
        if clipped != stripes:
            clipped_crosswalks.append(crosswalk["idx"])
        assert len(stripes) >= 4, f"{crosswalk['idx']}: too few zebra stripes"
        assert 0.0 < stripe_width < 0.75
        sx = stripes[1][0] - stripes[0][0]
        sy = stripes[1][1] - stripes[0][1]
        closed = polygon + [polygon[0]]
        axis_error = min(
            min(
                abs(sx * (b[1] - a[1]) - sy * (b[0] - a[0])),
                abs(sx * (b[0] - a[0]) + sy * (b[1] - a[1])),
            )
            / (
                math.hypot(sx, sy)
                * math.hypot(b[0] - a[0], b[1] - a[1])
            )
            for a, b in zip(closed, closed[1:])
            if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-9
        )
        assert axis_error <= 1e-6, (
            f"{crosswalk['idx']}: zebra stripes do not follow an outline axis"
        )
        expected_area = polygon_area(polygon)
        actual_area = triangle_area(triangles)
        assert abs(actual_area - expected_area) <= max(1e-5, expected_area * 1e-6), (
            f"{crosswalk['idx']}: triangle area mismatch "
            f"({actual_area} vs {expected_area})"
        )

    assert set(clipped_crosswalks) == {"B3256W000312", "B3256W000319"}, clipped_crosswalks
    horizontal = [(-5, -1, 0), (5, -1, 0), (5, 1, 0), (-5, 1, 0)]
    vertical = [(-1, -5, 0), (1, -5, 0), (1, 5, 0), (-1, 5, 0)]
    clipped, width = crosswalk_stripes(horizontal, exclude_polygons=[vertical])
    assert clipped
    assert all(abs(point[0]) >= 1 + width / 2 - 1e-9 for point in clipped)
    print(
        f"OK: {len(lanelets)} lanelets, {len(high)} high-speed unlimited, "
        f"{len(lanelets) - len(high)} at 60 km/h, "
        f"{len(crosswalks)} crosswalk polygons, geometry valid; "
        f"{len(clipped_crosswalks)} diagonal paint overlaps clipped"
    )


if __name__ == "__main__":
    main()
