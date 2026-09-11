#!/usr/bin/python3
"""Convert the bundled MORAI K-City MGeo layers to a Lanelet2 OSM map."""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

from mgeo_common import clean_points, distance, is_high_speed, is_pedestrian_crosswalk, lane_bounds, load_mgeo


SHAPE_TO_SUBTYPE = {
    "solid": "solid",
    "broken": "dashed",
    "solid broken": "solid_dashed",
    "broken solid": "dashed_solid",
}


class Osm:
    def __init__(self):
        self.root = ET.Element("osm", version="0.6", generator="morai_kcity_hd_map")
        self.next_id = 1
        self.nodes = {}
        self.ways = {}
        self.relations = {}

    def _id(self):
        value = self.next_id
        self.next_id += 1
        return value

    def node(self, point):
        key = tuple(round(v, 4) for v in point)
        if key in self.nodes:
            return self.nodes[key]
        osm_id = self._id()
        node = ET.SubElement(self.root, "node", id=str(osm_id), visible="true", version="1", lat="0", lon="0")
        for name, value in (("local_x", point[0]), ("local_y", point[1]), ("ele", point[2])):
            ET.SubElement(node, "tag", k=name, v=f"{value:.6f}")
        self.nodes[key] = osm_id
        return osm_id

    def way(self, points, tags):
        points = clean_points(points)
        if len(points) < 2:
            raise ValueError("way requires at least two different points")
        osm_id = self._id()
        way = ET.SubElement(self.root, "way", id=str(osm_id), visible="true", version="1")
        for point in points:
            ET.SubElement(way, "nd", ref=str(self.node(point)))
        for key, value in tags.items():
            ET.SubElement(way, "tag", k=str(key), v=str(value))
        self.ways[osm_id] = way
        return osm_id

    def relation(self, members, tags):
        osm_id = self._id()
        relation = ET.SubElement(self.root, "relation", id=str(osm_id), visible="true", version="1")
        for member_type, ref, role in members:
            ET.SubElement(relation, "member", type=member_type, ref=str(ref), role=role)
        for key, value in tags.items():
            ET.SubElement(relation, "tag", k=str(key), v=str(value))
        self.relations[osm_id] = relation
        return osm_id


def boundary_tags(boundary):
    lane_type = int(boundary.get("lane_type", [0])[0])
    shape = boundary.get("lane_shape", ["solid"])[0]
    if lane_type == 530:
        return {"type": "stop_line", "color": "white", "source_id": boundary["idx"]}
    subtype = SHAPE_TO_SUBTYPE.get(shape, "solid")
    restricted = lane_type in {501, 505, 506, 515, 530, 531} or "solid" in shape
    return {
        "type": "line_thin",
        "subtype": subtype,
        "color": boundary.get("lane_color", ["white"])[0],
        "lane_change": "no" if restricted else "yes",
        "width": boundary.get("lane_width", 0.15),
        "source_id": boundary["idx"],
        "source_lane_type": lane_type,
    }


def midpoint(points):
    point = points[len(points) // 2]
    return point[0], point[1], point[2]


def add_crosswalks(osm, markings):
    existing = {
        tags.get("source_id")
        for relation in osm.root.findall("relation")
        for tags in [{tag.attrib["k"]: tag.attrib["v"] for tag in relation.findall("tag")}]
        if tags.get("subtype") == "crosswalk"
    }
    added = 0
    for marking in markings:
        if not is_pedestrian_crosswalk(marking) or marking["idx"] in existing:
            continue
        points = clean_points(marking["points"])
        if len(points) > 2 and distance(points[0], points[-1]) > 0.01:
            points.append(points[0])
        outer = osm.way(points, {"type": "line_thin", "subtype": "solid", "color": "white", "source_id": marking["idx"]})
        osm.relation(
            [("way", outer, "outer")],
            {"type": "multipolygon", "subtype": "crosswalk", "participant:pedestrian": "yes",
             "source_id": marking["idx"], "source_sign_type": str(marking["sign_type"])},
        )
        existing.add(marking["idx"])
        added += 1
    return added


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default=str(Path(__file__).resolve().parents[1] / "raw_mgeo"))
    parser.add_argument("--output", default=str(Path(__file__).resolve().parents[1] / "map" / "lanelet2_map.osm"))
    parser.add_argument("--add-missing-crosswalks", action="store_true",
                        help="Append missing crosswalk areas to --output; preserve every existing map element")
    args = parser.parse_args()

    data = load_mgeo(args.input)
    if args.add_missing_crosswalks:
        osm = Osm()
        osm.root = ET.parse(args.output).getroot()
        osm.next_id = max(int(element.attrib["id"]) for element in osm.root
                          if element.tag in {"node", "way", "relation"}) + 1
        added = add_crosswalks(osm, data["singlecrosswalk_set"])
        if added:
            ET.ElementTree(osm.root).write(args.output, encoding="utf-8", xml_declaration=True)
        print(f"added {added} missing crosswalk areas; existing map elements preserved")
        return
    links = [link for link in data["link_set"] if not link.get("lazy_init")]
    boundaries = {boundary["idx"]: boundary for boundary in data["lane_boundary_set"]}
    osm = Osm()

    source_boundary_ways = {}
    stop_line_ways = {}
    for boundary in boundaries.values():
        points = clean_points(boundary["points"])
        if len(points) < 2:
            continue
        way_id = osm.way(points, boundary_tags(boundary))
        source_boundary_ways[boundary["idx"]] = way_id
        if 530 in boundary.get("lane_type", []):
            stop_line_ways[boundary["idx"]] = way_id

    speed_60 = osm.relation([], {"type": "regulatory_element", "subtype": "speed_limit", "sign_type": "60 km/h"})
    lanelet_by_source = {}
    lanelet_element_by_source = {}
    bound_cache = {}

    def compound_way(link, side, points, real_boundary):
        ids = tuple(sorted(link.get(f"lane_mark_{side}", [])))
        direction = 1
        if ids:
            first = clean_points(boundaries[ids[0]]["points"])
            direction = 1 if distance(first[0], points[0]) <= distance(first[-1], points[0]) else -1
        key = (side, ids, direction) if ids and real_boundary else (side, link["idx"])
        if key in bound_cache:
            return bound_cache[key]
        if ids and real_boundary:
            tags = boundary_tags(boundaries[ids[0]])
            tags["source_boundary_ids"] = ";".join(ids)
        else:
            tags = {"type": "virtual", "subtype": "road_border", "source_id": f"synthetic:{link['idx']}:{side}"}
        bound_cache[key] = osm.way(points, tags)
        return bound_cache[key]

    for link in links:
        left, right, left_real, right_real = lane_bounds(link, boundaries)
        left_way = compound_way(link, "left", left, left_real)
        right_way = compound_way(link, "right", right, right_real)
        high_speed = is_high_speed(link)
        members = [("way", left_way, "left"), ("way", right_way, "right")]
        if not high_speed:
            members.append(("relation", speed_60, "regulatory_element"))
        tags = {
            "type": "lanelet",
            "subtype": "road",
            "location": "nonurban" if high_speed else "urban",
            "one_way": "yes",
            "participant:vehicle": "yes",
            "source_link_id": link["idx"],
            "road_id": link.get("road_id", ""),
            "lane_change_left": "yes" if link.get("can_move_left_lane") else "no",
            "lane_change_right": "yes" if link.get("can_move_right_lane") else "no",
        }
        if high_speed:
            tags["contest_zone"] = "high_speed"
            tags["contest_speed_limit"] = "unlimited"
        else:
            tags["speed_limit"] = "60 km/h"
        if link.get("related_signal"):
            tags["turn_direction"] = link["related_signal"]
        relation_id = osm.relation(members, tags)
        lanelet_by_source[link["idx"]] = relation_id
        lanelet_element_by_source[link["idx"]] = osm.relations[relation_id]

    stop_lines = [(way_id, clean_points(boundaries[source_id]["points"])) for source_id, way_id in stop_line_ways.items()]
    for light in data["traffic_light_set"]:
        x, y, z = (float(v) for v in light["point"])
        width = max(0.35, float(light.get("width") or 0.5))
        heading = float(light.get("heading") or 0.0) * 3.141592653589793 / 180.0
        dx, dy = 0.5 * width * __import__("math").cos(heading), 0.5 * width * __import__("math").sin(heading)
        light_way = osm.way(
            [(x - dx, y - dy, z), (x + dx, y + dy, z)],
            {"type": "traffic_light", "subtype": light.get("type", "car"), "height": light.get("height", 0.7), "source_id": light["idx"]},
        )
        controlled = [source for source in light.get("link_id_list", []) if source in lanelet_by_source]
        candidates = []
        for source in controlled:
            endpoint = clean_points(next(link["points"] for link in links if link["idx"] == source))[-1]
            candidates.extend((distance(endpoint, midpoint(points)), way_id) for way_id, points in stop_lines)
        members = [("way", light_way, "refers")]
        if candidates:
            nearest_distance, nearest_way = min(candidates)
            if nearest_distance <= 25.0:
                members.append(("way", nearest_way, "ref_line"))
        relation_id = osm.relation(members, {"type": "regulatory_element", "subtype": "traffic_light", "source_id": light["idx"]})
        for source in controlled:
            ET.SubElement(lanelet_element_by_source[source], "member", type="relation", ref=str(relation_id), role="regulatory_element")

    add_crosswalks(osm, data["singlecrosswalk_set"])

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(osm.root).write(output, encoding="utf-8", xml_declaration=True)
    print(f"wrote {output}: {len(links)} lanelets, {sum(is_high_speed(link) for link in links)} high-speed, "
          f"{len(boundaries)} boundaries, {len(stop_line_ways)} stop lines, "
          f"{len(data['traffic_light_set'])} traffic lights, "
          f"{sum(is_pedestrian_crosswalk(item) for item in data['singlecrosswalk_set'])} crosswalk polygons")


if __name__ == "__main__":
    main()
