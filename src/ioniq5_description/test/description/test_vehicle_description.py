#!/usr/bin/env python3

import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
VEHICLE_CONFIG = PACKAGE_ROOT / "config" / "vehicle" / "vehicle_specs.yaml"
SENSOR_CONFIG = (
    PACKAGE_ROOT / "config" / "sensors" / "molit_2026_sensor_mounts.yaml"
)
XACRO_FILE = PACKAGE_ROOT / "urdf" / "vehicle" / "ioniq5_molit.urdf.xacro"


def xyz(element):
    return tuple(float(value) for value in element.attrib["xyz"].split())


class VehicleDescriptionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        command = [
            "rosrun",
            "xacro",
            "xacro",
            str(XACRO_FILE),
            "vehicle_config:=" + str(VEHICLE_CONFIG),
            "sensor_config:=" + str(SENSOR_CONFIG),
        ]
        result = subprocess.run(
            command,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        cls.robot = ET.fromstring(result.stdout)

    def joint_origin(self, joint_name):
        joint = self.robot.find("./joint[@name='{}']".format(joint_name))
        self.assertIsNotNone(joint, "missing joint {}".format(joint_name))
        origin = joint.find("origin")
        self.assertIsNotNone(origin, "missing origin for {}".format(joint_name))
        return xyz(origin)

    def named_visual(self, link_name, visual_name):
        link = self.robot.find("./link[@name='{}']".format(link_name))
        self.assertIsNotNone(link, "missing link {}".format(link_name))
        visual = link.find("./visual[@name='{}']".format(visual_name))
        self.assertIsNotNone(
            visual, "missing visual {} on {}".format(visual_name, link_name)
        )
        return visual

    def test_body_bounds_use_rear_axle_origin(self):
        body = self.named_visual("base_link", "body")
        center_x, _, center_z = xyz(body.find("origin"))
        length, width, height = (
            float(value) for value in body.find("geometry/box").attrib["size"].split()
        )

        self.assertAlmostEqual(length, 4.635, places=6)
        self.assertAlmostEqual(width, 1.892, places=6)
        self.assertAlmostEqual(height, 1.605, places=6)
        self.assertAlmostEqual(center_x - length / 2.0, -0.790, places=6)
        self.assertAlmostEqual(center_x + length / 2.0, 3.845, places=6)
        self.assertAlmostEqual(center_z - height / 2.0, -0.370, places=6)

    def test_base_link_is_rear_axle_height_above_ground(self):
        self.assertEqual(
            self.joint_origin("base_footprint_to_base_link"),
            (0.0, 0.0, 0.37),
        )

    def test_wheel_centers_identify_rear_and_front_axles(self):
        expected_x = {
            "rear_left_wheel": 0.0,
            "rear_right_wheel": 0.0,
            "front_left_wheel": 3.0,
            "front_right_wheel": 3.0,
        }
        for visual_name, wanted_x in expected_x.items():
            with self.subTest(visual=visual_name):
                wheel = self.named_visual("base_link", visual_name)
                wheel_x, _, wheel_z = xyz(wheel.find("origin"))
                self.assertAlmostEqual(wheel_x, wanted_x, places=6)
                self.assertAlmostEqual(wheel_z, 0.0, places=6)

    def test_sensor_mounts_remain_relative_to_rear_axle(self):
        self.assertEqual(
            self.joint_origin("base_link_to_gps_link"),
            (0.0, 0.0, 1.2),
        )
        self.assertEqual(
            self.joint_origin("base_link_to_lidar_link"),
            (1.5, 0.0, 1.25),
        )


if __name__ == "__main__":
    unittest.main()
