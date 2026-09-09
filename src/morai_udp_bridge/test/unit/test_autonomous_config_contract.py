#!/usr/bin/env python3

import unittest
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[2]


class AutonomousConfigContractTest(unittest.TestCase):
    @staticmethod
    def load(section, name, node):
        with (PACKAGE_ROOT / "config" / section / name).open(
            encoding="utf-8"
        ) as stream:
            return yaml.safe_load(stream)[node]

    def test_competition_status_receiver_defaults(self):
        config = self.load(
            "status",
            "molit_2026_vehicle_status.yaml",
            "competition_vehicle_status_receiver_node",
        )
        self.assertEqual(config["listen_port"], 9094)
        self.assertEqual(config["status_topic"], "/vehicle/competition_status")
        self.assertEqual(config["stale_timeout_sec"], 0.25)
        self.assertGreater(config["receive_buffer_bytes"], 0)

    def test_runtime_gear_change_is_prepared_but_disabled(self):
        config = self.load(
            "control", "molit_2026_control.yaml", "control_sender_node"
        )
        self.assertFalse(config["gear_command_enabled"])
        self.assertEqual(config["drive_gear"], 4)
        self.assertEqual(config["gear_command_topic"], "/control/gear_command")
        self.assertEqual(
            config["vehicle_status_topic"], "/vehicle/competition_status"
        )
        self.assertLessEqual(
            config["gear_change_maximum_abs_speed_mps"], 0.1
        )
        self.assertGreaterEqual(
            config["gear_change_minimum_brake_command"], 0.5
        )


if __name__ == "__main__":
    unittest.main()
