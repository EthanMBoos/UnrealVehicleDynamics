#!/usr/bin/env python3
"""Fast checks for the dual launcher and synchronized-report contract."""

import copy
import json
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ardupilot"))

import run
from report_common import (
    dual_lockstep_summary,
    lockstep_summary,
    run_coverage_summary,
)


def exchange_rows(count=4):
    return [
        {
            "time_s": f"{(index + 1) / 120.0:.9f}",
            "servo_frame": str(100 + index),
            "step": str(500 + index),
            "physics_time_s": f"{(500 + index) / 120.0:.9f}",
            "frame_rate_hz": "120",
            "missed_frames": "0",
            "duplicate_frames": "0",
        }
        for index in range(count)
    ]


def truth_rows(count=2, final_time_s=0.025):
    return [
        {"time_s": f"{index * final_time_s / (count - 1):.9f}", "value": "0.0"}
        for index in range(count)
    ]


class DualContractTest(unittest.TestCase):
    def test_example_endpoints_and_controller_command(self):
        profile = run.PROFILES["dual"]
        plane = json.loads(profile["plane_config"].read_text())
        otter = json.loads(profile["otter_config"].read_text())
        self.assertEqual(plane["fixed_dt_s"], 1.0 / 120.0)
        self.assertEqual(plane["fixed_dt_s"], otter["fixed_dt_s"])
        self.assertEqual(plane["duration_s"], otter["duration_s"])
        self.assertEqual(plane["origin"], otter["origin"])
        self.assertEqual(plane["controller"], {"udp_port": 9002, "control_port": 9003})
        self.assertEqual(otter["controller"], {"udp_port": 9012, "control_port": 9013})

        command = run.dual_controller_command(
            profile, "otter", otter, Path("/tmp/scenario"), "test-otter", 17
        )
        self.assertIn("UVD_SIM_PORT_IN=9013", command)
        self.assertIn("UVD_SIM_PORT_OUT=9012", command)
        self.assertIn("UVD_SYSID=17", command)
        self.assertIn("UVD_FINISH_FILE=/scenario/finish", command)

    def test_report_rejects_frame_fault_and_step_skew(self):
        plane = exchange_rows()
        otter = copy.deepcopy(plane)
        self.assertTrue(lockstep_summary(plane)["verified"])
        self.assertEqual(dual_lockstep_summary(plane, otter)["max_step_skew"], 0)

        plane[2]["missed_frames"] = "1"
        self.assertFalse(lockstep_summary(plane)["verified"])
        otter[0]["step"] = str(int(otter[0]["step"]) + 1)
        self.assertEqual(dual_lockstep_summary(plane, otter)["max_step_skew"], 1)

    def test_report_rejects_truncated_coverage(self):
        complete = run_coverage_summary(
            exchange_rows(6), truth_rows(final_time_s=0.05), 0.05, 1.0 / 120.0, 20
        )
        self.assertTrue(complete["exchange_complete"])
        self.assertTrue(complete["truth_complete"])

        truncated = run_coverage_summary(
            exchange_rows(5), truth_rows(final_time_s=0.05), 0.05, 1.0 / 120.0, 20
        )
        self.assertFalse(truncated["exchange_complete"])


if __name__ == "__main__":
    unittest.main()
