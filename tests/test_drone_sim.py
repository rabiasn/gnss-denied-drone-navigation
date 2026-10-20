import math
import os
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from mapnav.drone_sim import check_target, iris_model, launch_pad, read_triangles, top_height, remember_home, flight_timeout_due, resolve_city_assets


class DroneSimulationTests(unittest.TestCase):
    def test_portable_city_assets_are_resolved_without_changing_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            city = Path(tmp)
            mesh = city / "surface.obj"
            mesh.write_text("v 0 0 0\n")
            world = ET.fromstring('<world><uri>surface.obj</uri></world>')
            resolve_city_assets(world, city)
            self.assertEqual(world.findtext("uri"), str(mesh.resolve()))
            self.assertEqual(mesh.read_text(), "v 0 0 0\n")

    def test_remote_city_assets_are_rejected(self):
        for uri in ("https://example.com/model.obj", "model://remote/model.obj", "file://remote/model.obj"):
            world = ET.fromstring(f"<world><uri>{uri}</uri></world>")
            with self.assertRaises(ValueError):
                resolve_city_assets(world, Path("."))

    def test_missing_city_asset_is_not_silently_downloaded(self):
        with tempfile.TemporaryDirectory() as tmp:
            world = ET.fromstring('<world><uri>missing.obj</uri></world>')
            with self.assertRaises(FileNotFoundError):
                resolve_city_assets(world, Path(tmp))

    def test_home_is_not_replaced_after_landing_away_and_rearming(self):
        first = remember_home(None, [-110., 120., -2.8])
        later = remember_home(first, [-100., 120., -2.87])
        self.assertEqual(first, later)
        self.assertIsInstance(later, tuple)
        self.assertEqual(remember_home(later, [-105., 124., -3.]), first)

    def test_home_rejects_invalid_initial_reference(self):
        for position in ([0, 0], [0, 0, math.nan], [math.inf, 0, 0]):
            with self.assertRaises(ValueError):
                remember_home(None, position)

    def test_timeout_returns_only_once_and_does_not_restart_return_dwell(self):
        self.assertFalse(flight_timeout_due(179.9, 180, True, "hover", False))
        self.assertTrue(flight_timeout_due(180, 180, True, "hover", False))
        self.assertTrue(flight_timeout_due(181, 180, True, "moving", False))
        self.assertFalse(flight_timeout_due(181, 180, True, "hover", True))
        for state in ("returning", "landing", "landed", "ready", "await_armed"):
            self.assertFalse(flight_timeout_due(181, 180, True, state, False))
        self.assertFalse(flight_timeout_due(181, 180, False, "hover", False))

    def test_training_targets_are_bounded_and_finite(self):
        self.assertEqual(check_target(6, 4, 18), (6., 4., 18.))
        for args in ((8, 8, 18), (0, 0, 7), (0, 0, 31), (math.nan, 0, 18), (0, math.inf, 18)):
            with self.assertRaises(ValueError):
                check_target(*args)

    def test_height_uses_top_layer_and_rejects_outside(self):
        floor = [[0, 0, 2], [2, 0, 2], [0, 2, 2]]
        roof = [[0, 0, 12], [2, 0, 12], [0, 2, 12]]
        triangles = np.array([floor, roof], dtype=float)
        self.assertEqual(top_height(triangles, .5, .5), 12.)
        self.assertIsNone(top_height(triangles, 3, 3))

    def test_obj_fan_triangles(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "surface.obj"
            p.write_text("v 0 0 1\nv 1 0 1\nv 1 1 1\nv 0 1 1\nf 1 2 3 4\n")
            self.assertEqual(read_triangles(p).shape, (2, 3, 3))

    def test_pad_has_one_physical_surface(self):
        w = ET.Element("world")
        launch_pad(w, 120, -110, 2.6)
        self.assertEqual(len(w.findall(".//collision")), 1)
        self.assertEqual(len(w.findall(".//visual")), 4)
        pose = list(map(float, w.findtext("model/pose").split()))
        self.assertAlmostEqual(pose[2]+.1, 2.6)

    def test_iris_preserves_dynamics_and_upstream_autopilot_imu(self):
        root = Path(__file__).resolve().parents[1]
        upstream = Path(os.environ.get("MAPNAV_TEST_RUNTIME", str(root))) / "vendor/ardupilot_gazebo"
        if not upstream.is_dir():
            self.skipTest("Local upstream simulator assets not installed")
        original = ET.parse(upstream / "models/iris_with_standoffs/model.sdf").getroot().find("model")
        model = iris_model(upstream, 19002, [120, -110, 3, 0, 0, math.pi/2])
        derived = model.find("model")
        self.assertEqual([n.text for n in original.findall(".//inertial/mass")], [n.text for n in derived.findall(".//inertial/mass")])
        self.assertEqual([n.text for n in original.findall(".//joint/axis/dynamics/damping")], [n.text for n in derived.findall(".//joint/axis/dynamics/damping")])
        original_imu = original.find("link[@name='imu_link']/sensor")
        copied_imu = derived.find("link[@name='imu_link']/sensor")
        self.assertEqual(ET.tostring(original_imu), ET.tostring(copied_imu))
        self.assertEqual(len(model.findall("plugin[@name='gz::sim::systems::LiftDrag']")), 8)
        self.assertEqual(len(model.findall("plugin[@name='gz::sim::systems::ApplyJointForce']")), 4)
        autopilot = model.find("plugin[@name='ArduPilotPlugin']")
        self.assertEqual(autopilot.findtext("fdm_addr"), "127.0.0.1")
        self.assertEqual(autopilot.findtext("fdm_port_in"), "19002")
        self.assertEqual(len(autopilot.findall("control")), 4)
        self.assertEqual(derived.findtext("link[@name='base_link']/sensor[@name='downward_camera']/update_rate"), "10")
        self.assertTrue(all(Path(u.text).is_file() for u in model.iter("uri")))


if __name__ == "__main__":
    unittest.main()
