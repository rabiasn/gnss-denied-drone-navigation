from pathlib import Path
import itertools
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET

from mapnav.gazebo_view import check_local_world, view_commands, gui_config_with_pose


class GazeboViewTests(unittest.TestCase):
    def test_view_bookmark_changes_only_camera_and_preserves_original(self):
        path=Path(__file__).resolve().parents[1]/"config/helsinki-view.config"
        before=path.read_text();new=gui_config_with_pose(path,[0,0,100,0,1,0])
        root=ET.fromstring("<config>"+new+"</config>")
        self.assertEqual(root.findtext("plugin[@filename='MinimalScene']/camera_pose"),"0 0 100 0 1 0")
        self.assertEqual(root.findtext("plugin[@filename='MinimalScene']/camera_clip/far"),"5000")
        self.assertEqual(path.read_text(),before)
        with self.assertRaises(ValueError):gui_config_with_pose(path,[0,0,float('nan'),0,0,0])

    def test_helsinki_overview_clip_contains_the_whole_map(self):
        config = Path(__file__).resolve().parents[1] / "config/helsinki-view.config"
        # Gazebo config files have multiple top-level elements.
        root = ET.fromstring("<config>" + config.read_text() + "</config>")
        scene = root.find("plugin[@filename='MinimalScene']")
        pose = [float(value) for value in scene.findtext("camera_pose").split()]
        near = float(scene.findtext("camera_clip/near"))
        far = float(scene.findtext("camera_clip/far"))
        pitch, yaw = pose[4:6]
        forward = (math.cos(pitch) * math.cos(yaw),
                   math.cos(pitch) * math.sin(yaw), -math.sin(pitch))
        right = (-math.sin(yaw), math.cos(yaw), 0)
        up = (math.sin(pitch) * math.cos(yaw), math.sin(pitch) * math.sin(yaw), math.cos(pitch))
        tangent = math.tan(math.radians(float(scene.findtext("horizontal_fov"))) / 2)
        # Conservative bounds of this real 1 km map, including source heights.
        for corner in itertools.product((-500, 500), (-500, 500), (-10, 100)):
            depth = sum((corner[i] - pose[i]) * forward[i] for i in range(3))
            self.assertGreater(depth, near)
            self.assertLess(depth, far)
            delta = [corner[i] - pose[i] for i in range(3)]
            # Includes a wide GUI viewport (aspect 1.6), not only a square one.
            self.assertLess(abs(sum(delta[i] * right[i] for i in range(3))), depth * tangent)
            self.assertLess(abs(sum(delta[i] * up[i] for i in range(3))), depth * tangent / 1.6)

    def test_mac_uses_two_processes_and_preserves_path_spaces(self):
        path = Path("folder with spaces/world.sdf").resolve()
        server, gui = view_commands(path)
        self.assertIn("-s", server)
        self.assertNotIn("-g", server)
        self.assertIn("-g", gui)
        self.assertNotIn("-s", gui)
        self.assertEqual(server[-1], str(path))
        self.assertIn("--render-engine-server-api-backend", server)
        self.assertIn("--render-engine-gui-api-backend", gui)

    def test_invalid_backend_rejected(self):
        with self.assertRaises(ValueError):
            view_commands("world.sdf", "unknown")

    def test_gui_config_path_is_one_argument(self):
        config = Path("folder with spaces/gui.config").resolve()
        server, gui = view_commands("world.sdf", gui_config=config)
        self.assertEqual(gui[-2:], ["--gui-config", str(config)])
        self.assertNotIn("--gui-config", server)

    def test_local_assets_checked_without_launch(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "shape.obj").write_text("# fixture\n")
            world = root / "world.sdf"
            world.write_text('<sdf><world name="fixture"><model><link><visual><geometry><mesh><uri>shape.obj</uri></mesh></geometry></visual></link></model></world></sdf>')
            check = check_local_world(world)
            self.assertEqual(check["asset_references"], 1)
            self.assertEqual(check["world_name"], "fixture")

    def test_missing_asset_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory) / "world.sdf"
            world.write_text('<sdf><world name="fixture"><uri>missing.obj</uri></world></sdf>')
            with self.assertRaises(FileNotFoundError):
                check_local_world(world)

    def test_saved_world_local_file_uris_preserve_paths_and_world(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            asset = root / "shape + 100%.obj"
            asset.write_text("# fixture\n")
            world = root / "world.sdf"
            uris = ("file://" + str(asset), asset.as_uri(),
                    asset.as_uri().replace("file:///", "file://localhost/", 1))
            for uri in uris:
                with self.subTest(uri=uri):
                    world.write_text(f'<sdf><world name="fixture"><uri>{uri}</uri></world></sdf>')
                    before = world.read_bytes()
                    check = check_local_world(world)
                    self.assertEqual(check["asset_references"], 1)
                    self.assertTrue(check["all_local_assets_present"])
                    self.assertEqual(world.read_bytes(), before)

    def test_nonlocal_or_ambiguous_file_uris_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory) / "world.sdf"
            for uri in ("file://other-computer/tmp/model.obj", "file:relative.obj",
                        "file:///tmp/model.obj?download=1", "file:///tmp/model.obj#mesh"):
                with self.subTest(uri=uri):
                    world.write_text(f'<sdf><world name="fixture"><uri>{uri}</uri></world></sdf>')
                    with self.assertRaises(ValueError):
                        check_local_world(world)

    def test_remote_asset_does_not_trigger_download(self):
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory) / "world.sdf"
            for uri in ("https://example.com/model", "model://remote-model", "ftp://example.com/model"):
                with self.subTest(uri=uri):
                    world.write_text(f'<sdf><world name="fixture"><uri>{uri}</uri></world></sdf>')
                    with self.assertRaises(ValueError):
                        check_local_world(world)


if __name__ == "__main__":
    unittest.main()
