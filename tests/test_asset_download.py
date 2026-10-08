import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

SPEC = importlib.util.spec_from_file_location("assets", Path(__file__).resolve().parents[1] / "scripts/download_gazebo_assets.py")
assets = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(assets)


def fixture(root, extra=None, member_digest=None):
    world = root / "world"
    world.mkdir()
    original = b"saved user lighting\n"
    (world / "world.sdf").write_bytes(original)
    payload = {"models/tile.obj": b"v 0 0 0\n", "world.sdf": b"packaged world\n"}
    manifest = {"schema_version": 1, "files": {name: {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
                for name, data in payload.items()}}
    if member_digest is not None:
        manifest["files"]["models/tile.obj"]["sha256"] = member_digest
    encoded = (json.dumps(manifest, indent=2) + "\n").encode()
    (world / "assets-manifest.json").write_bytes(encoded)
    archive = root / "release.zip"
    with zipfile.ZipFile(archive, "w") as handle:
        for name, data in payload.items():
            handle.writestr(assets.PREFIX + name, data)
        handle.writestr(assets.PREFIX + "assets-manifest.json", encoded)
        if extra:
            handle.writestr(extra, b"unexpected")
    release = {"bytes": archive.stat().st_size, "sha256": assets.sha(archive)}
    return world, archive, release, manifest, original


class AssetDownloadTests(unittest.TestCase):
    def test_installs_models_without_overwriting_saved_world(self):
        with tempfile.TemporaryDirectory() as directory:
            world, archive, release, manifest, original = fixture(Path(directory))
            self.assertEqual(assets.install_archive(archive, world, release, manifest), "installed")
            self.assertEqual((world / "models/tile.obj").read_bytes(), b"v 0 0 0\n")
            self.assertEqual((world / "world.sdf").read_bytes(), original)
            self.assertTrue(assets.verify_installed(world, manifest))
            self.assertEqual(assets.install_archive(archive, world, release, manifest), "already_verified")

    def test_archive_hash_mismatch_is_rejected_before_extraction(self):
        with tempfile.TemporaryDirectory() as directory:
            world, archive, release, manifest, original = fixture(Path(directory))
            release["sha256"] = "0" * 64
            with self.assertRaises(ValueError):
                assets.install_archive(archive, world, release, manifest)
            self.assertFalse((world / "models").exists())
            self.assertEqual((world / "world.sdf").read_bytes(), original)

    def test_unexpected_path_and_extra_files_are_rejected(self):
        for extra in ("../escape.obj", assets.PREFIX + "unexpected.obj"):
            with self.subTest(extra=extra), tempfile.TemporaryDirectory() as directory:
                world, archive, release, manifest, original = fixture(Path(directory), extra)
                with self.assertRaises(ValueError):
                    assets.install_archive(archive, world, release, manifest)
                self.assertFalse((world / "models").exists())

    def test_member_hash_mismatch_leaves_no_partial_models(self):
        with tempfile.TemporaryDirectory() as directory:
            world, archive, release, manifest, original = fixture(Path(directory), member_digest="0" * 64)
            with self.assertRaises(ValueError):
                assets.install_archive(archive, world, release, manifest)
            self.assertFalse((world / "models").exists())
            self.assertEqual((world / "world.sdf").read_bytes(), original)

    def test_modified_models_are_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            world, archive, release, manifest, original = fixture(Path(directory))
            assets.install_archive(archive, world, release, manifest)
            target = world / "models/tile.obj"
            target.write_text("my modified geometry\n")
            with self.assertRaises(ValueError):
                assets.install_archive(archive, world, release, manifest)
            self.assertEqual(target.read_text(), "my modified geometry\n")

    def test_symlinked_models_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            world, archive, release, manifest, original = fixture(root)
            elsewhere = root / "elsewhere"
            elsewhere.mkdir()
            (world / "models").symlink_to(elsewhere, target_is_directory=True)
            with self.assertRaises(ValueError):
                assets.install_archive(archive, world, release, manifest)
            self.assertEqual(list(elsewhere.iterdir()), [])

    def test_unsafe_member_names(self):
        for name in ("../escape", "/absolute", "models/../escape", "models\\tile.obj", "models//tile.obj"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                assets.safe_name(name)


if __name__ == "__main__":
    unittest.main()
