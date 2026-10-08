#!/usr/bin/env python3
"""Install the pinned release's model assets; never overwrite a saved world."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import stat
import tempfile
import urllib.request
import zipfile

PROJECT = Path(__file__).resolve().parents[1]
WORLD_DIR = PROJECT / "gazebo/helsinki"
PREFIX = "gazebo/helsinki/"


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def safe_name(name):
    path = PurePosixPath(name)
    if path.is_absolute() or ".." in path.parts or "\\" in name or path.as_posix() != name:
        raise ValueError("Unsafe archive path")
    return path


def verify_installed(world_dir, manifest):
    models = world_dir / "models"
    if not models.exists() and not models.is_symlink():
        return False
    if models.is_symlink() or not models.is_dir():
        raise ValueError("Existing models path is not a normal directory")
    for name, entry in manifest["files"].items():
        safe_name(name)
        if not name.startswith("models/"):
            continue
        path = world_dir / name
        if path.is_symlink() or not path.is_file() or not path.resolve().is_relative_to(models.resolve()):
            raise ValueError("Existing models are incomplete; no files were overwritten")
        if path.stat().st_size != entry["bytes"] or sha(path) != entry["sha256"]:
            raise ValueError("Existing models differ from this release; no files were overwritten")
    return True


def install_archive(archive_path, world_dir, release, manifest):
    if archive_path.stat().st_size != release["bytes"] or sha(archive_path) != release["sha256"]:
        raise ValueError("Archive SHA-256 or size does not match the pinned release")
    if verify_installed(world_dir, manifest):
        return "already_verified"
    expected = {PREFIX + name: entry for name, entry in manifest["files"].items()}
    expected[PREFIX + "assets-manifest.json"] = None
    with zipfile.ZipFile(archive_path) as archive:
        names = [item.filename for item in archive.infolist()]
        if len(names) != len(set(names)) or set(names) != set(expected):
            raise ValueError("Archive contents do not match the pinned manifest")
        for item in archive.infolist():
            safe_name(item.filename)
            if item.is_dir() or stat.S_ISLNK(item.external_attr >> 16) or item.flag_bits & 1:
                raise ValueError("Unsupported directory, symlink or encrypted entry")
            entry = expected[item.filename]
            limit = entry["bytes"] if entry else (world_dir / "assets-manifest.json").stat().st_size
            if item.file_size != limit:
                raise ValueError("Unexpected uncompressed file size")
        with tempfile.TemporaryDirectory(prefix=".assets-", dir=world_dir) as directory:
            staging = Path(directory)
            for number, item in enumerate(archive.infolist(), 1):
                relative = safe_name(item.filename.removeprefix(PREFIX))
                entry = expected[item.filename]
                digest = hashlib.sha256()
                if relative.parts[0] == "models":
                    target = staging / relative
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with archive.open(item) as src, target.open("xb") as dest:
                        for chunk in iter(lambda: src.read(1024 * 1024), b""):
                            dest.write(chunk)
                            digest.update(chunk)
                else:
                    with archive.open(item) as src:
                        for chunk in iter(lambda: src.read(1024 * 1024), b""):
                            digest.update(chunk)
                wanted = entry["sha256"] if entry else sha(world_dir / "assets-manifest.json")
                if digest.hexdigest() != wanted:
                    raise ValueError("Archive member checksum mismatch")
                if number % 512 == 0:
                    print(f"Doğrulanan dosya: {number}/{len(names)}", flush=True)
            destination = world_dir / "models"
            if destination.exists() or destination.is_symlink():
                raise ValueError("Models appeared during extraction; no files were overwritten")
            (staging / "models").rename(destination)
    return "installed"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--archive", type=Path, help="Use an already downloaded local ZIP")
    args = ap.parse_args()
    release = json.loads((WORLD_DIR / "release.json").read_text())
    manifest = json.loads((WORLD_DIR / "assets-manifest.json").read_text())
    try:
        if verify_installed(WORLD_DIR, manifest):
            print("Model dosyaları mevcut; SHA-256 kontrolleri geçti.")
            return 0
        if args.archive is not None:
            result = install_archive(args.archive.resolve(), WORLD_DIR, release, manifest)
        else:
            if not release["url"].startswith("https://github.com/rabiasn/gnss-denied-drone-navigation/releases/download/"):
                raise ValueError("Unexpected release download URL")
            with tempfile.TemporaryDirectory(prefix="gazebo-download-") as directory:
                path = Path(directory) / "world.zip"
                request = urllib.request.Request(release["url"], headers={"User-Agent": "Helsinki-Gazebo-assets"})
                print("Gazebo model paketi indiriliyor…", flush=True)
                with urllib.request.urlopen(request, timeout=30) as response, path.open("xb") as dest:
                    total, last_report = 0, 0
                    for chunk in iter(lambda: response.read(1024 * 1024), b""):
                        total += len(chunk)
                        if total > release["bytes"]:
                            raise ValueError("Download exceeds pinned archive size")
                        dest.write(chunk)
                        if total - last_report >= 32 * 1024 * 1024:
                            print(f"İndirilen: {total / 2**20:.0f}/{release['bytes'] / 2**20:.0f} MiB", flush=True)
                            last_report = total
                result = install_archive(path, WORLD_DIR, release, manifest)
        print(f"Model paketi hazır ({result}). Işıklar ve dünya dosyası değiştirilmedi.")
        return 0
    except (OSError, ValueError, zipfile.BadZipFile) as exc:
        ap.exit(1, f"Model paketi yüklenemedi: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
