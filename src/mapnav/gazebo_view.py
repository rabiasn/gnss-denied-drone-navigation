"""Local Gazebo viewer helpers. Never attach to or terminate other sessions."""
import os
import math
from pathlib import Path
import signal
import subprocess
import xml.etree.ElementTree as ET
from urllib.parse import unquote, urlsplit


def gui_config_with_pose(path, pose):
    """Change only the starting user camera; original GUI config stays intact."""
    if len(pose) != 6 or not all(math.isfinite(v) for v in pose):
        raise ValueError("Finite xyz/rpy camera pose required")
    root = ET.fromstring("<config>" + Path(path).read_text() + "</config>")
    camera = root.find("plugin[@filename='MinimalScene']/camera_pose")
    if camera is None: raise ValueError("GUI camera pose not configured")
    camera.text = " ".join(map(str, pose))
    ET.indent(root)
    return "\n".join(ET.tostring(child, encoding="unicode") for child in root) + "\n"


def view_commands(world, backend="metal", executable="gz", gui_config=None):
    if backend not in ("metal", "opengl"):
        raise ValueError("Unsupported rendering backend")
    server = [executable, "sim", "-s", "-r", "-v", "3",
              "--render-engine-server-api-backend", backend, str(Path(world).resolve())]
    gui = [executable, "sim", "-g", "-v", "3",
           "--render-engine-gui-api-backend", backend]
    if gui_config is not None:
        gui += ["--gui-config", str(Path(gui_config).resolve())]
    return server, gui


def check_local_world(world):
    """Read-only preflight for this locally generated world, without sockets."""
    world = Path(world).resolve()
    root = ET.parse(world).getroot()
    element = root.find("world")
    if element is None:
        raise ValueError("SDF must contain a world")
    assets = []
    for node in root.iter():
        if node.tag not in ("uri", "albedo_map") or not node.text:
            continue
        value = node.text.strip()
        uri = urlsplit(value)
        if uri.scheme:
            # Gazebo's Save world as exports local mesh paths as file:// URIs.
            # Accept those without relaxing the ban on remote model downloads.
            if uri.scheme != "file":
                raise ValueError("Viewer preflight accepts local assets only, not remote model downloads")
            if uri.netloc.lower() not in ("", "localhost") or uri.query or uri.fragment:
                raise ValueError("Viewer preflight accepts local file:// URIs only")
            path = Path(unquote(uri.path, errors="strict"))
            if not path.is_absolute():
                raise ValueError("Local file:// asset URI must contain an absolute path")
        else:
            path = Path(value)
        if not path.is_absolute():
            path = world.parent / path
        if not path.is_file():
            raise FileNotFoundError(f"Missing world asset: {path}")
        assets.append(str(path.resolve()))
    return {"world_name": element.get("name"), "world": str(world),
            "asset_references": len(assets), "all_local_assets_present": True}


def stop_owned_processes(processes):
    """Only Popen children started with start_new_session=True may be passed."""
    for process in reversed(processes):
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                continue
    for process in reversed(processes):
        if process.poll() is None:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=3)
