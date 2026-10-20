"""Local Helsinki/Iris simulation construction and bounded flight helpers.

No hardware endpoints. ArduPilot GPS remains enabled in this first flight layer.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import subprocess
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

import numpy as np

VEHICLE_NAME = "project_iris"
WORLD_NAME = "helsinki_drone"


def remember_home(previous, position):
    """Capture the first launch reference once; later arming cannot move it."""
    values = tuple(float(v) for v in (previous if previous is not None else position))
    if len(values) != 3 or not all(map(math.isfinite, values)):
        raise ValueError("Home için sonlu üç boyutlu konum gerekli")
    return values


def flight_timeout_due(elapsed, limit, armed, state, handled):
    """One-shot return trigger, never repeated while returning or landing."""
    return bool(armed and not handled and elapsed >= limit and state in ("hover", "moving", "taking_off"))


def sub(parent, tag, text=None, **attrs):
    el = ET.SubElement(parent, tag, attrs)
    if text is not None:
        el.text = str(text)
    return el


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def scene_signature(world):
    return [ET.tostring(x, encoding="unicode") for x in world if x.tag in ("scene", "light")]


def top_height(triangles, x, y):
    """Highest vertical triangle intersection; no extrapolation across holes."""
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    den = (b[:, 1]-c[:, 1])*(a[:, 0]-c[:, 0])+(c[:, 0]-b[:, 0])*(a[:, 1]-c[:, 1])
    valid = np.abs(den) > 1e-10
    u, v = np.full(len(a), -1.), np.full(len(a), -1.)
    u[valid] = ((b[valid, 1]-c[valid, 1])*(x-c[valid, 0])+(c[valid, 0]-b[valid, 0])*(y-c[valid, 1]))/den[valid]
    v[valid] = ((c[valid, 1]-a[valid, 1])*(x-c[valid, 0])+(a[valid, 0]-c[valid, 0])*(y-c[valid, 1]))/den[valid]
    mask = valid & (u >= -1e-8) & (v >= -1e-8) & (u+v <= 1+1e-8)
    if not mask.any():
        return None
    return float((u[mask]*a[mask, 2]+v[mask]*b[mask, 2]+(1-u[mask]-v[mask])*c[mask, 2]).max())


def read_triangles(path):
    vertices, faces = [], []
    for line in Path(path).read_text().splitlines():
        t = line.split()
        if not t:
            continue
        if t[0] == "v":
            vertices.append([float(v) for v in t[1:4]])
        elif t[0] == "f":
            ids = [int(v.split("/")[0])-1 for v in t[1:]]
            for i in range(1, len(ids)-1):
                faces.append([ids[0], ids[i], ids[i+1]])
    return np.asarray(vertices)[np.asarray(faces)]


def survey_patch(city_dir, center=(120., -110.), radius=20., step=2.):
    """Sample a deliberately small initial flight envelope in the 3D map."""
    meta = json.loads((city_dir / "metadata.json").read_text())
    x, y = center
    chunks = []
    for model in meta["models"]:
        x0, y0, x1, y1 = model["core_xy_m"]
        if x0 > x+radius or x1 < x-radius or y0 > y+radius or y1 < y-radius:
            continue
        chunks.append(read_triangles(city_dir / "models" / model["derived"]))
    triangles = np.concatenate(chunks)
    points = [(float(px), float(py)) for px in np.arange(x-radius, x+radius+.01, step)
              for py in np.arange(y-radius, y+radius+.01, step)]
    heights = [top_height(triangles, px, py) for px, py in points]
    if any(z is None for z in heights):
        raise ValueError("Initial flight envelope contains unsampled/missing map surface")
    pad_points = [(x+dx, y+dy) for dx in np.linspace(-2., 2., 9) for dy in np.linspace(-2., 2., 9)]
    pad_heights = [top_height(triangles, px, py) for px, py in pad_points]
    if any(z is None for z in pad_heights):
        raise ValueError("Missing launch-pad surface")
    return {"center_xy_m": list(center), "radius_m": radius, "sample_step_m": step,
            "samples": len(points), "minimum_z_m": min(heights), "maximum_z_m": max(heights),
            "pad_minimum_z_m": min(pad_heights), "pad_maximum_z_m": max(pad_heights),
            "pad_surface_range_m": max(pad_heights)-min(pad_heights),
            "scope": "Sampled initial envelope only, not continuous obstacle avoidance or map certification"}


def material(visual, color):
    m = sub(visual, "material")
    for tag in ("ambient", "diffuse"):
        sub(m, tag, color)
    sub(m, "specular", "0 0 0 1")


def launch_pad(world, x, y, top):
    m = sub(world, "model", name="launch_pad")
    sub(m, "static", "true")
    sub(m, "pose", f"{x} {y} {top-.10} 0 0 0")
    body = sub(m, "link", name="body")
    for kind in ("collision", "visual"):
        e = sub(body, kind, name="pad_"+kind)
        sub(sub(sub(e, "geometry"), "box"), "size", "4 4 0.2")
        if kind == "visual":
            material(e, "0.15 0.18 0.19 1")
    # Thin visual-only H: no mass or hidden collision surface.
    for name, px, py, sx, sy in (("left", -.6, 0, .17, 1.6), ("right", .6, 0, .17, 1.6), ("bar", 0, 0, 1.2, .17)):
        v = sub(body, "visual", name="H_"+name)
        sub(v, "pose", f"{px} {py} 0.102 0 0 0")
        sub(sub(sub(v, "geometry"), "box"), "size", f"{sx} {sy} .004")
        material(v, "0.95 0.8 0.08 1")


def iris_model(upstream, fdm_port, pose):
    if not 1024 <= fdm_port <= 65535:
        raise ValueError("Invalid local simulation port")
    model = ET.Element("model", {"name": VEHICLE_NAME})
    sub(model, "pose", " ".join(map(str, pose)))
    iris = deepcopy(ET.parse(upstream / "models/iris_with_standoffs/model.sdf").getroot().find("model"))
    base = iris.find("link[@name='base_link']")
    camera = sub(base, "sensor", name="downward_camera", type="camera")
    sub(camera, "pose", "0 0 -0.08 0 1.5707963267948966 0")
    sub(camera, "topic", "/mapnav/uav/camera/image")
    sub(camera, "always_on", "true")
    sub(camera, "update_rate", 10)
    c = sub(camera, "camera")
    sub(c, "horizontal_fov", 1.0471975511965976)
    sub(c, "camera_info_topic", "/mapnav/uav/camera/info")
    image = sub(c, "image")
    for tag, value in (("width", 640), ("height", 480), ("format", "R8G8B8")):
        sub(image, tag, value)
    clip = sub(c, "clip")
    sub(clip, "near", .05)
    sub(clip, "far", 1000)
    imu = sub(base, "sensor", name="companion_imu", type="imu")
    sub(imu, "always_on", "true")
    sub(imu, "update_rate", 200)
    sub(imu, "topic", "/mapnav/uav/imu")
    # Keep ArduPilot's original 1000 Hz, FRD-rotated IMU and scoped topic.
    truth = sub(iris, "plugin", filename="gz-sim-odometry-publisher-system", name="gz::sim::systems::OdometryPublisher")
    for key, value in (("odom_frame", "world_enu"), ("robot_base_frame", "body_flu"), ("dimensions", 3),
                       ("odom_publish_frequency", 20), ("odom_topic", "/mapnav/evaluation/ground_truth")):
        sub(truth, key, value)
    model.append(iris)
    parent = ET.parse(upstream / "models/iris_with_gimbal/model.sdf").getroot().find("model")
    keep = {"gz::sim::systems::JointStatePublisher", "gz::sim::systems::LiftDrag", "gz::sim::systems::ApplyJointForce", "ArduPilotPlugin"}
    for original in parent.findall("plugin"):
        if original.get("name") not in keep:
            continue
        p = deepcopy(original)
        if p.get("name") == "ArduPilotPlugin":
            p.find("fdm_port_in").text = str(fdm_port)
            p.find("fdm_addr").text = "127.0.0.1"
            for control in list(p.findall("control")):
                if int(control.get("channel")) > 3:
                    p.remove(control)
        model.append(p)
    # Resolve all model meshes locally rather than permitting network fetches.
    for uri in model.iter("uri"):
        if uri.text.startswith("model://"):
            asset = upstream / "models" / uri.text.removeprefix("model://")
            if not asset.is_file():
                raise FileNotFoundError(asset)
            uri.text = str(asset.resolve())
    return model


def resolve_city_assets(world, city):
    """Rebase the portable city's local URIs into this run's absolute paths."""
    for uri in world.iter("uri"):
        parsed = urlsplit(uri.text or "")
        if parsed.scheme not in ("", "file") or parsed.netloc not in ("", "localhost") or parsed.query or parsed.fragment:
            raise ValueError("Only local city assets are supported")
        asset = Path(unquote(parsed.path)) if parsed.scheme == "file" else Path(uri.text or "")
        if not asset.is_absolute():
            asset = city / asset
        if not asset.is_file():
            raise FileNotFoundError(f"Missing city asset; run scripts/download_gazebo_assets.py: {asset}")
        uri.text = str(asset.resolve())


def build_world(project, out, fdm_port, center=(120., -110.), *, upstream=None):
    project, out = Path(project).resolve(), Path(out).resolve()
    city = project / "gazebo/helsinki"
    source = city / "world.sdf"
    # The published city already contains the saved scene and all five lights.
    # Never modify it or require a private world-lighting.sdf from another PC.
    saved_lighting = source
    tree = ET.parse(source)
    world = tree.getroot().find("world")
    lighting = ET.parse(saved_lighting).getroot().find("world")
    resolve_city_assets(world, city)
    for e in list(world):
        if e.tag in ("scene", "light") or e.tag == "model" and e.get("name") == "camera_rig":
            world.remove(e)
    for e in lighting:
        if e.tag in ("scene", "light"):
            world.append(deepcopy(e))
    world.set("name", WORLD_NAME)
    for plugin_name, filename in (("gz::sim::systems::NavSat", "gz-sim-navsat-system"),):
        if not any(p.get("name") == plugin_name for p in world.findall("plugin")):
            sub(world, "plugin", filename=filename, name=plugin_name)
    # Exact projected centre transformed using installed PROJ, without downloads.
    converted = subprocess.run(["cs2cs", "-r", "EPSG:3879", "EPSG:4326", "-f", "%.10f"],
                               input="25498500 6675500 0\n", text=True, capture_output=True, check=True)
    lat, lon, _ = map(float, converted.stdout.split())
    if not 59 < lat < 61 or not 24 < lon < 26:
        raise ValueError("Unexpected projected-to-geodetic axis order")
    anchor = [lat, lon, 0.]
    spherical = world.find("spherical_coordinates")
    if spherical is not None:
        world.remove(spherical)
    spherical = sub(world, "spherical_coordinates")
    for tag, value in (("latitude_deg", lat), ("longitude_deg", lon), ("elevation", 0), ("heading_deg", 0), ("surface_model", "EARTH_WGS84")):
        sub(spherical, tag, value)
    survey = survey_patch(city, center)
    if survey["pad_surface_range_m"] > .6:
        raise ValueError("Launch area too uneven for a small training pad")
    top = survey["pad_maximum_z_m"] + .12
    altitude = max(12., math.ceil(survey["maximum_z_m"]-top+5))
    if altitude > 30:
        raise ValueError("Initial envelope has tall obstacles; select another launch point")
    x, y = center
    launch_pad(world, x, y, top)
    upstream = Path(upstream) if upstream is not None else project / "vendor/ardupilot_gazebo"
    world.append(iris_model(upstream, fdm_port, [x, y, top+.25, 0, 0, math.pi/2]))
    if scene_signature(world) != scene_signature(lighting):
        raise ValueError("Saved scene/lighting changed")
    # The source city uses absolute local assets. Keep those references unchanged.
    for uri in world.iter("uri"):
        parsed = urlsplit(uri.text or "")
        asset = Path(unquote(parsed.path)) if parsed.scheme == "file" else Path(uri.text)
        if not asset.is_absolute() or not asset.is_file():
            raise ValueError(f"Missing/nonlocal simulation mesh {uri.text}")
    out.mkdir(parents=True, exist_ok=False)
    ET.indent(tree)
    tree.write(out / "world.sdf", encoding="utf-8", xml_declaration=True)
    # Whitespace indentation is not a scene/light semantic change.
    info = {"world_name": WORLD_NAME, "vehicle_name": VEHICLE_NAME, "source_world": str(source),
            "source_world_sha256": sha(source), "lighting_world_sha256": sha(saved_lighting),
            "saved_scene_and_lights_preserved": True, "survey": survey,
            "launch_xy_m": list(center), "pad_top_z_m": top, "recommended_altitude_m": altitude,
            "geodetic_anchor_lat_lon_alt": anchor, "anchor_note": "Helsinki map origin; heights retained from source EGM96 mesh, simplified SITL geodesy",
            "fdm_port": fdm_port, "camera_topic": "/mapnav/uav/camera/image", "imu_topic": "/mapnav/uav/imu",
            "gps_enabled": True, "not_validated": ["GNSS-denied return", "visual-inertial control", "continuous obstacle avoidance"],
            "vehicle_source": "https://github.com/ArduPilot/ardupilot_gazebo", "vehicle_license": "LGPL-3.0",
            "city_source": "City of Helsinki, Helsinki 3D Mesh 2017, CC BY 4.0"}
    (out / "world-info.json").write_text(json.dumps(info, indent=2)+"\n")
    return info


def check_target(north, east, altitude):
    values = (north, east, altitude)
    if not all(math.isfinite(v) for v in values) or math.hypot(north, east) > 10 or not 8 <= altitude <= 30:
        raise ValueError("İlk uçuş sınırı: başlangıçtan en çok 10 m, yükseklik 8..30 m")
    return tuple(map(float, values))
