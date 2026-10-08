"""Preserve a textured photogrammetry mesh while recentering / adding normals."""
import numpy as np
import math
from .mesh import obj_index


def smooth_obj_normals(text, crease_degrees=35):
    """Area-weighted normals within a geometric crease; coordinates/UVs unchanged."""
    if not math.isfinite(crease_degrees) or not 0 < crease_degrees < 90:
        raise ValueError("Crease must be finite and in (0,90) degrees")
    lines=text.splitlines();vertices=[];faces=[];face_lines=[];incident={}
    for number,line in enumerate(lines):
        tokens=line.split()
        if not tokens:continue
        if tokens[0]=='v':vertices.append(list(map(float,tokens[1:4])))
        elif tokens[0]=='f':
            corners=[c.split('/') for c in tokens[1:]]
            if len(corners)!=3:raise ValueError('Triangulation required')
            indices=[obj_index(c[0],len(vertices)) for c in corners]
            fi=len(faces);faces.append((indices,corners));face_lines.append(number)
            for vi in indices:incident.setdefault(vi,[]).append(fi)
    if not faces:raise ValueError('No triangles')
    indices=np.asarray([f[0] for f in faces]);p=np.asarray(vertices)[indices]
    cross=np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]);length=np.linalg.norm(cross,axis=1)
    if (length<=1e-10).any():raise ValueError('Degenerate triangles must be excluded before smoothing')
    unit=cross/length[:,None];threshold=math.cos(math.radians(crease_degrees))
    normal_ids={};lookup={}
    for vi,fi in incident.items():
        local=unit[fi]
        mask=(local@local.T)>=threshold-1e-12
        weighted=mask.astype(float)@cross[fi]
        weighted/=np.linalg.norm(weighted,axis=1)[:,None]
        for face,normal in zip(fi,weighted):
            key=tuple(round(float(v),10) for v in normal)
            lookup[(vi,face)]=normal_ids.setdefault(key,len(normal_ids)+1)
    rewrite={}
    for fi,(vertex_indices,corners) in enumerate(faces):
        rewrite[face_lines[fi]]='f '+' '.join(f'{c[0]}/{c[1] if len(c)>1 else ""}/{lookup[(vi,fi)]}' for vi,c in zip(vertex_indices,corners))
    output=[];inserted=False
    for number,line in enumerate(lines):
        tag=line.split()[:1]
        if tag==['vn']:continue
        if tag==['f'] and not inserted:
            output.extend('vn '+' '.join(f'{v:.10f}' for v in normal) for normal in normal_ids)
            inserted=True
        output.append(rewrite.get(number,line))
    return '\n'.join(output)+'\n',{'normal_policy':'area_weighted_crease','crease_degrees':crease_degrees,
        'normals':len(normal_ids),'coordinates_uv_materials_unchanged':True}


def clip_obj_to_core(text, origin_xyz, core_xy):
    """Clip source triangle/UV pairs to a nominal tile, without inventing caps.

    Linear edge interpolation preserves the source texture at cut points.
    Tile-border holes are NOT closed with arbitrary walls or a fake ground plane.
    Material names/libraries stay intact; normals are recomputed after cutting.
    """
    origin = tuple(float(v) for v in origin_xyz)
    limits = tuple(float(v) for v in core_xy)
    if len(origin) != 3 or len(limits) != 4 or not all(map(math.isfinite, origin + limits)):
        raise ValueError("Finite origin and xmin/ymin/xmax/ymax core required")
    if limits[2] <= limits[0] or limits[3] <= limits[1]:
        raise ValueError("Positive core size required")
    vertices, uvs, libraries, triangles = [], [], [], []
    material, source_faces, outside, cut, degenerate = None, 0, 0, 0, 0

    def cut_plane(polygon, axis, bound, keep_above):
        if not polygon: return []
        result = []
        previous = polygon[-1]
        prev_in = (previous[axis] >= bound) if keep_above else (previous[axis] <= bound)
        for current in polygon:
            curr_in = (current[axis] >= bound) if keep_above else (current[axis] <= bound)
            if curr_in != prev_in:
                t = (bound - previous[axis]) / (current[axis] - previous[axis])
                point = [a + t * (b - a) for a, b in zip(previous, current)]
                point[axis] = bound
                result.append(tuple(point))
            if curr_in: result.append(current)
            previous, prev_in = current, curr_in
        return result

    for line in text.splitlines():
        tokens = line.split("#", 1)[0].split()
        if not tokens: continue
        if tokens[0] == "mtllib": libraries.append(line)
        elif tokens[0] == "usemtl": material = " ".join(tokens[1:])
        elif tokens[0] == "v":
            if len(tokens) != 4: raise ValueError("3D vertices required")
            point = tuple(float(v) - origin[i] for i, v in enumerate(tokens[1:]))
            if not all(map(math.isfinite, point)): raise ValueError("Non-finite vertex")
            vertices.append(point)
        elif tokens[0] == "vt":
            if len(tokens) != 3: raise ValueError("2D UVs required")
            uv = tuple(map(float, tokens[1:]))
            if not all(map(math.isfinite, uv)): raise ValueError("Non-finite UV")
            uvs.append(uv)
        elif tokens[0] == "f":
            if len(tokens) != 4: raise ValueError("Source triangulation required")
            source_faces += 1
            corners = [value.split("/") for value in tokens[1:]]
            textured = [len(c) > 1 and bool(c[1]) for c in corners]
            if any(textured) and not all(textured): raise ValueError("Mixed UV presence in a face")
            polygon = [vertices[obj_index(c[0], len(vertices))] +
                       (uvs[obj_index(c[1], len(uvs))] if all(textured) else ()) for c in corners]
            original = polygon[:]
            for axis, bound, above in ((0, limits[0], True), (0, limits[2], False),
                                       (1, limits[1], True), (1, limits[3], False)):
                polygon = cut_plane(polygon, axis, bound, above)
            if len(polygon) < 3:
                outside += 1; continue
            if polygon != original: cut += 1
            for i in range(1, len(polygon) - 1):
                points = tuple(tuple(round(v, 8 if j < 3 else 12) for j, v in enumerate(point))
                               for point in (polygon[0], polygon[i], polygon[i+1]))
                a, b, c = points
                u, v = [b[j]-a[j] for j in range(3)], [c[j]-a[j] for j in range(3)]
                normal = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
                length = math.sqrt(sum(n*n for n in normal))
                if length < 1e-10: degenerate += 1; continue
                triangles.append((material, points, tuple(n / length for n in normal)))
    if not triangles: raise ValueError("Tile core contains no nondegenerate triangles")
    vertex_ids, uv_ids, normals, face_lines = {}, {}, [], []
    last_material = None
    for material, points, normal in triangles:
        if material != last_material:
            if material is not None: face_lines.append("usemtl " + material)
            last_material = material
        corners = []
        normals.append("vn " + " ".join(f"{v:.12g}" for v in normal))
        for point in points:
            xyz = tuple(round(v, 8) for v in point[:3])
            vi = vertex_ids.setdefault(xyz, len(vertex_ids) + 1)
            ti = ""
            if len(point) == 5:
                uv = tuple(round(v, 12) for v in point[3:])
                ti = str(uv_ids.setdefault(uv, len(uv_ids) + 1))
            corners.append(f"{vi}/{ti}/{len(normals)}")
        face_lines.append("f " + " ".join(corners))
    output = ["# UV-preserving nominal-core clipping; no synthetic seam caps", *libraries]
    output += ["v " + " ".join(f"{v:.8f}" for v in xyz) for xyz in vertex_ids]
    output += ["vt " + " ".join(f"{v:.12g}" for v in uv) for uv in uv_ids]
    output += normals + ["s off"] + face_lines
    bounds = [list(map(min, zip(*vertex_ids))), list(map(max, zip(*vertex_ids)))]
    return "\n".join(output) + "\n", {"source_triangles": source_faces, "triangles": len(triangles),
        "source_triangles_outside_core": outside, "source_triangles_cut": cut,
        "zero_area_cut_triangles_excluded": degenerate, "vertices": len(vertex_ids),
        "uvs": len(uv_ids), "core_xy_m": list(limits), "bounds_xyz_m": bounds,
        "changes": "Common translation, nominal-core clipping with linear UV interpolation, flat geometric normals"}


def prepare_obj(text, origin_xyz):
    origin = np.asarray(origin_xyz, dtype=float)
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError("Finite three-dimensional origin required")
    lines, vertices, faces = text.splitlines(), [], []
    rewritten = {}
    uv_count = 0
    for number, line in enumerate(lines):
        tokens = line.split("#", 1)[0].split()
        if not tokens: continue
        if tokens[0] == "v":
            if len(tokens) != 4: raise ValueError("Three-dimensional vertex required")
            value = np.asarray([float(v) for v in tokens[1:]]) - origin
            if not np.isfinite(value).all(): raise ValueError("Non-finite vertex")
            value = np.round(value, 8)
            vertices.append(value)
            rewritten[number] = "v " + " ".join(f"{v:.8f}" for v in value)
        elif tokens[0] == "vt":
            if not np.isfinite([float(v) for v in tokens[1:]]).all():
                raise ValueError("Non-finite UV coordinate")
            uv_count += 1
        elif tokens[0] == "f":
            if len(tokens) != 4: raise ValueError("Expected source-triangulated faces")
            corners = [c.split("/") for c in tokens[1:]]
            indices = [obj_index(c[0], len(vertices)) for c in corners]
            for corner in corners:
                if len(corner) > 1 and corner[1]: obj_index(corner[1], uv_count)
            a, b, c = [vertices[i] for i in indices]
            cross = np.cross(b - a, c - a)
            length = float(np.linalg.norm(cross))
            faces.append({"line": number, "corners": corners, "cross": cross, "length": length})
    if not faces: raise ValueError("No source mesh faces")
    if sum(f["cross"][2] for f in faces) <= 0:
        raise ValueError("Photogrammetric terrain winding is not predominantly upward")
    normal_lines, degenerate = [], 0
    for face in faces:
        if face["length"] < 1e-10:
            rewritten[face["line"]] = "# excluded zero-area source triangle"
            degenerate += 1
            continue
        normal = face["cross"] / face["length"]
        normal_lines.append("vn " + " ".join(f"{0. if n == 0 else n:.12g}" for n in normal))
        normal_index = len(normal_lines)
        rewritten[face["line"]] = "f " + " ".join(
            f"{corner[0]}/{corner[1] if len(corner) > 1 else ''}/{normal_index}"
            for corner in face["corners"])
    output, inserted = [], False
    for number, line in enumerate(lines):
        if line.split("#", 1)[0].split()[:1] == ["vn"]: continue
        if line.split("#", 1)[0].split()[:1] == ["f"] and not inserted:
            output.extend(normal_lines); inserted = True
        output.append(rewritten.get(number, line))
    array = np.asarray(vertices)
    return "\n".join(output) + "\n", {
        "vertices": len(vertices), "source_triangles": len(faces), "triangles": len(normal_lines),
        "zero_area_source_triangles_excluded": degenerate,
        "bounds_xyz_m": [array.min(axis=0).tolist(), array.max(axis=0).tolist()],
        "signed_projected_area_m2": float(sum(f["cross"][2] for f in faces) / 2),
        "changes": "Common translation only, 1e-8 m rounding, geometric flat normals; source winding / UVs / material assignments retained"}
