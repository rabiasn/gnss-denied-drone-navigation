"""Repair OBJ triangle orientation without changing geometry or UV seams.

Orient adjacent faces consistently, choose the outside of closed components by
signed volume (open components by supplied normals), and recompute flat normals.
This is not hole filling, extrusion, or geometric / geodetic certification.
"""
from collections import Counter, defaultdict
import numpy as np


def obj_index(token, count):
    value = int(token)
    index = value - 1 if value > 0 else count + value if value < 0 else -1
    if not 0 <= index < count: raise ValueError("OBJ index outside declared array")
    return index


def audit_obj_surfaces(text):
    """Read-only geometric-normal and boundary audit; coordinates are untouched.

    Welding is only for measurement. OBJ units are assumed to be metres, just as
    in the HGM adapter. This is not a self-intersection or geodetic certificate.
    """
    vertices, normals, faces, normal_faces = [], [], [], []
    texcoord_count = 0
    for line in text.splitlines():
        tokens = line.split("#", 1)[0].split()
        if not tokens:
            continue
        if tokens[0] in ("v", "vn"):
            if len(tokens) != 4:
                raise ValueError("Expected three-dimensional OBJ vertex / normal")
            value = np.asarray([float(x) for x in tokens[1:]])
            if not np.isfinite(value).all():
                raise ValueError("Non-finite vertex / normal")
            (vertices if tokens[0] == "v" else normals).append(value)
        elif tokens[0] == "vt":
            if not 2 <= len(tokens) <= 4 or not np.isfinite([float(x) for x in tokens[1:]]).all():
                raise ValueError("Invalid OBJ texture coordinate")
            texcoord_count += 1
        elif tokens[0] == "f":
            corners = [c.split("/") for c in tokens[1:]]
            if len(corners) != 3 or any(len(c) != 3 or not c[2] for c in corners):
                raise ValueError("Expected triangle with supplied normals")
            for corner in corners:
                if corner[1]:
                    obj_index(corner[1], texcoord_count)
            faces.append([obj_index(c[0], len(vertices)) for c in corners])
            normal_faces.append([obj_index(c[2], len(normals)) for c in corners])
    if not faces:
        raise ValueError("No mesh triangles")
    vertices, normals = np.asarray(vertices), np.asarray(normals)
    welded, inverse = np.unique(np.round(vertices * 1e6).astype(np.int64), axis=0, return_inverse=True)
    edges, directions = Counter(), Counter()
    degenerate = bad_normals = 0
    minimum_cosine, volume = 1., 0.
    for face, normal_face in zip(faces, normal_faces):
        a, b, c = vertices[face]
        cross = np.cross(b - a, c - a)
        length = float(np.linalg.norm(cross))
        if length < 1e-10:
            degenerate += 1
        else:
            geometric = cross / length
            lengths = np.linalg.norm(normals[normal_face], axis=1)
            if np.any(np.abs(lengths - 1) > 1e-9):
                bad_normals += 1
            for index, norm in zip(normal_face, lengths):
                cosine = float(np.dot(geometric, normals[index]) / norm) if norm > 1e-12 else -1.
                minimum_cosine = min(minimum_cosine, cosine)
        volume += float(np.dot(a, np.cross(b, c)) / 6)
        ids = [int(inverse[index]) for index in face]
        for first, second in zip(ids, ids[1:] + ids[:1]):
            key = tuple(sorted((first, second)))
            edges[key] += 1
            directions[key] += 1 if first < second else -1
    boundary = [key for key, count in edges.items() if count == 1]
    adjacency = defaultdict(set)
    for a, b in boundary:
        adjacency[a].add(b); adjacency[b].add(a)
    seen, boundary_components = set(), []
    for seed in adjacency:
        if seed in seen:
            continue
        pending, ids = [seed], set()
        while pending:
            index = pending.pop()
            if index in ids:
                continue
            ids.add(index); seen.add(index)
            pending.extend(adjacency[index] - ids)
        component_edges = [e for e in boundary if e[0] in ids]
        component = {"vertices": len(ids), "edges": len(component_edges)}
        if len(ids) == 3 and len(component_edges) == 3:
            points = welded[sorted(ids)].astype(float) / 1e6
            a, b, c = points
            area = float(np.linalg.norm(np.cross(b - a, c - a)) / 2)
            longest = max(float(np.linalg.norm(points[i] - points[j])) for i in range(3) for j in range(i))
            component.update({"area_m2_assuming_meter_units": area,
                              "maximum_edge_m": longest,
                              "effective_width_m": 2 * area / longest})
        boundary_components.append(component)
    inconsistent = sum(directions[key] != 0 for key, count in edges.items() if count == 2)
    nonmanifold = sum(count > 2 for count in edges.values())
    return {"vertices": len(vertices), "triangles": len(faces), "degenerate_triangles": degenerate,
            "non_unit_normal_faces": bad_normals, "minimum_winding_normal_cosine": minimum_cosine,
            "boundary_edges": len(boundary), "nonmanifold_edges": nonmanifold,
            "inconsistent_shared_edges": inconsistent,
            "closed_consistently_oriented_after_welding": not boundary and not nonmanifold and not inconsistent,
            "boundary_components": boundary_components, "signed_volume_m3": volume,
            "measurement_weld_tolerance_m": 1e-6}


def correct_obj_winding(text):
    lines = text.splitlines()
    vertices, normals, faces = [], [], []
    for line_number, line in enumerate(lines):
        tokens = line.split("#", 1)[0].split()
        if not tokens:
            continue
        if tokens[0] in ("v", "vn"):
            if len(tokens) != 4:
                raise ValueError("Expected three-dimensional OBJ vertex / normal")
            value = np.asarray([float(x) for x in tokens[1:]])
            if not np.isfinite(value).all():
                raise ValueError("Non-finite OBJ vertex / normal")
            (vertices if tokens[0] == "v" else normals).append(value)
        elif tokens[0] == "f":
            if len(tokens) != 4:
                raise ValueError("Only source-triangulated OBJ faces are accepted")
            corners = [corner.split("/") for corner in tokens[1:]]
            if any(len(corner) != 3 or not corner[2] for corner in corners):
                raise ValueError("All triangle corners need supplied normal indices")
            indices = [obj_index(corner[0], len(vertices)) for corner in corners]
            declared = np.mean([normals[obj_index(c[2], len(normals))] for c in corners], axis=0)
            a, b, c = [vertices[i] for i in indices]
            cross = np.cross(b - a, c - a)
            if np.linalg.norm(cross) < 1e-10:
                raise ValueError("Degenerate triangle")
            faces.append({"line": line_number, "corners": corners, "indices": indices,
                          "cross": cross, "declared": declared})
    if not faces:
        raise ValueError("No auditable mesh triangles")
    vertices = np.asarray(vertices)
    # Weld coordinates ONLY for topology analysis, preserving exported vertices
    # and UV seams verbatim. Source seam coordinates match to micrometre precision.
    _, inverse = np.unique(np.round(vertices * 1e6).astype(np.int64), axis=0, return_inverse=True)
    edges = defaultdict(list)
    face_edges = []
    for index, face in enumerate(faces):
        ids = [int(inverse[i]) for i in face["indices"]]
        keys = []
        for first, second in zip(ids, ids[1:] + ids[:1]):
            key = tuple(sorted((first, second)))
            edges[key].append((index, 1 if first < second else -1))
            keys.append(key)
        face_edges.append(keys)
    neighbors = defaultdict(list)
    for entries in edges.values():
        if len(entries) == 2:
            (i, direction_i), (j, direction_j) = entries
            relation = -direction_i * direction_j
            neighbors[i].append((j, relation))
            neighbors[j].append((i, relation))
    signs, components = {}, []
    for seed in range(len(faces)):
        if seed in signs:
            continue
        signs[seed] = 1
        pending, component = [seed], []
        while pending:
            index = pending.pop()
            component.append(index)
            for neighbor, relation in neighbors[index]:
                expected = signs[index] * relation
                if neighbor in signs:
                    if signs[neighbor] != expected:
                        raise ValueError("Non-orientable / contradictory mesh topology")
                else:
                    signs[neighbor] = expected
                    pending.append(neighbor)
        component_edges = {key for i in component for key in face_edges[i]}
        closed = all(len(edges[key]) == 2 for key in component_edges)
        volume, normal_vote = 0., 0.
        for i in component:
            a, b, c = vertices[faces[i]["indices"]]
            volume += signs[i] * float(np.dot(a, np.cross(b, c)) / 6)
            normal_vote += signs[i] * float(np.dot(faces[i]["cross"], faces[i]["declared"]))
        if closed:
            if abs(volume) < 1e-9:
                raise ValueError("Closed component has ambiguous signed volume")
            outside_sign, method = (1 if volume > 0 else -1), "positive_signed_volume"
        else:
            if abs(normal_vote) < 1e-9:
                raise ValueError("Open component has no usable outward source normals")
            outside_sign, method = (1 if normal_vote > 0 else -1), "area_weighted_source_normal_vote"
        for i in component:
            signs[i] *= outside_sign
        components.append({"triangles": len(component), "closed_after_welding": closed,
                           "outside_chosen_by": method})

    rewritten, normal_lines = {}, []
    flipped = roof_faces = wall_faces = base_faces = normal_disagreements = 0
    edge_directions = Counter()
    volume = 0.
    for i, face in enumerate(faces):
        order = [0, 2, 1] if signs[i] < 0 else [0, 1, 2]
        flipped += int(signs[i] < 0)
        normal = signs[i] * face["cross"] / np.linalg.norm(face["cross"])
        normal_disagreements += int(np.dot(normal, face["declared"]) < -1e-9)
        normal_lines.append("vn " + " ".join(f"{0. if value == 0 else value:.12g}" for value in normal))
        # A corner's UV travels with its vertex; only the normal is replaced.
        corners = [face["corners"][k] for k in order]
        rewritten[face["line"]] = "f " + " ".join(f"{c[0]}/{c[1]}/{i + 1}" for c in corners)
        indices = [face["indices"][k] for k in order]
        a, b, c = vertices[indices]
        volume += float(np.dot(a, np.cross(b, c)) / 6)
        ids = [int(inverse[index]) for index in indices]
        for first, second in zip(ids, ids[1:] + ids[:1]):
            edge_directions[tuple(sorted((first, second)))] += 1 if first < second else -1
        roof_faces += int(normal[2] > .5)
        wall_faces += int(abs(normal[2]) < .25)
        base_faces += int(normal[2] < -.5)
    output, inserted = [], False
    for line_number, line in enumerate(lines):
        if line.split("#", 1)[0].split()[:1] == ["vn"]:
            continue
        if line_number in rewritten:
            if not inserted:
                output.extend(normal_lines)
                inserted = True
            output.append(rewritten[line_number])
        else:
            output.append(line)
    nonmanifold = sum(len(entries) != 2 for entries in edges.values())
    inconsistent = sum(edge_directions[key] != 0 for key, entries in edges.items() if len(entries) == 2)
    return "\n".join(output) + "\n", {
        "vertices": len(vertices), "triangles": len(faces), "flipped_triangles": flipped,
        "roof_triangles": roof_faces, "wall_triangles": wall_faces, "base_triangles": base_faces,
        "source_normal_disagreements_after": normal_disagreements,
        "source_zero_normals": int(sum(np.linalg.norm(n) < 1e-9 for n in normals)),
        "normals_recomputed": True, "orientation_components": components,
        "welded_nonmanifold_edges": nonmanifold, "inconsistent_directed_edges": inconsistent,
        "closed_consistently_oriented_after_welding": nonmanifold == 0 and inconsistent == 0,
        "signed_volume_after_m3_assuming_meter_units": volume,
        "geometry_modified": False, "uv_coordinates_modified": False,
        "note": "Consistent outward orientation and flat normals; source holes / shapes not repaired or certified"}
