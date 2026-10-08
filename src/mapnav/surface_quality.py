"""Small, measured city-mesh repairs; no image synthesis or hidden ground.

Seam pairs must agree within a declared tolerance and have similar surface
normals. Small planar interior hole caps are explicitly inferred, not surveyed.
Large, ambiguous or multi-material openings are left unchanged and reported.
"""
from collections import defaultdict
from dataclasses import dataclass
import math
import numpy as np
from .mesh import obj_index


@dataclass
class Surface:
    vertices: np.ndarray
    uvs: np.ndarray
    faces: np.ndarray
    texture_faces: np.ndarray
    materials: list
    libraries: list
    core: tuple

    @classmethod
    def read(cls, text, core):
        vertices, uvs, faces, texture_faces, materials, libraries = [], [], [], [], [], []
        material = None
        for line in text.splitlines():
            tokens = line.split()
            if not tokens:
                continue
            if tokens[0] == "v":
                vertices.append(list(map(float, tokens[1:4])))
            elif tokens[0] == "vt":
                uvs.append(list(map(float, tokens[1:3])))
            elif tokens[0] == "mtllib":
                libraries.append(line)
            elif tokens[0] == "usemtl":
                material = " ".join(tokens[1:])
            elif tokens[0] == "f":
                corners = [t.split("/") for t in tokens[1:]]
                if len(corners) != 3:
                    raise ValueError("Triangulated OBJ required")
                faces.append([obj_index(c[0], len(vertices)) for c in corners])
                texture_faces.append([obj_index(c[1], len(uvs)) if len(c)>1 and c[1] else -1 for c in corners])
                materials.append(material)
        v, uv, f, tf = np.asarray(vertices, dtype=float), np.asarray(uvs, dtype=float).reshape(-1, 2), np.asarray(faces, dtype=int), np.asarray(texture_faces, dtype=int)
        if v.ndim != 2 or v.shape[1] != 3 or not len(f) or not np.isfinite(v).all() or not np.isfinite(uv).all():
            raise ValueError("Finite nonempty mesh required")
        return cls(v, uv, f, tf, materials, libraries, tuple(core))

    def cross(self):
        p = self.vertices[self.faces]
        return np.cross(p[:, 1]-p[:, 0], p[:, 2]-p[:, 0])

    def boundary_edges(self):
        directed = self.faces[:, ((0, 1), (1, 2), (2, 0))].reshape(-1, 2)
        keys = np.sort(directed, axis=1)
        _, first, counts = np.unique(keys, axis=0, return_index=True, return_counts=True)
        singles = first[counts == 1]
        return [(int(directed[i, 0]), int(directed[i, 1]), int(i // 3)) for i in singles], int((counts > 2).sum())

    def obj(self):
        cross = self.cross(); lengths = np.linalg.norm(cross, axis=1)
        if (lengths <= 1e-10).any():
            raise ValueError("Repair produced a zero-area triangle")
        normals = cross / lengths[:, None]
        output = ["# Measured local surface repair; original UV atlas and materials retained", *self.libraries]
        output.extend("v " + " ".join(f"{p:.8f}" for p in xyz) for xyz in self.vertices)
        output.extend("vt " + " ".join(f"{p:.12g}" for p in uv) for uv in self.uvs)
        output.extend("vn " + " ".join(f"{p:.12g}" for p in n) for n in normals)
        output.append("s off"); previous = None
        for i, (face, uv, material) in enumerate(zip(self.faces, self.texture_faces, self.materials)):
            if material != previous and material is not None:
                output.append("usemtl " + material)
            previous = material
            output.append("f " + " ".join(f"{v+1}/{t+1 if t>=0 else ''}/{i+1}" for v, t in zip(face, uv)))
        return "\n".join(output) + "\n"


def stitch_seams(surfaces, max_gap_m=.15, normal_angle_deg=25):
    """Match boundary segments and split T-junctions with interpolated UVs.

    Only mutually unambiguous pairings are used. Large roof/tree height changes
    and layer ambiguities are not collapsed. Each vertex moves at most half the
    matching gap; corner conflicts reject their whole seam group.
    """
    if not math.isfinite(max_gap_m) or not 0 < max_gap_m <= .25:
        raise ValueError("Seam gap must be finite and in (0, .25] metres")
    groups = defaultdict(lambda: {"minus": [], "plus": []})
    for si, surface in enumerate(surfaces):
        boundaries, _ = surface.boundary_edges()
        cross = surface.cross(); unit = cross / np.linalg.norm(cross, axis=1)[:, None]
        core = surface.core
        for axis, bound, side in ((0,core[0],"plus"),(0,core[2],"minus"),(1,core[1],"plus"),(1,core[3],"minus")):
            key = axis, bound, core[1-axis], core[3-axis]
            for a, b, fi in boundaries:
                p, q = surface.vertices[[a,b]]
                if max(abs(p[axis]-bound), abs(q[axis]-bound)) > 1e-8 or abs(p[1-axis]-q[1-axis]) <= 1e-8:
                    continue
                groups[key][side].append((si,a,b,fi,p.copy(),q.copy(),unit[fi]))
    edits = defaultdict(dict); vertex_moves = {}; matched = skipped = conflicts = rejected_geometry = 0; max_before = 0.
    cosine = math.cos(math.radians(normal_angle_deg))
    for key, sides in groups.items():
        if not sides["minus"] or not sides["plus"]:
            continue
        axis, bound, _, _ = key; other = 1-axis
        events = sorted(set(round(float(e[j][other]),8) for side in sides.values() for e in side for j in (4,5)))
        proposals = []; local_moves = {}; valid_group = True
        for lo, hi in zip(events, events[1:]):
            if hi-lo <= 1e-8:
                continue
            mid = (lo+hi)/2
            active = {side:[e for e in entries if min(e[4][other],e[5][other])-1e-9 <= mid <= max(e[4][other],e[5][other])+1e-9] for side,entries in sides.items()}
            def point(e,t):
                alpha = (t-e[4][other])/(e[5][other]-e[4][other])
                p = e[4]+alpha*(e[5]-e[4]); p[axis]=bound; p[other]=t
                return p,alpha
            candidates = []
            for i,a in enumerate(active["minus"]):
                for j,b in enumerate(active["plus"]):
                    if float(np.dot(a[6],b[6])) < cosine:
                        continue
                    pa=[point(a,t)[0] for t in (lo,hi)]; pb=[point(b,t)[0] for t in (lo,hi)]
                    delta = max(abs(p[2]-q[2]) for p,q in zip(pa,pb))
                    if delta <= max_gap_m:
                        candidates.append((i,j,a,b,pa,pb,delta))
            for i,j,a,b,pa,pb,delta in candidates:
                if sum(c[0]==i for c in candidates)!=1 or sum(c[1]==j for c in candidates)!=1:
                    skipped += 1; continue
                for t,p,q in zip((lo,hi),pa,pb):
                    target = (p+q)/2
                    for edge in (a,b):
                        _,alpha=point(edge,t)
                        if alpha <= 1e-8 or alpha >= 1-1e-8:
                            vid=edge[1] if alpha<=1e-8 else edge[2]
                            vk=(edge[0],vid)
                            previous=local_moves.get(vk,vertex_moves.get(vk))
                            if previous is not None and np.linalg.norm(previous-target)>1e-6:
                                valid_group=False
                            local_moves[vk]=target
                        proposals.append((edge,alpha,target))
                max_before=max(max_before,delta)
        if not valid_group:
            conflicts += 1; continue
        # Transactional guard: seam welding must not collapse a thin source
        # face or reverse a fan triangle. Reject the entire paired group so
        # neither side receives a half-applied repair.
        proposed_edges=defaultdict(dict)
        affected=defaultdict(set)
        for edge,alpha,target in proposals:
            si,a,b,fi=edge[:4]
            proposed_edges[si].setdefault((a,b,fi),{})[round(float(alpha),12)]=target
            affected[si].add(fi)
        for si in affected:
            surface=surfaces[si]
            moved_ids=[vi for s,vi in local_moves if s==si]
            if moved_ids:
                affected[si].update(np.flatnonzero(np.isin(surface.faces,moved_ids).any(axis=1)).tolist())
            for fi in affected[si]:
                face=surface.faces[fi]
                corners=np.array([local_moves.get((si,int(v)),vertex_moves.get((si,int(v)),surface.vertices[v])) for v in face])
                polygon=[];has_insert=False
                for corner in range(3):
                    a,b=int(face[corner]),int(face[(corner+1)%3])
                    polygon.append(corners[corner])
                    points=dict(edits[si].get((a,b,fi),{}));points.update(proposed_edges[si].get((a,b,fi),{}))
                    for alpha,target in sorted(points.items()):
                        if 1e-8<alpha<1-1e-8:
                            polygon.append(target);has_insert=True
                original=surface.vertices[face]
                reference=np.cross(original[1]-original[0],original[2]-original[0]);reference/=np.linalg.norm(reference)
                if has_insert:
                    center=corners.mean(axis=0)
                    triangles=[(center,polygon[k],polygon[(k+1)%len(polygon)]) for k in range(len(polygon))]
                else:
                    triangles=[corners]
                for triangle in triangles:
                    a,b,c=triangle;cross=np.cross(b-a,c-a);length=float(np.linalg.norm(cross))
                    if length<=1e-10 or float(cross@reference)<.5*length:
                        valid_group=False;break
                if not valid_group:break
            if not valid_group:break
        if not valid_group:
            rejected_geometry+=1;continue
        for edge,alpha,target in proposals:
            si,a,b,fi= edge[:4]
            edits[si].setdefault((a,b,fi),{})[round(float(alpha),12)]=target
        vertex_moves.update(local_moves)
        matched += len(proposals)//4
    inserted = split_faces = 0; moved = 0; max_movement = 0.
    for si,surface in enumerate(surfaces):
        original_v = surface.vertices.copy()
        for (s,vi),target in vertex_moves.items():
            if s==si:
                displacement=float(np.linalg.norm(target-original_v[vi]))
                if displacement > max_gap_m/2+1e-6:
                    raise ValueError("Seam displacement exceeded declared bound")
                surface.vertices[vi]=target
                moved+=int(displacement>1e-8); max_movement=max(max_movement,displacement)
        additions_v,additions_uv=[],[]; face_edits=defaultdict(dict)
        for (a,b,fi),points in edits[si].items():
            face_edits[fi][(a,b)]=points
        faces=[];texture_faces=[];materials=[]
        for fi,(face,uv,material) in enumerate(zip(surface.faces,surface.texture_faces,surface.materials)):
            polygon=[]; poly_uv=[]; has_insert=False
            for corner in range(3):
                a,b=int(face[corner]),int(face[(corner+1)%3]); ta,tb=int(uv[corner]),int(uv[(corner+1)%3])
                polygon.append(a);poly_uv.append(ta)
                for alpha,target in sorted(face_edits.get(fi,{}).get((a,b),{}).items()):
                    if not 1e-8 < alpha < 1-1e-8:
                        continue
                    vi=len(surface.vertices)+len(additions_v); additions_v.append(target)
                    ti=-1
                    if ta>=0 and tb>=0:
                        ti=len(surface.uvs)+len(additions_uv)
                        additions_uv.append(surface.uvs[ta]+alpha*(surface.uvs[tb]-surface.uvs[ta]))
                    polygon.append(vi);poly_uv.append(ti);has_insert=True;inserted+=1
            if not has_insert:
                faces.append(face);texture_faces.append(uv);materials.append(material);continue
            # A center fan keeps every collinear boundary split, including UV seams.
            center=len(surface.vertices)+len(additions_v)
            additions_v.append(surface.vertices[face].mean(axis=0))
            center_uv=-1
            if (uv>=0).all():
                center_uv=len(surface.uvs)+len(additions_uv); additions_uv.append(surface.uvs[uv].mean(axis=0))
            for k in range(len(polygon)):
                faces.append((center,polygon[k],polygon[(k+1)%len(polygon)]))
                texture_faces.append((center_uv,poly_uv[k],poly_uv[(k+1)%len(polygon)]));materials.append(material)
            split_faces+=1
        if additions_v:
            surface.vertices=np.vstack((surface.vertices,additions_v))
        if additions_uv:
            surface.uvs=np.vstack((surface.uvs,additions_uv))
        surface.faces=np.asarray(faces,dtype=int);surface.texture_faces=np.asarray(texture_faces,dtype=int);surface.materials=materials
        if (np.linalg.norm(surface.cross(),axis=1)<=1e-10).any():
            raise ValueError("Seam repair produced degenerate geometry")
    return {"matched_seam_intervals":matched,"ambiguous_interval_pairs_skipped":skipped,
            "conflicting_boundary_groups_skipped":conflicts,"split_faces":split_faces,
            "thin_or_inverted_face_groups_skipped":rejected_geometry,
            "inserted_boundary_vertices":inserted,"moved_vertices":moved,
            "maximum_matched_height_gap_before_m":max_before,"maximum_vertex_displacement_m":max_movement,
            "max_gap_m":max_gap_m,"normal_angle_deg":normal_angle_deg}


def interior_loops(surface):
    edges,nonmanifold=surface.boundary_edges();out=defaultdict(list);incoming=defaultdict(list)
    for a,b,fi in edges:
        out[a].append((b,fi));incoming[b].append(a)
    seen=set();loops=[]
    for seed in out:
        if seed in seen or len(out[seed])!=1 or len(incoming[seed])!=1:
            continue
        chain=[];faces=[];p=seed
        while p not in seen and len(out[p])==1 and len(incoming[p])==1:
            seen.add(p);chain.append(p);q,fi=out[p][0];faces.append(fi);p=q
        if p!=seed or len(chain)<3:
            continue
        xyz=surface.vertices[chain];c=surface.core
        # Border loops are tile cuts, not interior holes.
        if any(np.any(np.abs(xyz[:,axis]-bound)<1e-6) for axis,bound in ((0,c[0]),(0,c[2]),(1,c[1]),(1,c[3]))):
            continue
        loops.append((chain,faces))
    return loops,len(edges),nonmanifold


def triangulate_polygon(points):
    """Ear-clipping without discarding collinear boundary vertices."""
    def cross(a,b,c):
        u,v=b-a,c-a;return u[0]*v[1]-u[1]*v[0]
    points=np.asarray(points);n=len(points)
    area=sum(points[i,0]*points[(i+1)%n,1]-points[(i+1)%n,0]*points[i,1] for i in range(n))/2
    if abs(area)<=1e-10:
        return []
    sign=1 if area>0 else -1;pending=list(range(n));triangles=[]
    for _ in range(n*n):
        if len(pending)<=3:
            break
        found=False
        for j in range(len(pending)):
            a,b,c=pending[j-1],pending[j],pending[(j+1)%len(pending)]
            if sign*cross(points[a],points[b],points[c])<=1e-10:
                continue
            if any(min(sign*cross(points[a],points[b],points[k]),sign*cross(points[b],points[c],points[k]),sign*cross(points[c],points[a],points[k]))>=-1e-10 for k in pending if k not in (a,b,c)):
                continue
            triangles.append((a,b,c));pending.pop(j);found=True;break
        if not found:
            return []
    if len(pending)!=3 or sign*cross(*points[pending])<=1e-10:
        return []
    triangles.append(tuple(pending));return triangles


def fill_small_planar_holes(surface,max_area_m2=1,max_diameter_m=2,max_planarity_m=.02):
    """Cap only small interior loops with coherent existing material/UVs.

    The result is local interpolation, not new measured geometry. No large
    facade, passage, tile cut or source-boundary cap is created.
    """
    loops,boundary,nonmanifold=interior_loops(surface);accepted=[];rejected=defaultdict(int)
    cross=surface.cross();unit=cross/np.linalg.norm(cross,axis=1)[:,None]
    new_faces=[];new_uv=[];new_materials=[]
    for chain,incident in loops:
        xyz=surface.vertices[chain]
        if len(chain)>32:
            rejected["large_or_complex"]+=1;continue
        diameter=float(np.linalg.norm(xyz[:,None,:]-xyz[None,:,:],axis=2).max())
        if diameter>max_diameter_m:
            rejected["large_or_complex"]+=1;continue
        _,_,vh=np.linalg.svd(xyz-xyz.mean(axis=0),full_matrices=False)
        plane=vh[-1];planarity=float(np.abs((xyz-xyz.mean(axis=0))@plane).max())
        if planarity>max_planarity_m:
            rejected["nonplanar"]+=1;continue
        if len({surface.materials[fi] for fi in incident})!=1:
            rejected["material_seam"]+=1;continue
        texture_ids={}
        untextured=all((surface.texture_faces[fi]<0).all() for fi in incident)
        for vi in chain:
            tids={int(surface.texture_faces[fi,list(surface.faces[fi]).index(vi)]) for fi in incident if vi in surface.faces[fi]}
            if len(tids)!=1 or (next(iter(tids))<0 and not untextured):
                break
            texture_ids[vi]=next(iter(tids))
        if len(texture_ids)!=len(chain):
            rejected["ambiguous_uv"]+=1;continue
        # Reverse the existing oriented hole rim to close it consistently.
        chain=list(reversed(chain));xyz=surface.vertices[chain]
        drop=int(np.argmax(np.abs(plane)));flat=np.delete(xyz,drop,axis=1)
        cap=triangulate_polygon(flat)
        if not cap:
            rejected["untriangulatable"]+=1;continue
        tri_xyz=xyz[np.asarray(cap)];cc=np.cross(tri_xyz[:,1]-tri_xyz[:,0],tri_xyz[:,2]-tri_xyz[:,0]);length=np.linalg.norm(cc,axis=1)
        area=float(length.sum()/2);vote=unit[incident].mean(axis=0);vote_length=float(np.linalg.norm(vote))
        if vote_length<=1e-8:
            rejected["ambiguous_normal"]+=1;continue
        vote/=vote_length
        if area>max_area_m2 or (length<=1e-10).any() or (cc@vote/length<.95).any():
            rejected["area_or_normal_disagreement"]+=1;continue
        if (unit[incident]@vote<.9).any():
            rejected["surface_crease"]+=1;continue
        for triangle in cap:
            ids=[chain[i] for i in triangle]
            new_faces.append(ids);new_uv.append([texture_ids[i] for i in ids]);new_materials.append(surface.materials[incident[0]])
        accepted.append({"centroid_xyz_m":xyz.mean(axis=0).tolist(),"area_m2":area,
                         "diameter_m":diameter,"planarity_residual_m":planarity,
                         "added_triangles":len(cap),"untextured_source_material":untextured,"inferred_geometry":True})
    if new_faces:
        surface.faces=np.vstack((surface.faces,new_faces));surface.texture_faces=np.vstack((surface.texture_faces,new_uv));surface.materials.extend(new_materials)
    return {"interior_boundary_loops":len(loops),"boundary_edges_before":boundary,
            "nonmanifold_edges":nonmanifold,"filled_small_planar_holes":len(accepted),
            "added_triangles":len(new_faces),"rejected_loop_reasons":dict(rejected),"patches":accepted,
            "limits":{"max_area_m2":max_area_m2,"max_diameter_m":max_diameter_m,"max_planarity_m":max_planarity_m}}
