import unittest
import numpy as np
from mapnav.mesh import audit_obj_surfaces
from mapnav.photogrammetry import prepare_obj, clip_obj_to_core, smooth_obj_normals


class PhotogrammetryTests(unittest.TestCase):
    def test_smooth_normals_keep_geometry_uv_materials_and_sharp_edge(self):
        source='mtllib tile.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nvt 0 0\nusemtl tile\nf 1/1 2/1 3/1\nf 1/1 4/1 2/1\n'
        fixed, report=smooth_obj_normals(source,35)
        for prefix in ('v ','vt ','usemtl ','mtllib '):
            self.assertEqual([l for l in source.splitlines() if l.startswith(prefix)],
                             [l for l in fixed.splitlines() if l.startswith(prefix)])
        self.assertEqual(report['normals'],2)
        self.assertAlmostEqual(audit_obj_surfaces(fixed)['minimum_winding_normal_cosine'],1)

    def test_smooth_normals_reject_degenerate_mesh(self):
        with self.assertRaises(ValueError):smooth_obj_normals('v 0 0 0\nv 1 0 0\nv 2 0 0\nf 1 2 3\n')
        with self.assertRaises(ValueError):smooth_obj_normals('',90)

    def test_core_clip_interpolates_uv_and_keeps_material(self):
        source = "mtllib tile.mtl\nv -1 0 0\nv 1 0 0\nv 0 2 0\nvt 0 0\nvt 1 0\nvt .5 1\nusemtl roof\nf 1/1 2/2 3/3\n"
        fixed, report = clip_obj_to_core(source, [0, 0, 0], [0, 0, 2, 2])
        self.assertEqual(report["bounds_xyz_m"], [[0., 0., 0.], [1., 2., 0.]])
        self.assertIn("vt 0.5 0", fixed)
        self.assertIn("usemtl roof", fixed)
        self.assertIn("mtllib tile.mtl", fixed)
        self.assertEqual(report["triangles"], 1)
        self.assertAlmostEqual(audit_obj_surfaces(fixed)["minimum_winding_normal_cosine"], 1)

    def test_core_clip_untextured_vertical_triangle(self):
        fixed, report = clip_obj_to_core("v -1 0 0\nv 1 0 0\nv 0 0 2\nf 1 2 3\n", [0, 0, 0], [0, -1, 2, 1])
        self.assertNotIn("vt ", fixed)
        self.assertEqual(report["triangles"], 1)
        self.assertIn("//1", fixed)

    def test_core_clip_quad_triangulates_without_caps(self):
        fixed, report = clip_obj_to_core("v -1 -1 0\nv 2 -1 0\nv 0 2 0\nf 1 2 3\n", [0, 0, 0], [0, 0, 1, 1])
        self.assertGreater(report["triangles"], 1)
        self.assertEqual(report["bounds_xyz_m"][0][2], 0)
        self.assertEqual(report["bounds_xyz_m"][1][2], 0)

    def test_core_clip_rejects_invalid_bounds_and_mixed_uvs(self):
        with self.assertRaises(ValueError): clip_obj_to_core("", [0, 0, 0], [1, 0, 0, 1])
        with self.assertRaises(ValueError):
            clip_obj_to_core("v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nf 1/1 2 3\n", [0, 0, 0], [0, 0, 2, 2])

    def test_translation_preserves_uv_and_material_and_adds_normals(self):
        source = "mtllib tile.mtl\nv 8000 7000 5\nv 8001 7000 5\nv 8000 7001 5\nvt 0 0\nvt 1 0\nvt 0 1\nusemtl tile\nf 1/1 2/2 3/3\n"
        fixed, report = prepare_obj(source, [8000, 7000, 0])
        self.assertIn("v 0.00000000 0.00000000 5.00000000", fixed)
        self.assertIn("f 1/1/1 2/2/1 3/3/1", fixed)
        self.assertIn("usemtl tile", fixed)
        self.assertIn("vt 1 0", fixed)
        self.assertEqual(report["triangles"], 1)
        audit = audit_obj_surfaces(fixed)
        self.assertEqual(audit["minimum_winding_normal_cosine"], 1)

    def test_inverted_terrain_rejected_instead_of_blindly_fixing(self):
        with self.assertRaises(ValueError):
            prepare_obj("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 3 2\n", [0, 0, 0])

    def test_origin_must_be_finite(self):
        with self.assertRaises(ValueError): prepare_obj("", [np.nan, 0, 0])


if __name__ == "__main__": unittest.main()
