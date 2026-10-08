import unittest
from mapnav.mesh import audit_obj_surfaces, correct_obj_winding


class MeshTests(unittest.TestCase):
    def test_reverses_roof_and_preserves_uv_corners(self):
        text = "mtllib roof.mtl\nv 0 0 1\nv 1 0 1\nv 0 1 1\nvt 0 0\nvt 1 0\nvt 0 1\nvn 0 0 1\nusemtl roof\nf 1/1/1 3/3/1 2/2/1\n"
        fixed, report = correct_obj_winding(text)
        self.assertIn("f 1/1/1 2/2/1 3/3/1", fixed)
        self.assertEqual(report["flipped_triangles"], 1)
        self.assertEqual(report["roof_triangles"], 1)
        self.assertFalse(report["geometry_modified"])
        for line in text.splitlines():
            if not line.startswith(("f ", "vn ")): self.assertIn(line, fixed.splitlines())
        _, again = correct_obj_winding(fixed)
        self.assertEqual(again["flipped_triangles"], 0)

    def test_negative_indices_and_missing_normals(self):
        fixed, report = correct_obj_winding("v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 1\nf -3//-1 -2//-1 -1//-1\n")
        self.assertEqual(report["flipped_triangles"], 0)
        with self.assertRaises(ValueError): correct_obj_winding("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")

    def test_degenerate_rejected(self):
        with self.assertRaises(ValueError): correct_obj_winding("v 0 0 0\nv 1 0 0\nv 2 0 0\nvn 0 0 1\nf 1//1 2//1 3//1\n")

    def test_zero_corner_normals_recomputed(self):
        fixed, report = correct_obj_winding("v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 0\nvn 0 0 1\nf 1//1 3//1 2//2\n")
        self.assertIn("vn 0 0 1", fixed)
        self.assertEqual(report["source_zero_normals"], 1)
        self.assertEqual(report["flipped_triangles"], 1)

    def test_closed_component_uses_topology_despite_bad_normals(self):
        # A tetrahedron, with one triangle reversed and all source normals wrong.
        text = "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nvn 0 0 -1\nf 1//1 3//1 2//1\nf 1//1 2//1 4//1\nf 1//1 3//1 4//1\nf 2//1 3//1 4//1\n"
        fixed, report = correct_obj_winding(text)
        self.assertTrue(report["closed_consistently_oriented_after_welding"])
        self.assertAlmostEqual(report["signed_volume_after_m3_assuming_meter_units"], 1 / 6)
        self.assertEqual(report["flipped_triangles"], 1)
        again, second = correct_obj_winding(fixed)
        self.assertEqual(second["flipped_triangles"], 0)
        self.assertEqual(fixed, again)

    def test_audit_reports_open_boundary_and_aligned_normals(self):
        report = audit_obj_surfaces("v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 1\nf 1//1 2//1 3//1\n")
        self.assertEqual(report["boundary_edges"], 3)
        self.assertEqual(report["minimum_winding_normal_cosine"], 1)
        self.assertAlmostEqual(report["boundary_components"][0]["area_m2_assuming_meter_units"], .5)
        self.assertFalse(report["closed_consistently_oriented_after_welding"])

    def test_audit_detects_wrong_normals_without_changing_mesh(self):
        report = audit_obj_surfaces("v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 -2\nf 1//1 2//1 3//1\n")
        self.assertEqual(report["non_unit_normal_faces"], 1)
        self.assertEqual(report["minimum_winding_normal_cosine"], -1)

    def test_audit_rejects_missing_uv_reference(self):
        with self.assertRaises(ValueError):
            audit_obj_surfaces("v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nvn 0 0 1\nf 1/1/1 2/2/1 3/1/1\n")


if __name__ == "__main__": unittest.main()
