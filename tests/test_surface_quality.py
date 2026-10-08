import unittest
import numpy as np
from mapnav.surface_quality import Surface, stitch_seams, fill_small_planar_holes


def plane(core,z=0,split=False):
    x0,y0,x1,y1=core
    v=[[x0,y0,z],[x1,y0,z],[x1,y1,z],[x0,y1,z]]
    uv=[[0,0],[1,0],[1,1],[0,1]]
    faces=[[0,1,2],[0,2,3]]
    if split:
        v.append([x0,(y0+y1)/2,z]);uv.append([0,.5]);faces=[[0,1,4],[1,2,4],[2,3,4]]
    return Surface(np.array(v,dtype=float),np.array(uv,dtype=float),np.array(faces),np.array(faces),["tile"]*len(faces),["mtllib tile.mtl"],core)


def ring(hole_half=.25):
    lo,hi=2-hole_half,2+hole_half
    v=[[0,0,0],[4,0,0],[4,4,0],[0,4,0],[lo,lo,0],[hi,lo,0],[hi,hi,0],[lo,hi,0]]
    faces=[[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
    return Surface(np.array(v,dtype=float),np.array(v,dtype=float)[:,:2]/4,np.array(faces),np.array(faces),["tile"]*8,["mtllib tile.mtl"],(-1,-1,5,5))


class SurfaceQualityTests(unittest.TestCase):
    def test_obj_roundtrip_preserves_uvs_materials_and_faces(self):
        s=plane((0,0,1,1));r=Surface.read(s.obj(),s.core)
        np.testing.assert_array_equal(r.faces,s.faces)
        np.testing.assert_array_equal(r.uvs,s.uvs)
        self.assertEqual(r.materials,s.materials)

    def test_stitch_splits_t_junction_and_interpolates_uv(self):
        left,right=plane((0,0,1,1)),plane((1,0,2,1),.08,True)
        report=stitch_seams([left,right])
        self.assertEqual(report['matched_seam_intervals'],2)
        self.assertGreater(report['inserted_boundary_vertices'],0)
        for surface in (left,right):
            seam=surface.vertices[np.isclose(surface.vertices[:,0],1)]
            np.testing.assert_allclose(seam[:,2],.04)
            self.assertTrue((np.linalg.norm(surface.cross(),axis=1)>1e-10).all())
        self.assertTrue(any(np.allclose(uv,[1,.5]) for uv in left.uvs))
        self.assertLessEqual(report['maximum_vertex_displacement_m'],.075)

    def test_large_height_difference_is_not_flattened(self):
        a,b=plane((0,0,1,1)),plane((1,0,2,1),3)
        before=[s.vertices.copy() for s in (a,b)]
        report=stitch_seams([a,b])
        self.assertEqual(report['matched_seam_intervals'],0)
        for original,s in zip(before,(a,b)):np.testing.assert_array_equal(original,s.vertices)

    def test_small_planar_hole_is_closed_with_same_uv_material(self):
        s=ring();uv=s.uvs.copy();report=fill_small_planar_holes(s)
        self.assertEqual(report['filled_small_planar_holes'],1)
        self.assertEqual(report['added_triangles'],2)
        self.assertAlmostEqual(report['patches'][0]['area_m2'],.25)
        self.assertEqual(len(s.boundary_edges()[0]),4)
        self.assertTrue((s.cross()[:,2]>0).all())
        np.testing.assert_array_equal(s.uvs,uv)
        self.assertEqual(set(s.materials),{'tile'})

    def test_large_holes_and_tile_cuts_are_not_capped(self):
        s=ring(1.2);before=s.faces.copy();report=fill_small_planar_holes(s)
        self.assertEqual(report['filled_small_planar_holes'],0)
        np.testing.assert_array_equal(s.faces,before)
        p=plane((0,0,1,1));self.assertEqual(fill_small_planar_holes(p)['interior_boundary_loops'],0)

    def test_ambiguous_uv_and_material_seams_are_left_unchanged(self):
        s=ring();s.materials[1]='different';self.assertEqual(fill_small_planar_holes(s)['filled_small_planar_holes'],0)
        s=ring();s.texture_faces[1,2]=0;self.assertEqual(fill_small_planar_holes(s)['filled_small_planar_holes'],0)

    def test_untextured_hole_uses_the_original_untextured_material(self):
        s=ring();s.texture_faces[:]=-1;s.materials=['tile_untextured']*len(s.faces)
        report=fill_small_planar_holes(s)
        self.assertEqual(report['filled_small_planar_holes'],1)
        self.assertTrue(report['patches'][0]['untextured_source_material'])
        self.assertTrue((s.texture_faces<0).all())
        self.assertEqual(set(s.materials),{'tile_untextured'})

    def test_invalid_gap_is_rejected(self):
        for gap in (0,.5,float('nan')):
            with self.assertRaises(ValueError):stitch_seams([],gap)


if __name__=='__main__':unittest.main()
