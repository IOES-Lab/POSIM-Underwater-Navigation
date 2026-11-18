import unittest
import numpy as np
from posim_underwater_navigation.gnss_alignment import align_planar_tracks

class AlignmentTests(unittest.TestCase):
    def test_measured_rigid_alignment_and_no_ready_claim(self):
        x=np.array([[i*3.,np.sin(i)] for i in range(10)])
        theta=.7;r=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        result=align_planar_tracks(x,x@r.T+[10,20],[.49]*10)
        np.testing.assert_allclose(result['rotation'],r,atol=1e-12)
        np.testing.assert_allclose(result['translation'],[10,20],atol=1e-12)
        self.assertFalse(result['navigation_ready'])
    def test_scale_mismatch_cannot_be_fitted_away(self):
        x=np.array([[i*3.,0.] for i in range(10)])
        with self.assertRaises(ValueError):align_planar_tracks(x,2*x,[.49]*10)
    def test_insufficient_motion_and_bad_measurements(self):
        x=np.array([[i*.1,0.] for i in range(10)])
        for y,v in ((x,[.49]*10),(x,[0.]*10),(x,[float('nan')]*10)):
            with self.assertRaises(ValueError):align_planar_tracks(x,y,v)

if __name__=='__main__':unittest.main()
