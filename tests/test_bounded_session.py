"""Filter-level candidate tests, not native or release qualification."""
import sys,unittest
from pathlib import Path
import numpy as np

from posim_underwater_navigation.prediction import predicted_snapshot
from posim_underwater_navigation.bounded_session import BoundedInertialDvlSession
from posim_underwater_navigation.core.inertial_filter import InertialFilter

CAL=dict(noise_density=[.001,.02,.0001,.001],dvl_to_imu=np.eye(3),imu_to_dvl=[0,0,0])
class CandidateTests(unittest.TestCase):
    def session(self,**limits):
        s=BoundedInertialDvlSession(CAL,**limits)
        s.filter=InertialFilter(stamp=0.,position=[0,0,0],velocity=[.1,0,0],attitude=np.eye(3),
            gyro_bias=[0,0,0],accel_bias=[0,0,0],state_covariance=np.eye(15)*1e-5,
            noise_density=CAL['noise_density'])
        s.state='tracking';s.last_dvl_stamp=0.;s.last_aiding_stamp=0.;s.last_dvl_wall=0.;s.bottom_locked=True
        for t in np.linspace(0,1.,201):s.window.add(float(t),[0,0,0],[0,0,9.80665])
        return s
    def test_loss_propagates_measured_imu_without_zero_dvl_or_ready_claim(self):
        s=self.session();s.bottom_loss(.5,.5);out=predicted_snapshot(s,CAL)
        self.assertEqual(out['navigation_quality'],'inertial_coasting')
        self.assertFalse(out['navigation_ready']);self.assertFalse(out['bottom_locked'])
        np.testing.assert_allclose(out['position'],[.1,0,0],atol=1e-9)
        self.assertGreater(out['covariance'][0][0],s.filter.covariance[0,0])
        self.assertEqual(s.filter.stamp,0.);self.assertEqual(s.accepted,0)
    def test_uncertainty_stops_output_and_discards_segment(self):
        s=self.session(position_three_sigma_m=.001);s.bottom_loss(.5,.5)
        out=predicted_snapshot(s,CAL)
        self.assertEqual(out['reason'],'inertial_confidence_exceeded')
        self.assertIsNone(out['position']);self.assertIsNone(s.filter);self.assertEqual(s.segment,1)
    def test_horizon_not_extended_by_loss_packets(self):
        s=self.session();s.bottom_loss(1.,1.);s.bottom_loss(1.6,1.6)
        self.assertIsNone(s.filter);self.assertEqual(s.reason,'dvl_coast_horizon_exceeded')
    def test_fresh_bottom_lock_recovers_only_with_real_measurement(self):
        s=self.session();s.bottom_loss(.5,.5)
        s.dvl(1.,[.1,0,0],np.eye(3)*.001,True,1.)
        out=predicted_snapshot(s,CAL)
        self.assertTrue(out['bottom_locked']);self.assertEqual(out['navigation_quality'],'dvl_aided')
        self.assertEqual(s.last_aiding_stamp,1.);self.assertEqual(s.accepted,1)
    def test_time_reversal_is_not_coasting(self):
        for stamp in (-1,0,True,float('nan')):
            s=self.session();s.bottom_loss(stamp,1.)
            self.assertIsNone(s.filter)
        s=self.session();s.bottom_loss(.5,.5);s.dvl(.4,[0,0,0],np.eye(3),True,1.)
        self.assertEqual(s.reason,'dvl_reacquisition_time_reversal')
    def test_imu_staleness_stays_fatal(self):
        s=self.session();s.last_imu_wall=0.;s.tick(2.1)
        self.assertEqual(s.reason,'imu_stale');self.assertIsNone(s.filter)
    def test_prediction_timeout_invalidates_owner_not_only_display(self):
        s=self.session()
        for t in np.linspace(1.005,1.6,120):s.window.add(float(t),[0,0,0],[0,0,9.80665])
        out=predicted_snapshot(s,CAL)
        self.assertEqual(out['state'],'lost');self.assertIsNone(s.filter)
    def test_invalid_covariance_rejected(self):
        for bad in (float('nan'),-1.):
            s=self.session();v=predicted_snapshot(s,CAL);v['covariance'][0][0]=bad
            self.assertEqual(s.qualify_prediction(v)['state'],'lost')
    def test_limit_validation(self):
        for v in (0,2.,True,float('nan')):
            with self.assertRaises(ValueError):self.session(max_coast_s=v)

if __name__=='__main__':unittest.main()
