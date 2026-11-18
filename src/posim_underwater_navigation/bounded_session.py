"""Experimental, measured-IMU-only coast for explicitly selected moving sessions.

Not a geographic or vehicle qualification. Defaults and known-start sessions
retain their existing fail-closed behavior. No synthetic zero DVL measurement is
created during bottom-lock loss. Clock/malformed-input faults remain fatal.
"""
import math
import numpy as np
from .core.inertial_session import InertialDvlSession


class BoundedInertialDvlSession(InertialDvlSession):
    def __init__(self, calibration, *, max_coast_s=1.5, position_three_sigma_m=5.,
                 velocity_three_sigma_m_s=.5, attitude_three_sigma_deg=5., **kwargs):
        super().__init__(calibration, **kwargs)
        self.coast_limits=(max_coast_s, position_three_sigma_m,
                           velocity_three_sigma_m_s, attitude_three_sigma_deg)
        ceilings=(1.5,25.,1.,10.)
        if any(isinstance(x,bool) or not math.isfinite(x) or not 0<x<=c
               for x,c in zip(self.coast_limits,ceilings)):
            raise ValueError('invalid_coast_limits')
        self.bottom_locked=False
        self.last_loss_stamp=None
        self.last_aiding_stamp=None

    def invalidate(self, reason):
        self.bottom_locked=False
        self.last_loss_stamp=None
        self.last_aiding_stamp=None
        super().invalidate(reason)

    def bottom_loss(self, stamp, now):
        if not self.valid_wall_time(now):
            self.invalidate('invalid_sensor_receipt_time');return
        if (isinstance(stamp,bool) or not isinstance(stamp,(int,float))
                or not math.isfinite(stamp) or stamp<0
                or (self.last_dvl_stamp is not None and stamp<=self.last_dvl_stamp)
                or (self.last_loss_stamp is not None and stamp<=self.last_loss_stamp)):
            self.invalidate('invalid_dvl_loss_time');return
        self.last_loss_stamp=stamp
        self.bottom_locked=False
        if self.filter is None:
            self.invalidate('dvl_bottom_tracking_lost');return
        if stamp-self.filter.stamp>self.coast_limits[0]:
            self.invalidate('dvl_coast_horizon_exceeded');return
        self.reason='inertial_coasting_bottom_lock_lost'

    def dvl(self, stamp, velocity, uncertainty, locked, now):
        if (isinstance(stamp,bool) or not isinstance(stamp,(int,float))
                or not math.isfinite(stamp) or stamp<0):
            self.invalidate('invalid_dvl_stamp');return
        if self.last_loss_stamp is not None and stamp<=self.last_loss_stamp:
            self.invalidate('dvl_reacquisition_time_reversal');return
        super().dvl(stamp,velocity,uncertainty,locked,now)

    def flush(self):
        # A DVL packet may arrive before its IMU endpoint. Initialization and
        # accepted updates can therefore occur in imu(), not only in dvl().
        previous=self.filter
        previous_stamp=previous.stamp if previous is not None else None
        super().flush()
        if self.filter is None:return
        if previous is None:
            self.last_aiding_stamp=self.filter.stamp
            self.bottom_locked=self.last_loss_stamp is None or self.filter.stamp>self.last_loss_stamp
            if self.bottom_locked:self.last_loss_stamp=None
        elif self.filter.stamp!=previous_stamp:
            accepted=bool(self.last_update and self.last_update.get('accepted'))
            self.bottom_locked=accepted and (self.last_loss_stamp is None or self.filter.stamp>self.last_loss_stamp)
            if accepted:self.last_aiding_stamp=self.filter.stamp
            if self.bottom_locked:self.last_loss_stamp=None

    def qualify_prediction(self, value):
        """Bound actual propagated covariance, never the last DVL covariance."""
        if value.get('state')=='lost' and self.filter is not None:
            self.invalidate(value.get('reason','prediction_lost'))
        if self.filter is None or value.get('state')!='tracking':return value
        try:
            age=float(value['stamp'])-float(self.last_aiding_stamp)
            c=np.asarray(value['covariance'],dtype=float)
            if c.shape!=(15,15) or not np.isfinite(c).all() or not np.allclose(c,c.T,atol=1e-10):
                raise ValueError('coast_covariance_invalid')
            if np.linalg.eigvalsh(c).min() < -1e-10:raise ValueError('coast_covariance_invalid')
            bounds=[3*math.sqrt(max(0.,float(np.linalg.eigvalsh(c[a:a+3,a:a+3]).max())))
                    for a in (0,3,6)]
            bounds[2]=math.degrees(bounds[2])
            if not math.isfinite(age) or not 0<=age<=self.coast_limits[0]:
                raise ValueError('dvl_coast_horizon_exceeded')
            if any(x>limit for x,limit in zip(bounds,self.coast_limits[1:])):
                raise ValueError('inertial_confidence_exceeded')
        except (ValueError,KeyError,TypeError) as error:
            self.invalidate(str(error))
            return dict(value,state='lost',reason=str(error),stamp=None,position=None,
                        attitude=None,covariance=None,inertial_fused=False,navigation_ready=False)
        return dict(value,bottom_locked=self.bottom_locked,
                    navigation_quality='dvl_aided' if self.bottom_locked else 'inertial_coasting',
                    position_three_sigma_m=bounds[0],velocity_three_sigma_m_s=bounds[1],
                    attitude_three_sigma_deg=bounds[2],dvl_age_s=age,navigation_ready=False)
