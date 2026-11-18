"""Read-only prediction of a session using only its buffered IMU measurements."""
from copy import deepcopy
import numpy as np


def predicted_snapshot(session, calibration):
    result=session.snapshot()
    if session.filter is None:
        return result
    try:
        endpoint=session.window.samples[-1][0]
        horizon=endpoint-session.filter.stamp
        if not 0<=horizon<=1.5:
            raise ValueError('dvl_prediction_horizon_exceeded')
        predicted=deepcopy(session.filter)
        predicted.propagate(session.window,endpoint)
        result.update(stamp=predicted.stamp,position=predicted.position.tolist(),
                      attitude=predicted.attitude.tolist(),velocity=predicted.velocity.tolist(),
                      covariance=predicted.covariance.tolist(),last_dvl_stamp=session.filter.stamp,
                      prediction='received_raw_imu')
        times,measurements=session.window.interval(endpoint-.01,endpoint)
        result['angular_velocity']=(measurements[-1,:3]-predicted.gyro_bias).tolist()
        result['angular_variance']=float(calibration['noise_density'][0])**2/float(np.min(np.diff(times)))
    except (ValueError,IndexError) as error:
        result.update(state='lost',reason=str(error),stamp=None,position=None,attitude=None,
                      covariance=None,inertial_fused=False,navigation_ready=False)
    if hasattr(session,'qualify_prediction'):
        result=session.qualify_prediction(result)
    return result
