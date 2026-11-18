"""Candidate rigid horizontal alignment of time-paired measured trajectories.

The caller supplies metric ENU GNSS samples, corrected antenna lever arms and
matching sensor timestamps. This function neither projects geodetic coordinates
nor qualifies an estimator for vehicle control. It never estimates a scale.
"""
import numpy as np


def align_planar_tracks(local, gnss_enu, variance, *, minimum_baseline_m=15.,
                        maximum_residual_m=3.):
    x=np.asarray(local,dtype=float);y=np.asarray(gnss_enu,dtype=float)
    v=np.asarray(variance,dtype=float)
    if (x.ndim!=2 or x.shape[1:]!=(2,) or x.shape!=y.shape or len(x)<8
            or v.shape!=(len(x),) or not all(np.isfinite(a).all() for a in (x,y,v))
            or np.any(v<=0)):
        raise ValueError('invalid_alignment_measurements')
    if (not np.isfinite([minimum_baseline_m,maximum_residual_m]).all()
            or minimum_baseline_m<=0 or maximum_residual_m<=0):
        raise ValueError('invalid_alignment_limits')
    weights=1/v;weights/=weights.sum()
    center_x=weights@x;center_y=weights@y
    a=x-center_x;b=y-center_y
    baseline=np.max(np.linalg.norm(x[:,None,:]-x[None,:,:],axis=2))
    if baseline<minimum_baseline_m:raise ValueError('insufficient_motion_baseline')
    left,_,right=np.linalg.svd((a*weights[:,None]).T@b)
    correction=np.eye(2);correction[-1,-1]=np.linalg.det(right.T@left.T)
    rotation=right.T@correction@left.T
    translation=center_y-rotation@center_x
    residuals=np.linalg.norm(x@rotation.T+translation-y,axis=1)
    rms=float(np.sqrt(weights@(residuals**2)))
    if rms>maximum_residual_m or residuals.max()>2*maximum_residual_m:
        raise ValueError('inconsistent_alignment')
    energy=float(weights@np.sum(a*a,axis=1))
    yaw_variance=float(np.mean(v)/(len(x)*energy))
    if yaw_variance>np.deg2rad(10)**2:raise ValueError('heading_uncertainty_exceeded')
    return dict(rotation=rotation,translation=translation,residual_rms_m=rms,
                yaw_variance_rad2=yaw_variance,anchor_variance_m2=float(np.mean(v)+rms*rms),
                baseline_m=float(baseline),navigation_ready=False)
