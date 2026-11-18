"""Validated SO(3) and three-vector primitives; no visual SLAM dependency."""
import numpy as np

def rotation(value):
    value=np.asarray(value,dtype=float)
    if (value.shape!=(3,3) or not np.isfinite(value).all()
        or not np.allclose(value.T@value,np.eye(3),atol=1e-5)
        or abs(np.linalg.det(value)-1)>1e-5):raise ValueError('invalid_rotation')
    return value


def vector(value):
    value=np.asarray(value,dtype=float)
    if value.shape!=(3,) or not np.isfinite(value).all():raise ValueError('invalid_vector')
    return value
