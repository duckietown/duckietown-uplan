import geometry as geo
import numpy as np


def interpolate(q0, q1, alpha):
    v = geo.SE2.algebra_from_group(geo.SE2.multiply(geo.SE2.inverse(q0), q1))
    vi = v * alpha
    q = np.dot(q0, geo.SE2.group_from_algebra(vi))
    return q
