import math


def _quat_mul(a, b):
    a1, a2, a3, a0 = a
    b1, b2, b3, b0 = b
    return [
        a0*b1 + a1*b0 + a2*b3 - a3*b2,
        a0*b2 - a1*b3 + a2*b0 + a3*b1,
        a0*b3 + a1*b2 - a2*b1 + a3*b0,
        a0*b0 - a1*b1 - a2*b2 - a3*b3,
    ]


def _quat_conj(q):
    return [-q[0], -q[1], -q[2], q[3]]


def _to_jaw_frame(r_jaw, jaw_pos, p_world):
    d = [p_world[k] - jaw_pos[k] for k in range(3)]
    return [
        r_jaw[0]*d[0] + r_jaw[3]*d[1] + r_jaw[6]*d[2],
        r_jaw[1]*d[0] + r_jaw[4]*d[1] + r_jaw[7]*d[2],
        r_jaw[2]*d[0] + r_jaw[5]*d[1] + r_jaw[8]*d[2],
    ]
