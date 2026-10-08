"""X-SCARA geometry and analytic feedforward, matching the MATLAB model."""
import hashlib
import json
import math

PHYSICAL_KEYS = ('L1', 'L2', 'base', 'toolOffset', 'paperHalfSize', 'jointLimits',
                 'zLimits', 'mass1', 'mass2', 'massTool', 'zMass', 'rotorInertia',
                 'viscous', 'coulomb', 'gravity', 'brushK', 'brushC', 'brushMu',
                 'penLift', 'lineWidth0', 'actuatorLimits', 'dt')


def fingerprint(p):
    data = {key: p[key] for key in PHYSICAL_KEYS}
    return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


def fk(q, p):
    s, e, z = q
    a = s + e
    ox, oy = p['toolOffset']
    return [p['base'][0] - p['L1'] * math.sin(s) - p['L2'] * math.sin(a)
            + ox * math.cos(a) - oy * math.sin(a),
            p['base'][1] + p['L1'] * math.cos(s) + p['L2'] * math.cos(a)
            + ox * math.sin(a) + oy * math.cos(a), z]


def ik(xyz, p, branch=1, check_limits=True):
    if branch not in (-1, 1) or not all(math.isfinite(v) for v in xyz):
        raise ValueError('IK needs finite XYZ and branch +1 or -1.')
    dx, dy = xyz[0] - p['base'][0], xyz[1] - p['base'][1]
    ox, oy = p['toolOffset']
    outer = math.hypot(ox, p['L2'] + oy)
    delta = math.atan2(-ox, p['L2'] + oy)
    r2 = dx * dx + dy * dy
    if r2 < 1e-18 or outer <= 0 or p['L1'] <= 0:
        raise ValueError('Unreachable or undefined folded origin.')
    c = (r2 - p['L1']**2 - outer**2) / (2 * p['L1'] * outer)
    if abs(c) > 1 + 1e-10:
        raise ValueError('Point outside SCARA workspace.')
    effective = branch * math.acos(max(-1.0, min(1.0, c)))
    if abs(math.sin(effective)) < p['singularityMargin']:
        raise ValueError('Point too close to a kinematic singularity.')
    s = math.atan2(-dx, dy) - math.atan2(outer * math.sin(effective),
                                       p['L1'] + outer * math.cos(effective))
    q = [s, effective - delta, xyz[2]]
    if check_limits:
        limits = p['jointLimits'] + [p['zLimits']]
        if any(v < lo - 1e-10 or v > hi + 1e-10 for v, (lo, hi) in zip(q, limits)):
            raise ValueError('Reference exceeds assumed joint/Z limits.')
    return q


def motor_coordinates(q, p):
    a, b = q[0], q[1] + q[0] / p['elbowCrosstalkRatio']
    return [a, b, q[2]], [round(math.degrees(a) * p['stepsPerDegree'][0]),
                            round(math.degrees(b) * p['stepsPerDegree'][1]),
                            round(q[2] * p['stepsPerMeterZ'])]


def feedforward(q, dq, ddq, p, enabled=True):
    """Joint torques + Z force; contact feedforward is an analytic estimate."""
    result = [0.0, 0.0, p['zMass'] * p['gravity']]
    if not enabled:
        return result
    l, c1, c2 = p['L1'], p['L1'] / 2, p['L2'] / 2
    outer = math.hypot(p['toolOffset'][0], p['L2'] + p['toolOffset'][1])
    delta = math.atan2(-p['toolOffset'][0], p['L2'] + p['toolOffset'][1])
    b = p['mass2'] * l * c2 * math.cos(q[1]) + p['massTool'] * l * outer * math.cos(q[1] + delta)
    a = p['mass1'] * c1*c1 + p['mass1'] * l*l/12 + p['mass2'] * (l*l + c2*c2) + p['mass2'] * p['L2']**2/12 + p['massTool'] * (l*l + outer*outer)
    d = p['mass2'] * c2*c2 + p['mass2'] * p['L2']**2/12 + p['massTool'] * outer*outer
    h = p['mass2'] * l * c2 * math.sin(q[1]) + p['massTool'] * l * outer * math.sin(q[1] + delta)
    result[0] += (a + 2*b + p['rotorInertia'][0])*ddq[0] + (d+b)*ddq[1] - h*(2*dq[0]*dq[1] + dq[1]**2)
    result[1] += (d+b)*ddq[0] + (d + p['rotorInertia'][1])*ddq[1] + h*dq[0]**2
    result[2] += p['zMass'] * ddq[2]
    for j in range(3):
        result[j] += p['viscous'][j]*dq[j] + p['coulomb'][j]*math.tanh(dq[j]/0.005)
    if q[2] <= 0:
        normal = max(0.0, -p['brushK']*q[2] - p['brushC']*dq[2])
        s, angle = q[0], q[0] + q[1]
        ox, oy = p['toolOffset']
        j1 = [-l*math.cos(s) - p['L2']*math.cos(angle) - ox*math.sin(angle) - oy*math.cos(angle),
              -l*math.sin(s) - p['L2']*math.sin(angle) + ox*math.cos(angle) - oy*math.sin(angle)]
        j2 = [-p['L2']*math.cos(angle) - ox*math.sin(angle) - oy*math.cos(angle),
              -p['L2']*math.sin(angle) + ox*math.cos(angle) - oy*math.sin(angle)]
        vx, vy = (j1[k]*dq[0] + j2[k]*dq[1] for k in (0, 1))
        scale = -p['brushMu'] * normal / math.sqrt(vx*vx + vy*vy + 0.001**2)
        for j, jac in enumerate((j1, j2)):
            result[j] -= scale * (jac[0]*vx + jac[1]*vy)
        result[2] -= normal
    return result
