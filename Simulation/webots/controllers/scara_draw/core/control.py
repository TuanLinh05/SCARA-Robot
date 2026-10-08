"""Replace EffortPID.command to test another torque/force control algorithm."""
import math
from .model import feedforward


class EffortPID:
    def __init__(self, params, enabled=True):
        self.p = params
        self.enabled = enabled
        self.integral = [0.0] * 3

    def command(self, q, dq, qr, dqr, ddqr, dt):
        if dt <= 0 or not all(math.isfinite(v) for row in (q, dq, qr, dqr, ddqr) for v in row):
            raise ValueError('Non-finite controller state or invalid dt.')
        ff = feedforward(qr, dqr, ddqr, self.p, self.enabled)
        output = []
        for j in range(3):
            e, de = qr[j] - q[j], dqr[j] - dq[j]
            raw = self.p['Kp'][j]*e + self.p['Kd'][j]*de + self.p['Ki'][j]*self.integral[j] + ff[j]
            limit = self.p['actuatorLimits'][j]
            value = max(-limit, min(limit, raw))
            # Integrate only while unsaturated, or while error releases saturation.
            if abs(raw) <= limit or e * raw < 0:
                self.integral[j] += e * dt
            output.append(value)
        return output


def ink_allowed(requested, xyz, normal_force, p, settings):
    return (bool(requested) and math.isfinite(normal_force)
            and normal_force >= settings['minimum_contact_force_N']
            and all(math.isfinite(v) for v in xyz)
            and abs(xyz[0]) <= p['paperHalfSize'][0]
            and abs(xyz[1]) <= p['paperHalfSize'][1]
            and -0.0003 <= xyz[2] <= settings['maximum_tip_height_m'])
