"""Replace EffortPID.command to test another torque/force control algorithm.

[EN] Implements closed-loop joint torque/force control (`EffortPID`) with
     computed-torque feedforward and clamping anti-windup, plus the safety gate
     (`ink_allowed`) that decides when the virtual pen leaves ink on the paper.

[VI] Cai dat bo dieu khien momen/luc vong kin (`EffortPID`) ket hop bu dong luc hoc
     nguoc (feedforward) va chong bao hoa tich phan (anti-windup), cung ham kiem tra
     dieu kien an toan (`ink_allowed`) truoc khi cho phep but ve muc len giay.
"""
import math
from .model import feedforward


class EffortPID:
    """3-axis PID + analytical feedforward effort controller with conditional anti-windup.

    [EN] Control law for each axis `j` in `{0: shoulder, 1: elbow, 2: z}`:
         `u_raw = Kp * e + Kd * de + Ki * integral(e) + u_feedforward`
         `u_cmd = clamp(u_raw, -limit, +limit)`
    [VI] Luat dieu khien cho tung truc `j` trong `{0: vai, 1: khuyu, 2: truc Z}`:
         `u_raw = Kp * e + Kd * de + Ki * integral(e) + u_feedforward`
         `u_cmd = clamp(u_raw, -limit, +limit)`
    """

    def __init__(self, params, enabled=True):
        self.p = params
        self.enabled = enabled
        self.integral = [0.0] * 3

    def command(self, q, dq, qr, dqr, ddqr, dt):
        """Compute saturated actuator efforts `[tau_shoulder, tau_elbow, force_z]`.

        [EN] Validates inputs, evaluates analytical feedforward at the reference
             trajectory `(qr, dqr, ddqr)`, applies PID feedback, clamps to actuator
             limits, and updates the integral state only when unsaturated (anti-windup).
        [VI] Kiem tra dau vao, tinh momen/luc feedforward tai quy dao tham chieu
             `(qr, dqr, ddqr)`, cong phan hoi PID, gioi han theo nguong co cau chap hanh
             va chi tich phan sai so khi chua bao hoa (chong windup).
        """
        # Step 1: Validate sample time dt and finite state vectors
        # Buoc 1: Kiem tra chu ky lay mau dt va tinh huu han cua cac vector trang thai
        all_finite = all(
            math.isfinite(v)
            for row in (q, dq, qr, dqr, ddqr)
            for v in row
        )
        if dt <= 0 or not all_finite:
            raise ValueError('Non-finite controller state or invalid dt.')

        # Step 2: Compute analytical inverse-dynamics feedforward
        # Buoc 2: Tinh thanh phan bu dong luc hoc nguoc (feedforward)
        ff = feedforward(qr, dqr, ddqr, self.p, self.enabled)

        # Step 3: Compute PID effort and anti-windup clamping for each axis
        # Buoc 3: Tinh tin hieu PID, bao hoa dau ra va tich phan co dieu kien cho tung truc
        output = []
        for j in range(3):
            e = qr[j] - q[j]
            de = dqr[j] - dq[j]

            p_term = self.p['Kp'][j] * e
            d_term = self.p['Kd'][j] * de
            i_term = self.p['Ki'][j] * self.integral[j]
            raw = p_term + d_term + i_term + ff[j]

            limit = self.p['actuatorLimits'][j]
            value = max(-limit, min(limit, raw))

            # [EN] Integrate only while unsaturated, or while error pulls out of saturation.
            # [VI] Chi cong don khau tich phan khi chua bao hoa hoac khi sai so giup thoat bao hoa.
            if abs(raw) <= limit or e * raw < 0:
                self.integral[j] += e * dt

            output.append(value)

        return output


def ink_allowed(requested, xyz, normal_force, p, settings):
    """Return True only when the brush is commanded down and physically touching the paper.

    [EN] Checks 5 conditions before enabling the Webots Pen device:
         1. Trajectory requests pen-down (`requested` is True).
         2. Measured 3D contact sensor normal force >= `minimum_contact_force_N`.
         3. GPS tip coordinates `xyz` are finite.
         4. Tip `(x, y)` lies inside the paper rectangle (`±paperHalfSize`).
         5. Tip height `z` is near the paper surface (`-0.3 mm <= z <= maximum_tip_height_m`).

    [VI] Kiem tra 5 dieu kien truoc khi bat thiet bi Pen trong Webots de ve muc:
         1. Quy dao yeu cau ha but (`requested` = True).
         2. Luc phap tuyen do tu cam bien tiep xuc >= `minimum_contact_force_N`.
         3. Toa do mui but `xyz` tu GPS la huu han.
         4. Toa do `(x, y)` nam trong pham vi kho giay (`±paperHalfSize`).
         5. Do cao mui but `z` nam sat mat giay (`-0.3 mm <= z <= maximum_tip_height_m`).
    """
    return (
        bool(requested)
        and math.isfinite(normal_force)
        and normal_force >= settings['minimum_contact_force_N']
        and all(math.isfinite(v) for v in xyz)
        and abs(xyz[0]) <= p['paperHalfSize'][0]
        and abs(xyz[1]) <= p['paperHalfSize'][1]
        and -0.0003 <= xyz[2] <= settings['maximum_tip_height_m']
    )
