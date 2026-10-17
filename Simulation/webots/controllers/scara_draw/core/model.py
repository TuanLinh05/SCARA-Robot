"""X-SCARA geometry and analytic feedforward, matching the MATLAB model.

[EN] Core mathematical model for the 3-DOF X-SCARA robot in Webots:
     1. Forward Kinematics (`fk`) & Analytical Inverse Kinematics (`ik`).
     2. Differential belt coupling (`B = q2 + q1 / 3`) and stepper pulse quantization.
     3. Closed-form 2-DOF planar Lagrangian inverse dynamics + vertical Z dynamics
        + brush-paper contact Jacobian transpose feedforward (`feedforward`).

[VI] Mo hinh toan hoc cot loi cho robot X-SCARA 3 bac tu do trong Webots:
     1. Dong hoc thuan (`fk`) & Dong hoc nguoc giai tich (`ik`).
     2. Truyen dong dai vi sai (`B = q2 + q1 / 3`) va luong tu hoa xung dong co buoc.
     3. Dong luc hoc nguoc Lagrange 2 bac tu do phang + dong luc hoc truc Z
        + bu luc tiep xuc giua long co va mat giay qua ma tran chuyen vi Jacobian (`feedforward`).
"""
import hashlib
import json
import math

# [EN] Physical parameter keys hashed to verify that the Webots .proto file matches scara.json.
# [VI] Danh sach cac khoa thong so vat ly dung de bam (hash) kiem tra tinh dong bo giua file .proto va scara.json.
PHYSICAL_KEYS = (
    'L1', 'L2', 'base', 'toolOffset', 'paperHalfSize', 'jointLimits',
    'zLimits', 'mass1', 'mass2', 'massTool', 'zMass', 'rotorInertia',
    'viscous', 'coulomb', 'gravity', 'brushK', 'brushC', 'brushMu',
    'penLift', 'lineWidth0', 'actuatorLimits', 'dt',
)


def fingerprint(p):
    """Compute SHA-256 fingerprint of physical parameters to detect stale Webots worlds.

    [EN] Hashes the canonical JSON representation of `PHYSICAL_KEYS`.
    [VI] Bam SHA-256 chuoi JSON chuan hoa cua cac thong so vat ly `PHYSICAL_KEYS`.
    """
    data = {key: p[key] for key in PHYSICAL_KEYS}
    serialized = json.dumps(data, sort_keys=True).encode()
    return hashlib.sha256(serialized).hexdigest()


def fk(q, p):
    """Forward Kinematics: joint state `q = [q1, q2, z]` -> Cartesian tip `[x, y, z]`.

    [EN] Coordinate convention (matching MATLAB `scara_fk.m`):
         - Home pose (`q1 = 0, q2 = 0`) points straight along the `+Y` axis.
         - Positive angle rotates counter-clockwise from `+Y` toward `-X`, so a link
           at angle `theta` has direction vector `[-sin(theta), cos(theta)]`.
         - Tool offset `(ox, oy)` is defined in the distal link's local frame
           (`+X_local` = `[cos(a), sin(a)]`, `+Y_local` = `[-sin(a), cos(a)]`).

    [VI] Quy uoc toa do (dong nhat voi `scara_fk.m` trong MATLAB):
         - Tu the HOME (`q1 = 0, q2 = 0`) duoi thang doc theo truc `+Y`.
         - Chieu quay duong la nguoc chieu kim dong ho (tu `+Y` sang `-X`), nen khau
           o goc `theta` co vector chi phuong `[-sin(theta), cos(theta)]`.
         - Do lech dau but `(ox, oy)` duoc dinh nghia trong he toa do cuc bo cua khau 2
           (`+X_local` = `[cos(a), sin(a)]`, `+Y_local` = `[-sin(a), cos(a)]`).
    """
    s = q[0]          # [EN] Shoulder angle q1 (rad) / [VI] Goc khop vai q1 (rad)
    e = q[1]          # [EN] Relative elbow angle q2 (rad) / [VI] Goc khop khuyu tuong doi q2 (rad)
    z = q[2]          # [EN] Free brush-tip height z (m) / [VI] Do cao mui but tu do z (m)
    a = s + e         # [EN] Absolute orientation of link 2 (rad) / [VI] Goc tuyet doi cua khau 2 (rad)
    ox, oy = p['toolOffset']

    x = (
        p['base'][0]
        - p['L1'] * math.sin(s)
        - p['L2'] * math.sin(a)
        + ox * math.cos(a)
        - oy * math.sin(a)
    )
    y = (
        p['base'][1]
        + p['L1'] * math.cos(s)
        + p['L2'] * math.cos(a)
        + ox * math.sin(a)
        + oy * math.cos(a)
    )
    return [x, y, z]


def ik(xyz, p, branch=1, check_limits=True):
    """Analytical Inverse Kinematics: Cartesian target `[x, y, z]` -> `q = [q1, q2, z]`.

    [EN] Step-by-step geometry (matching MATLAB `scara_ik.m`):
         1. Combine link 2 (`L2` along `+Y_local`) and tool offset `(ox, oy)` into an
            effective virtual second link of length `outer = hypot(ox, L2 + oy)` and
            fixed angular offset `delta = atan2(-ox, L2 + oy)`.
         2. Apply the Law of Cosines on the triangle (Base -> Elbow -> Tip):
            `r^2 = L1^2 + outer^2 + 2 * L1 * outer * cos(effective)`.
         3. Solve for the effective elbow angle `effective = branch * acos(c)` and
            reject targets near kinematic singularities (`|sin(effective)| < margin`).
         4. Compute shoulder angle `s` and subtract `delta` to get physical `q2`.

    [VI] Cac buoc giai tich hinh hoc (dong nhat voi `scara_ik.m` trong MATLAB):
         1. Gop khau 2 (`L2`) va do lech dau but `(ox, oy)` thanh mot khau thu hai ao
            co chieu dai hieu dung `outer = hypot(ox, L2 + oy)` va goc lech co dinh
            `delta = atan2(-ox, L2 + oy)`.
         2. Ap dung Dinh ly Cosin cho tam giac (Goc Base -> Khuyu -> Mui but):
            `r^2 = L1^2 + outer^2 + 2 * L1 * outer * cos(effective)`.
         3. Giai goc khuyu hieu dung `effective = branch * acos(c)` va loai bo cac diem
            nam qua gan vung suy bien dong hoc (`|sin(effective)| < margin`).
         4. Tinh goc khop vai `s` va tru goc lech `delta` de thu duoc goc khuyu `q2`.
    """
    # Step 1: Validate inputs
    # Buoc 1: Kiem tra tinh hop le cua dau vao
    if branch not in (-1, 1) or not all(math.isfinite(v) for v in xyz):
        raise ValueError('IK needs finite XYZ and branch +1 or -1.')

    # Step 2: Compute target displacement from base and effective distal link (outer, delta)
    # Buoc 2: Tinh do doi diem dich so voi goc Base va thong so khau 2 hieu dung (outer, delta)
    dx = xyz[0] - p['base'][0]
    dy = xyz[1] - p['base'][1]
    ox, oy = p['toolOffset']

    outer = math.hypot(ox, p['L2'] + oy)
    delta = math.atan2(-ox, p['L2'] + oy)

    # Step 3: Law of Cosines for reachability check
    # Buoc 3: Ap dung Dinh ly Cosin va kiem tra tam voi
    r2 = dx * dx + dy * dy
    if r2 < 1e-18 or outer <= 0 or p['L1'] <= 0:
        raise ValueError('Unreachable or undefined folded origin.')

    c = (r2 - p['L1']**2 - outer**2) / (2 * p['L1'] * outer)
    if abs(c) > 1 + 1e-10:
        raise ValueError('Point outside SCARA workspace.')

    # Step 4: Solve effective elbow angle and check singularity margin
    # Buoc 4: Tinh goc khuyu hieu dung va kiem tra dieu kien suy bien
    clamped_c = max(-1.0, min(1.0, c))
    effective = branch * math.acos(clamped_c)
    if abs(math.sin(effective)) < p['singularityMargin']:
        raise ValueError('Point too close to a kinematic singularity.')

    # Step 5: Solve shoulder angle `s` and physical elbow angle `q2 = effective - delta`
    # Buoc 5: Tinh goc khop vai `s` va goc khop khuyu vat ly `q2 = effective - delta`
    target_azimuth = math.atan2(-dx, dy)
    shoulder_offset = math.atan2(
        outer * math.sin(effective),
        p['L1'] + outer * math.cos(effective),
    )
    s = target_azimuth - shoulder_offset
    q = [s, effective - delta, xyz[2]]

    # Step 6: Verify joint and Z workspace limits
    # Buoc 6: Kiem tra gioi han khop quay va hanh trinh truc Z
    if check_limits:
        limits = p['jointLimits'] + [p['zLimits']]
        if any(v < lo - 1e-10 or v > hi + 1e-10 for v, (lo, hi) in zip(q, limits)):
            raise ValueError('Reference exceeds assumed joint/Z limits.')

    return q


def round_nearest(x):
    """Symmetric half-away-from-zero rounding matching Firmware C `round_int` and MATLAB `round`.

    [EN] Python's built-in `round()` uses round-half-to-even (banker's rounding),
         whereas C firmware and MATLAB round `+2.5 -> +3` and `-2.5 -> -3`.
    [VI] Ham `round()` mac dinh cua Python lam tron ve so chan gan nhat, trong khi
         Firmware C va MATLAB lam tron doi xung ra xa so 0 (`+2.5 -> +3`, `-2.5 -> -3`).
    """
    if x >= 0:
        return int(math.floor(x + 0.5))
    return -int(math.floor(-x + 0.5))


def motor_coordinates(q, p):
    """Convert joint coordinates `q = [q1, q2, z]` into motor shafts `[A, B, Z]` and integer steps.

    [EN] Models the mechanical timing-belt cross-coupling (`B = q2 + q1 / 3`):
         when shoulder motor A rotates by `q1`, the elbow belt is pulled by `q1 / 3`,
         so motor B must rotate by `q2 + q1 / elbowCrosstalkRatio` to achieve relative
         elbow angle `q2`.
    [VI] Mo hinh hoa co cau truyen dong dai vi sai (`B = q2 + q1 / 3`):
         khi dong co vai A quay mot goc `q1`, day dai khuyu bi keo theo mot luong `q1 / 3`,
         do do dong co B phai quay `q2 + q1 / elbowCrosstalkRatio` de dat goc tuong doi `q2`.
    """
    a = q[0]
    b = q[1] + q[0] / p['elbowCrosstalkRatio']
    axes = [a, b, q[2]]

    steps_a = round_nearest(math.degrees(a) * p['stepsPerDegree'][0])
    steps_b = round_nearest(math.degrees(b) * p['stepsPerDegree'][1])
    steps_z = round_nearest(q[2] * p['stepsPerMeterZ'])
    steps = [steps_a, steps_b, steps_z]

    return axes, steps


def from_steps(steps, p):
    """Reconstruct joint state `[q1_rad, q2_rad, z_m]` from discrete motor steps `[step_A, step_B, step_Z]`.

    [EN] Converts discrete stepper counts back to radians/meters and subtracts the
         belt cross-coupling (`q2 = b - a / elbowCrosstalkRatio`).
    [VI] Chuyen doi so nguyen xung buoc nguoc ve radian/met va tru di luong ghep noi
         truyen dong dai (`q2 = b - a / elbowCrosstalkRatio`).
    """
    a = math.radians(steps[0] / p['stepsPerDegree'][0])
    b = math.radians(steps[1] / p['stepsPerDegree'][1])
    z = steps[2] / p['stepsPerMeterZ']
    q2 = b - a / p['elbowCrosstalkRatio']
    return [a, q2, z]


def quantize_stepper_pose(q, p):
    """Quantize continuous joint reference `q` through discrete stepper pulses and belt coupling.

    [EN] Used in `'stepper'` control mode to emulate finite stepper resolution.
    [VI] Dung trong che do dieu khien `'stepper'` de mo phong do phan giai huu han cua dong co buoc.
    """
    _, steps = motor_coordinates(q, p)
    return from_steps(steps, p)


def feedforward(q, dq, ddq, p, enabled=True):
    """Compute analytical inverse-dynamics feedforward `[tau_1 (N*m), tau_2 (N*m), F_z (N)]`.

    [EN] Solves the Euler-Lagrange equations of motion for the 3-DOF SCARA robot:
         `tau = M(q) * ddq + C(q, dq) * dq + F_gravity + F_friction + J_contact^T * F_contact`
         1. Gravity compensation: Only the vertical Z carriage (`zMass * g`) acts
            against gravity; the planar SCARA links move horizontally.
         2. Planar 2-DOF Inertia Matrix `M(q)`:
            - Link 1 (mass `m1`, length `L1`, COM at `c1 = L1/2`, rod inertia `m1*L1^2/12`).
            - Link 2 (mass `m2`, length `L2`, COM at `c2 = L2/2`, rod inertia `m2*L2^2/12`).
            - Brush tool (point mass `m_tool` at distance `outer` and angle offset `delta`).
            - `M_11 = a + 2*b + I_rotor1`, `M_12 = M_21 = d + b`, `M_22 = d + I_rotor2`.
         3. Coriolis & Centrifugal terms `C(q, dq) * dq`:
            - Derived from `h = -0.5 * d(M_11)/dq2`:
              shoulder term `-h * (2*dq1*dq2 + dq2^2)`, elbow centripetal term `+h * dq1^2`.
         4. Joint Friction: Viscous damping `viscous * dq` + smooth Coulomb `coulomb * tanh(dq / 0.005)`.
         5. Brush-Paper Contact (`q[2] <= 0`):
            - Normal bristle force `Fn = max(0, -K_brush * z - C_brush * dz)`.
            - Planar friction opposes tip velocity `[vx, vy] = J * [dq1, dq2]^T` and maps
              into joint torques via the Jacobian transpose `J^T`.

    [VI] Tinh momen/luc bu dong luc hoc nguoc giai tich `[tau_1 (N*m), tau_2 (N*m), F_z (N)]`
         theo phuong trinh Euler-Lagrange:
         `tau = M(q) * ddq + C(q, dq) * dq + F_trong_truong + F_ma_sat + J_tiep_xuc^T * F_tiep_xuc`
         1. Bu trong luc: Chi co cum truc Z (`zMass * g`) chiu tac dung cua trong luc;
            hai khau tay may SCARA chuyen dong trong mat phang ngang.
         2. Ma tran quan tinh phang 2 bac tu do `M(q)`:
            - Tinh theo dinh ly Steiner (truc song song) cho thanh dong chat L1, L2 va
              chat diem cum dau but `massTool` tai ban kinh `outer`, goc lech `delta`.
         3. Thanh phan Coriolis va luc ly tam `C(q, dq) * dq`:
            - Vai: `-h * (2*dq1*dq2 + dq2^2)`; Khuyu: `+h * dq1^2`.
         4. Ma sat khop: Ma sat nhot `viscous * dq` + ma sat kho Coulomb lam muot bang `tanh`.
         5. Tiep xuc dau co - mat giay (`q[2] <= 0`):
            - Luc phap tuyen dan hoi cua long co: `Fn = max(0, -K_brush * z - C_brush * dz)`.
            - Luc ma sat truot tren giay nguoc chieu van toc mui but `[vx, vy]` va duoc
              chieu ve momen khop thong qua ma tran chuyen vi Jacobian `J^T`.
    """
    # Step 1: Always compensate vertical gravity on the Z axis, even if feedforward is disabled
    # Buoc 1: Luon bu trong luc tren truc dung Z ngay ca khi tat feedforward dong luc hoc
    result = [0.0, 0.0, p['zMass'] * p['gravity']]
    if not enabled:
        return result

    # Step 2: Geometric lengths and effective tool polar coordinates (outer, delta)
    # Buoc 2: Cac thong so chieu dai hinh hoc va toa do cuc hieu dung cua dau but (outer, delta)
    l = p['L1']
    c1 = p['L1'] / 2
    c2 = p['L2'] / 2
    outer = math.hypot(p['toolOffset'][0], p['L2'] + p['toolOffset'][1])
    delta = math.atan2(-p['toolOffset'][0], p['L2'] + p['toolOffset'][1])

    # Step 3: Compute Lagrangian inertia matrix terms (a, b, d) and Coriolis factor (h)
    # Buoc 3: Tinh cac thanh phan ma tran quan tinh Lagrange (a, b, d) va he so Coriolis (h)
    b = (
        p['mass2'] * l * c2 * math.cos(q[1])
        + p['massTool'] * l * outer * math.cos(q[1] + delta)
    )
    a = (
        p['mass1'] * c1 * c1
        + p['mass1'] * l * l / 12
        + p['mass2'] * (l * l + c2 * c2)
        + p['mass2'] * p['L2']**2 / 12
        + p['massTool'] * (l * l + outer * outer)
    )
    d = (
        p['mass2'] * c2 * c2
        + p['mass2'] * p['L2']**2 / 12
        + p['massTool'] * outer * outer
    )
    h = (
        p['mass2'] * l * c2 * math.sin(q[1])
        + p['massTool'] * l * outer * math.sin(q[1] + delta)
    )

    # Step 4: Assemble inertial torques M(q)*ddq and Coriolis/centrifugal torques C(q,dq)*dq
    # Buoc 4: Tong hop momen quan tinh M(q)*ddq va momen Coriolis/ly tam C(q,dq)*dq
    m11 = a + 2 * b + p['rotorInertia'][0]
    m12 = d + b
    m22 = d + p['rotorInertia'][1]

    coriolis_shoulder = -h * (2 * dq[0] * dq[1] + dq[1]**2)
    coriolis_elbow = h * dq[0]**2

    result[0] += m11 * ddq[0] + m12 * ddq[1] + coriolis_shoulder
    result[1] += m12 * ddq[0] + m22 * ddq[1] + coriolis_elbow
    result[2] += p['zMass'] * ddq[2]

    # Step 5: Add viscous damping and smoothed Coulomb friction for all 3 axes
    # Buoc 5: Cong them ma sat nhot va ma sat kho Coulomb (lam muot bang tanh) cho ca 3 truc
    for j in range(3):
        viscous_term = p['viscous'][j] * dq[j]
        coulomb_term = p['coulomb'][j] * math.tanh(dq[j] / 0.005)
        result[j] += viscous_term + coulomb_term

    # Step 6: Add brush-paper contact reaction forces when the tip is compressed (q[2] <= 0)
    # Buoc 6: Tinh luc phan khang khi dau long co ep xuong mat giay (q[2] <= 0)
    if q[2] <= 0:
        normal = max(0.0, -p['brushK'] * q[2] - p['brushC'] * dq[2])
        s = q[0]
        angle = q[0] + q[1]
        ox, oy = p['toolOffset']

        # [EN] Planar Jacobian columns j1 = d(xy)/dq1 and j2 = d(xy)/dq2.
        # [VI] Hai cot cua ma tran Jacobian phang: j1 = d(xy)/dq1 va j2 = d(xy)/dq2.
        j1 = [
            -l * math.cos(s) - p['L2'] * math.cos(angle) - ox * math.sin(angle) - oy * math.cos(angle),
            -l * math.sin(s) - p['L2'] * math.sin(angle) + ox * math.cos(angle) - oy * math.sin(angle),
        ]
        j2 = [
            -p['L2'] * math.cos(angle) - ox * math.sin(angle) - oy * math.cos(angle),
            -p['L2'] * math.sin(angle) + ox * math.cos(angle) - oy * math.sin(angle),
        ]

        # [EN] Cartesian brush-tip sliding velocity [vx, vy] on the paper surface.
        # [VI] Van toc truot Descartes [vx, vy] cua mui co tren be mat giay.
        vx = j1[0] * dq[0] + j2[0] * dq[1]
        vy = j1[1] * dq[0] + j2[1] * dq[1]

        # [EN] Smoothly regularized kinetic friction projected via Jacobian transpose J^T.
        # [VI] Ma sat truot duoc lam muot va chieu ve khop qua chuyen vi Jacobian J^T.
        scale = -p['brushMu'] * normal / math.sqrt(vx * vx + vy * vy + 0.001**2)
        for j, jac in enumerate((j1, j2)):
            result[j] -= scale * (jac[0] * vx + jac[1] * vy)
        result[2] -= normal

    return result
