"""Quintic Cartesian paths, then IK; CSV row i controls segment i-1 -> i.

[EN] Generates smooth, minimum-jerk Cartesian trajectories (`flower`, `circle`,
     `square`, or custom `csv`), maps every waypoint into joint space via analytical
     Inverse Kinematics (`ik`), and differentiates numerically (`gradient`) to
     obtain reference joint velocities `dq` and accelerations `ddq`.

[VI] Sinh quy dao Descartes muot theo da thuc bac 5 (`flower`, `circle`, `square`,
     hoac doc tu file `csv`), anh xa tung diem sang khong gian khop bang Dong hoc
     nguoc giai tich (`ik`), sau do tinh dao ham so (`gradient`) de thu duoc van toc
     khop tham chieu `dq` va gia toc khop tham chieu `ddq`.
"""
import bisect
import csv
from dataclasses import dataclass
import math
from pathlib import Path
from .model import ik


def quintic(u):
    """Minimum-jerk 5th-order polynomial time scaling s(u) = 10*u^3 - 15*u^4 + 6*u^5.

    [EN] Maps normalized time `u in [0, 1]` to `[0, 1]` with zero velocity and
         zero acceleration at both endpoints (`u = 0` and `u = 1`).
    [VI] Anh xa thoi gian chuan hoa `u in [0, 1]` sang `[0, 1]` dam bao van toc
         va gia toc bang 0 tai ca hai dau mut (`u = 0` va `u = 1`).
    """
    return u**3 * (10 - 15 * u + 6 * u * u)


@dataclass
class Reference:
    """Time-indexed reference trajectory container with linear interpolation sampling.

    [EN] Stores discrete arrays of timestamps `t`, Cartesian poses `xyz`, joint
         positions `q`, joint velocities `dq`, joint accelerations `ddq`, and
         boolean pen-down flags `pen`.
    [VI] Luu tru cac danh sach theo thoi gian `t`, toa do Descartes `xyz`, goc khop `q`,
         van toc khop `dq`, gia toc khop `ddq` va co trang thai ha but `pen`.
    """
    t: list
    xyz: list
    q: list
    dq: list
    ddq: list
    pen: list

    def sample(self, time):
        """Interpolate `(qr, dqr, ddqr, xyz_ref, pen_down)` at arbitrary timestamp `time`.

        [EN] Clamps `time` to `[t[0], t[-1]]`, finds the enclosing interval `[t[i], t[i+1]]`
             via binary search (`bisect_right`), and linearly blends continuous vectors.
             `pen` is True only if both endpoints of the interval have `pen == True`.
        [VI] Gioi han `time` trong doan `[t[0], t[-1]]`, tim khoang thoi gian `[t[i], t[i+1]]`
             bang tim kiem nhi phan (`bisect_right`) va noi suy tuyen tinh cac vector.
             Co `pen` chi bang True khi ca hai dau doan deu co `pen == True`.
        """
        time = max(self.t[0], min(self.t[-1], time))
        i = max(0, min(len(self.t) - 2, bisect.bisect_right(self.t, time) - 1))
        u = (time - self.t[i]) / (self.t[i + 1] - self.t[i])

        def mix(data):
            return [a + u * (b - a) for a, b in zip(data[i], data[i + 1])]

        pen_active = self.pen[i] and self.pen[i + 1]
        return mix(self.q), mix(self.dq), mix(self.ddq), mix(self.xyz), pen_active


def gradient(values, dt):
    """Compute numerical time derivative of an Nx3 sequence using central differences.

    [EN] Uses 1st-order forward/backward differences at the boundaries (`i = 0` and
         `i = N - 1`, denominator `1 * dt`) and 2nd-order central differences in the
         interior (`0 < i < N - 1`, denominator `2 * dt`).
    [VI] Tinh dao ham so theo thoi gian cua chuoi Nx3 bang sai phan trung tam bac 2
         o cac diem ben trong (mau so `2 * dt`) va sai phan tien/lui bac 1 o hai bien
         (`i = 0` va `i = N - 1`, mau so `1 * dt`).
    """
    n = len(values)
    result = []
    for i in range(n):
        prev_idx = max(i - 1, 0)
        next_idx = min(i + 1, n - 1)
        span_steps = 1 if i in (0, n - 1) else 2
        denom = span_steps * dt
        row = [(values[next_idx][j] - values[prev_idx][j]) / denom for j in range(3)]
        result.append(row)
    return result


def build_reference(kind, p, dt=None, time_scale=1.0, csv_path=None):
    """Build a complete `Reference` trajectory for `flower`, `circle`, `square`, or `csv`.

    [EN] Constructs Cartesian waypoints `xyz` (where negative Z commands `-brushCompression`
         press the bristles onto the paper and positive Z commands `+penLift` lift the brush),
         then solves `ik()` for every sample before any motor command is issued.
    [VI] Xay dung chuoi diem quy dao Descartes `xyz` (trong do Z am `-brushCompression`
         ep dau long co xuong giay, Z duong `+penLift` nhac but len khoi mat giay),
         sau do giai `ik()` cho tat ca cac diem truoc khi cho phep dieu khien dong co.
    """
    # Step 1: Validate sample period `dt` and `time_scale`
    # Buoc 1: Kiem tra chu ky lay mau `dt` va he so co gian thoi gian `time_scale`
    dt = p['dt'] if dt is None else dt
    if dt <= 0 or not math.isfinite(dt) or not math.isfinite(time_scale) or time_scale <= 0:
        raise ValueError('dt and time_scale must be positive and finite.')

    t = [0.0]
    xyz = []

    def segment(target, seconds):
        """Append a straight-line Cartesian segment interpolated with `quintic(i / n)`."""
        n = max(2, math.ceil(seconds * time_scale / dt))
        start = xyz[-1]
        start_t = t[-1]
        for i in range(1, n + 1):
            w = quintic(i / n)
            xyz.append([a + (b - a) * w for a, b in zip(start, target)])
            t.append(start_t + i * dt)

    # Step 2: Generate Cartesian path waypoints based on `kind`
    # Buoc 2: Sinh cac diem quy dao Descartes tuy theo loai hinh `kind`
    if kind in ('flower', 'circle'):
        xyz.append([0.034, 0.0, p['penLift']])
        segment([0.034, 0.0, -p['brushCompression']], 0.6)

        n = max(2, math.ceil(8 * time_scale / dt))
        start_t = t[-1]
        for i in range(1, n + 1):
            angle = 2 * math.pi * quintic(i / n)
            if kind == 'flower':
                r = 0.023 + 0.011 * math.cos(5 * angle)
            else:
                r = 0.034
            xyz.append([r * math.cos(angle), r * math.sin(angle), -p['brushCompression']])
            t.append(start_t + i * dt)

        segment([xyz[-1][0], xyz[-1][1], p['penLift']], 0.6)

    elif kind == 'square':
        corners = [
            [0.032, -0.032],
            [0.032,  0.032],
            [-0.032, 0.032],
            [-0.032, -0.032],
            [0.032, -0.032],
        ]
        xyz.append(corners[0] + [p['penLift']])
        segment(corners[0] + [-p['brushCompression']], 0.6)
        for point in corners[1:]:
            segment(point + [-p['brushCompression']], 2)
        segment(corners[-1] + [p['penLift']], 0.6)

    elif kind == 'csv':
        if csv_path is None:
            raise ValueError('CSV trajectory needs a file path.')
        with Path(csv_path).open(encoding='utf-8-sig', newline='') as f:
            reader = csv.DictReader(f)
            if reader.fieldnames != ['x_mm', 'y_mm', 'pen']:
                raise ValueError('CSV header must be x_mm,y_mm,pen.')
            rows = [[float(row[k]) for k in reader.fieldnames] for row in reader]

        invalid_rows = (
            len(rows) < 2
            or any(not all(math.isfinite(v) for v in row) or row[2] not in (0, 1) for row in rows)
        )
        if invalid_rows:
            raise ValueError('CSV needs at least two finite rows; pen must be 0 or 1.')

        pts = [[row[0] * 0.001, row[1] * 0.001] for row in rows]
        xyz.append(pts[0] + [p['penLift']])
        for i in range(1, len(pts)):
            z = -p['brushCompression'] if rows[i][2] else p['penLift']
            segment(pts[i - 1] + [z], 0.4)
            move_duration = max(0.4, math.dist(pts[i], pts[i - 1]) / 0.020)
            segment(pts[i] + [z], move_duration)
        segment(pts[-1] + [p['penLift']], 0.4)

    else:
        raise ValueError('trajectory must be flower, circle, square or csv.')

    # Step 3: Solve IK for all waypoints (fail early if any point is invalid) and differentiate
    # Buoc 3: Giai IK cho toan bo quy dao (bao loi som neu co diem bat kha thi) va tinh dao ham
    q = [ik(point, p) for point in xyz]
    dq = gradient(q, dt)
    ddq = gradient(dq, dt)
    pen_flags = [point[2] < 0 for point in xyz]

    return Reference(t, xyz, q, dq, ddq, pen_flags)
