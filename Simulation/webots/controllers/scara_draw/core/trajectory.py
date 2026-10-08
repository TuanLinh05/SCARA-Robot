"""Quintic Cartesian paths, then IK; CSV row i controls segment i-1 -> i."""
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from .model import ik


def quintic(u):
    return u**3 * (10 - 15*u + 6*u*u)


@dataclass
class Reference:
    t: list
    xyz: list
    q: list
    dq: list
    ddq: list
    pen: list

    def sample(self, time):
        time = max(self.t[0], min(self.t[-1], time))
        i = max(0, min(len(self.t) - 2, bisect.bisect_right(self.t, time) - 1))
        u = (time - self.t[i]) / (self.t[i+1] - self.t[i])
        def mix(data):
            return [a + u*(b-a) for a, b in zip(data[i], data[i+1])]
        return mix(self.q), mix(self.dq), mix(self.ddq), mix(self.xyz), self.pen[i] and self.pen[i+1]


def gradient(values, dt):
    return [[(values[min(i+1, len(values)-1)][j] - values[max(i-1, 0)][j]) /
             ((1 if i in (0, len(values)-1) else 2) * dt) for j in range(3)]
            for i in range(len(values))]


def build_reference(kind, p, dt=None, time_scale=1.0, csv_path=None):
    dt = p['dt'] if dt is None else dt
    if dt <= 0 or not math.isfinite(dt) or not math.isfinite(time_scale) or time_scale <= 0:
        raise ValueError('dt and time_scale must be positive and finite.')
    t, xyz = [0.0], []
    def segment(target, seconds):
        n = max(2, math.ceil(seconds * time_scale / dt))
        start, start_t = xyz[-1], t[-1]
        for i in range(1, n+1):
            w = quintic(i/n)
            xyz.append([a + (b-a)*w for a, b in zip(start, target)])
            t.append(start_t + i*dt)
    if kind in ('flower', 'circle'):
        xyz.append([0.034, 0.0, p['penLift']])
        segment([0.034, 0.0, -p['brushCompression']], 0.6)
        n, start_t = max(2, math.ceil(8*time_scale/dt)), t[-1]
        for i in range(1, n+1):
            angle = 2*math.pi*quintic(i/n)
            r = 0.023 + 0.011*math.cos(5*angle) if kind == 'flower' else 0.034
            xyz.append([r*math.cos(angle), r*math.sin(angle), -p['brushCompression']])
            t.append(start_t + i*dt)
        segment([xyz[-1][0], xyz[-1][1], p['penLift']], 0.6)
    elif kind == 'square':
        corners = [[0.032, -0.032], [0.032, 0.032], [-0.032, 0.032], [-0.032, -0.032], [0.032, -0.032]]
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
        if len(rows) < 2 or any(not all(math.isfinite(v) for v in row) or row[2] not in (0, 1) for row in rows):
            raise ValueError('CSV needs at least two finite rows; pen must be 0 or 1.')
        pts = [[row[0]*0.001, row[1]*0.001] for row in rows]
        xyz.append(pts[0] + [p['penLift']])
        for i in range(1, len(pts)):
            z = -p['brushCompression'] if rows[i][2] else p['penLift']
            segment(pts[i-1] + [z], 0.4)
            segment(pts[i] + [z], max(0.4, math.dist(pts[i], pts[i-1])/0.020))
        segment(pts[-1] + [p['penLift']], 0.4)
    else:
        raise ValueError('trajectory must be flower, circle, square or csv.')
    # Fail before commanding motors if any requested waypoint is invalid.
    q = [ik(point, p) for point in xyz]
    dq = gradient(q, dt)
    return Reference(t, xyz, q, dq, gradient(dq, dt), [point[2] < 0 for point in xyz])
