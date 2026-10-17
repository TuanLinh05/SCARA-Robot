"""Export exact GUI/R9 DDA profiles for MATLAB, without opening a COM port.

[EN] Offline trajectory generator that imports the production Python GUI planner
     (`Software/src/cartesian_nc_*.py`), compiles a drawing pattern into R9 integer
     Digital Differential Analyzer (DDA) stepper pulse segments, simulates every
     individual step tick, and resamples the resulting timeline at fixed step `dt`
     (default 2 ms) into CSV and JSON files for MATLAB/Simulink.

[VI] Cong cu xuat quy dao ngoai tuyen (offline) su dung truc tiep bo lap quy dao
     cua phan mem GUI Python (`Software/src/cartesian_nc_*.py`), bien dich hinh ve
     thanh cac doan xung buoc DDA so nguyen (R9), mo phong tung nhip xung (tick)
     va lay mau lai theo chu ky co dinh `dt` (mac dinh 2 ms) ra file CSV va JSON
     de nap vao MATLAB/Simulink.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import sys

sys.dont_write_bytecode = True

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'Software/src'))

from cartesian_nc_model import NCConfig, MeasuredModel
from cartesian_nc_drawing import BKSettings, compile_bk
from cartesian_nc_path import smooth_square, ramp_ticks


def export(pattern, folder, dt=.002):
    """Compile `pattern` using the R9 integer DDA planner and write CSV + JSON to `folder`.

    [EN] Step-by-step workflow:
         1. Load `robot_calibration.json` and construct `MeasuredModel`.
         2. Compile the requested `pattern` from the parked step coordinates.
         3. Step tick-by-tick through continuous lookahead `runs` (pen-down) and
            point-to-point `plan` moves (pen-up / Z transitions) using Bresenham/DDA
            integer accumulators.
         4. Resample the discrete step event log onto a uniform `dt` grid.

    [VI] Quy trinh thuc hien tung buoc:
         1. Doc `robot_calibration.json` va khoi tao mo hinh `MeasuredModel`.
         2. Bien dich hinh ve `pattern` bat dau tu toa do xung tai vi tri do (park).
         3. Mo phong tung nhip xung (tick) qua cac chuoi noi suy lien tuc `runs` (khi ve)
            va cac chuyen dong diem-toi-diem `plan` (khi nhac/ha but) bang bo tich luy
            so nguyen Bresenham/DDA.
         4. Lay mau lai chuoi su kien xung tren luoi thoi gian deu `dt`.
    """
    # -------------------------------------------------------------------------
    # Step 1: Load calibration snapshot and build the kinematic/stepper model
    # Buoc 1: Doc file cau hinh hieu chuan va khoi tao mo hinh dong hoc xung
    # -------------------------------------------------------------------------
    calibration_path = Path(__file__).resolve().parents[1] / 'config/robot_calibration.json'
    snapshot = json.loads(calibration_path.read_text(encoding='utf-8'))

    cfg = NCConfig.from_dict(snapshot['config'])
    h = snapshot['calibration']
    factors = tuple(h['factors_pp_unit'])

    joint_limits_with_margin = tuple((a + 2, b - 2) for a, b in cfg.joint_limits_deg)
    z_limits_mm = (.1, h['range'][0] / factors[0] - .1)

    model = MeasuredModel(
        l1_mm=cfg.l1_mm,
        l2_mm=cfg.l2_mm,
        tool_offset_mm=tuple(cfg.tool_offset_mm),
        home_deg=(0, 0),
        home_z_mm=0,
        joint_limits_deg=joint_limits_with_margin,
        z_limits_mm=z_limits_mm,
        microsteps=tuple(cfg.microsteps),
        ratios=(cfg.lead_mm_rev, *cfg.ratios),
        positive_high=tuple(cfg.positive_high),
        coupling=h['coupling'],
        measured_factors=factors,
        branch=1,
        max_master_rate=cfg.max_master_rate,
        max_joint_speed_deg_s=tuple(cfg.max_joint_speed_deg_s),
        max_z_speed_mm_s=cfg.max_z_speed_mm_s,
    )

    # -------------------------------------------------------------------------
    # Step 2: Configure pattern drawing settings and initial parked step counts
    # Buoc 2: Cau hinh thong so hinh ve va toa do xung ban dau tai vi tri Park
    # -------------------------------------------------------------------------
    recorded = snapshot.get('drawing_settings', {}).get(pattern)
    if recorded:
        settings = BKSettings(**recorded)
    else:
        if pattern not in ('BK', 'LINH', 'THOA'):
            default_size_mm = 24
        elif pattern == 'BK':
            default_size_mm = 20
        else:
            default_size_mm = 40
        settings = BKSettings(
            pattern=pattern,
            size_mm=default_size_mm,
            center_x=0,
            center_y=165,
            paper_z=20,
            lift_mm=2,
        )

    start = (
        round(h['range'][0] - cfg.z_clear_mm * factors[0]),
        0,
        round(cfg.park_deg[1] * factors[2]),
    )

    program = compile_bk(model, start, settings)
    position = list(start)
    events = [(0, tuple(position), model.fk(model.from_steps(position)), 0)]

    # -------------------------------------------------------------------------
    # Step 3: Simulate firmware integer DDA pulse generation tick by tick
    # Buoc 3: Mo phong thuat toan phat xung DDA so nguyen cua Firmware theo tung tick
    # -------------------------------------------------------------------------
    now = 0
    index = 0
    runs = {r.first: r for r in program.runs}

    while index < len(program.operations):
        op = program.operations[index]

        if index in runs:
            # [EN] Multi-segment lookahead run (continuous pen-down drawing).
            # [VI] Chuoi doan noi suy nhin truoc (lookahead) khi ha but ve lien tuc.
            run = runs[index]
            now += .05
            for j, seg in enumerate(run.segments):
                previous = tuple(position)
                delta = tuple(b - a for a, b in zip(previous, seg.steps))
                accum = [0, 0, 0]
                p0 = model.fk(model.from_steps(previous))
                p1 = program.operations[index + j].point.xyz

                for tick in range(seg.ticks):
                    for a, d in enumerate(delta):
                        accum[a] += abs(d)
                        if accum[a] >= seg.ticks:
                            accum[a] -= seg.ticks
                            position[a] += (d > 0) - (d < 0)

                    target = tuple(a + (b - a) * (tick + 1) / seg.ticks for a, b in zip(p0, p1))
                    events.append((now, tuple(position), target, 1))

                    ramp_up_sq = smooth_square(seg.entry, seg.peak, tick, seg.ramp_up)
                    ramp_down_sq = smooth_square(seg.exit, seg.peak, seg.ticks - tick, seg.ramp_down)
                    rate = max(50, math.isqrt(min(ramp_up_sq, ramp_down_sq)))
                    now += (1000000 // rate) / 1e6

            index = run.end
        else:
            # [EN] Single point-to-point move (travel or Z pen lift/lower).
            # [VI] Chuyen dong diem-toi-diem don le (di chuyen khong ve hoac nhac/ha truc Z).
            plan = model.plan(tuple(position), op.point.xyz, op.point.speed_mm_s)
            delta = tuple(b - a for a, b in zip(position, plan.steps))
            n = max(map(abs, delta))
            accum = [0, 0, 0]
            old = model.fk(model.from_steps(position))
            now += .05

            for tick in range(n):
                for a, d in enumerate(delta):
                    accum[a] += abs(d)
                    if accum[a] >= n:
                        accum[a] -= n
                        position[a] += (d > 0) - (d < 0)

                target = tuple(a + (b - a) * (tick + 1) / n for a, b in zip(old, op.point.xyz))
                events.append((now, tuple(position), target, 0))

                is_z_only = not delta[1] and not delta[2]
                initial = min(plan.rate, cfg.z_latch_pps if is_z_only else 50)
                rate = initial + (plan.rate - initial) * min(tick, n - tick, 200) // 200
                now += (1000000 // max(50, rate)) / 1e6

            now += op.point.dwell_s
            index += 1

    events.append((now, tuple(position), model.fk(model.from_steps(position)), 0))

    # -------------------------------------------------------------------------
    # Step 4: Resample discrete events at uniform `dt` and write CSV + JSON
    # Buoc 4: Lay mau lai tai chu ky co dinh `dt` va ghi ra file CSV + JSON
    # -------------------------------------------------------------------------
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / (pattern.lower() + '.csv')
    cursor = 0

    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            't_s', 'Z_steps', 'A_steps', 'B_steps',
            'x_ref_m', 'y_ref_m', 'z_ref_m', 'pen', 'valid'
        ])
        total_samples = math.ceil(now / dt) + 1
        for i in range(total_samples):
            t = i * dt
            while cursor + 1 < len(events) and events[cursor + 1][0] <= t:
                cursor += 1
            _, pos, xyz, pen = events[cursor]
            writer.writerow([round(t, 6), *pos, *(v / 1000 for v in xyz), pen, 1])

    meta = dict(
        source='R9 integer DDA and GUI compiler; measured real-log calibration; simulated timing excludes USB upload and segment transition delay',
        source_log=snapshot['source_log'],
        dt=dt,
        pattern=pattern,
        settings=settings.__dict__,
        config=cfg.__dict__,
        calibration=h,
        start_steps=start,
        end_steps=position,
        duration_s=now,
        events=len(events),
        model_error_mm=program.max_model_error_mm,
    )
    json_path = folder / (pattern.lower() + '.json')
    json_path.write_text(json.dumps(meta, indent=2) + '\n', encoding='utf-8')
    return meta


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--pattern', default='FLOWER')
    parser.add_argument(
        '--output',
        type=Path,
        default=Path(__file__).resolve().parents[1] / 'output/reference',
    )
    args = parser.parse_args()
    m = export(args.pattern.upper(), args.output)
    print(json.dumps({k: m[k] for k in ('pattern', 'duration_s', 'events', 'model_error_mm')}))
