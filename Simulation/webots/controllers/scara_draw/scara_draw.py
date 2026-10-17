"""Webots entry point. Keep device I/O here and edit algorithms in core/.

[EN] Main Webots controller (`scara_draw.py`) for the 3-DOF X-SCARA robot:
     - Loads `config/demo.json` and `Simulation/common/config/scara.json`.
     - Initializes Webots motors, encoders, 3D contact sensor, bristle deflection
       sensor, tool GPS, and virtual ink Pen.
     - Runs a 4-phase execution state machine (`warmup` -> `settle` -> `draw` -> `hold`)
       supporting three control modes: `'position'`, `'stepper'`, and `'pid'`.

[VI] Chuong trinh dieu khien chinh tren Webots (`scara_draw.py`) cho robot X-SCARA:
     - Doc file cau hinh `config/demo.json` va `Simulation/common/config/scara.json`.
     - Khoi tao cac dong co, encoder, cam bien luc tiep xuc 3D, cam bien do nhun
       long co, GPS dau but va thiet bi ve muc Pen cua Webots.
     - Thuc thi may trang thai 4 giai doan (`warmup` -> `settle` -> `draw` -> `hold`)
       ho tro 3 che do dieu khien: `'position'`, `'stepper'` va `'pid'`.
"""
import argparse
import json
import math
from pathlib import Path
import sys

from controller import Robot
from core.control import EffortPID, ink_allowed
from core.diagnostics import RunLog
from core.model import fingerprint, fk, ik, motor_coordinates, quantize_stepper_pose
from core.trajectory import build_reference, quintic

PROJECT = Path(__file__).resolve().parents[2]
SIMULATION = PROJECT.parent


def load_settings():
    """Parse `--config` CLI argument and validate all fields in `demo.json`.

    [EN] Ensures control mode, timing parameters, contact thresholds, and hex ink
         color (`#RRGGBB`) are valid before starting the Webots simulation loop.
    [VI] Kiem tra tinh hop le cua che do dieu khien, thong so thoi gian, nguong luc
         tiep xuc va ma mau muc hex (`#RRGGBB`) truoc khi chay vong lap mo phong.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(PROJECT / 'config/demo.json'))
    args = parser.parse_args()

    path = Path(args.config)
    if not path.is_absolute():
        path = PROJECT / path

    settings = json.loads(path.read_text(encoding='utf-8'))

    if settings['control_mode'] not in ('position', 'pid', 'stepper'):
        raise ValueError('control_mode must be position, pid, or stepper.')

    for key in ('time_scale', 'warmup_seconds', 'settle_seconds', 'velocity_filter_hz'):
        if not math.isfinite(settings[key]) or settings[key] <= 0:
            raise ValueError(f'{key} must be positive and finite.')

    if not isinstance(settings['log_every_steps'], int) or settings['log_every_steps'] < 1:
        raise ValueError('log_every_steps must be a positive integer.')

    if not 0 <= settings['ink_density'] <= 1:
        raise ValueError('ink_density must be in [0,1].')

    for key in ('minimum_contact_force_N', 'maximum_tip_height_m'):
        if not math.isfinite(settings[key]) or settings[key] < 0:
            raise ValueError(f'{key} must be finite and nonnegative.')

    color = settings['ink_color']
    if len(color) != 7 or not color.startswith('#'):
        raise ValueError('ink_color needs #RRGGBB.')
    int(color[1:], 16)

    return settings


def main():
    """Run the Webots SCARA drawing controller step loop."""
    # -------------------------------------------------------------------------
    # Step 1: Load configuration, verify world fingerprint, and build trajectory
    # Buoc 1: Doc cau hinh, kiem tra ma bam (fingerprint) cua world va tao quy dao
    # -------------------------------------------------------------------------
    settings = load_settings()
    p = json.loads((SIMULATION / 'common/config/scara.json').read_text(encoding='utf-8'))

    robot = Robot()
    if robot.getCustomData() != fingerprint(p):
        raise ValueError(
            'Physical configuration changed. Run python Simulation/webots/tools/build_project.py, then reload the world.'
        )

    timestep = int(round(robot.getBasicTimeStep()))
    dt = timestep / 1000
    if abs(dt - p['dt']) > 1e-10:
        raise ValueError('World timestep differs from shared configuration; rebuild project.')

    csv_path = Path(settings['csv'])
    if not csv_path.is_absolute():
        csv_path = SIMULATION / csv_path

    ref = build_reference(settings['trajectory'], p, dt, settings['time_scale'], csv_path)

    # -------------------------------------------------------------------------
    # Step 2: Acquire and enable Webots actuators and sensors
    # Buoc 2: Ket noi va kich hoat cac co cau chap hanh va cam bien trong Webots
    # -------------------------------------------------------------------------
    def device(name):
        result = robot.getDevice(name)
        if result is None:
            raise RuntimeError(f'Missing Webots device: {name}')
        return result

    motors = [device(name + '_motor') for name in ('shoulder', 'elbow', 'z')]
    encoders = [device(name + '_sensor') for name in ('shoulder', 'elbow', 'z')]
    contact, deflection, gps, pen = [
        device(name)
        for name in ('brush_contact', 'brush_deflection_sensor', 'tool_gps', 'drawing_pen')
    ]

    for sensor in encoders + [contact, deflection, gps]:
        sensor.enable(timestep)
    for motor in motors[:2]:
        motor.enableTorqueFeedback(timestep)
    motors[2].enableForceFeedback(timestep)

    pen.write(False)
    pen.setInkColor(int(settings['ink_color'][1:], 16), settings['ink_density'])

    # -------------------------------------------------------------------------
    # Step 3: Initialize controller and command initial pose / gravity hold
    # Buoc 3: Khoi tao bo dieu khien va dat tu the ban dau / giu trong luc truc Z
    # -------------------------------------------------------------------------
    pid = EffortPID(p, settings['feedforward'])
    initial = ik([0.034, 0, p['penLift']], p)

    if settings['control_mode'] in ('position', 'stepper'):
        if settings['control_mode'] == 'stepper':
            init_cmd = quantize_stepper_pose(initial, p)
        else:
            init_cmd = initial
        for motor, q in zip(motors, init_cmd):
            motor.setPosition(q)
    else:
        motors[0].setTorque(0)
        motors[1].setTorque(0)
        motors[2].setForce(p['zMass'] * p['gravity'])

    log = RunLog(PROJECT / 'output/runs', p, settings)
    print(f'[SCARA] {settings["trajectory"]} | {settings["control_mode"]} | {dt*1000:g} ms', flush=True)
    print('[SCARA] Edit config/demo.json, then Reset/Reload to run another experiment.', flush=True)

    # -------------------------------------------------------------------------
    # Step 4: Main real-time control loop (stepped every `timestep` ms)
    # Buoc 4: Vong lap dieu khien thoi gian thuc (chay moi chu ky `timestep` ms)
    # -------------------------------------------------------------------------
    q_previous = None
    dq = None
    start_q = None
    start_time = None
    step = 0
    status = 'interrupted'
    last_print = -1
    completed = False

    try:
        while robot.step(timestep) != -1:
            # 4a. Read joint encoders, GPS tip position, and 3D contact normal force
            # 4a. Doc gia tri encoder khop, toa do GPS dau but va luc tiep xuc phap tuyen
            q = [sensor.getValue() for sensor in encoders]
            xyz = list(gps.getValues())
            force = abs(contact.getValues()[2])  # sensor Z stays parallel to world Z

            if not all(math.isfinite(v) for v in q + xyz + [force, deflection.getValue()]):
                raise RuntimeError('Non-finite sensor data; simulation stopped.')

            if start_time is None:
                start_time = robot.getTime()
                start_q = q[:]
                q_previous = q[:]
                dq = [0.0] * 3

            # 4b. Estimate joint velocity `dq` using finite difference + 1st-order low-pass filter
            # 4b. Uoc luong van toc khop `dq` bang sai phan huu han + bo loc thong thap bac 1
            measured = [(q[j] - q_previous[j]) / dt for j in range(3)]
            alpha = 1 - math.exp(-2 * math.pi * settings['velocity_filter_hz'] * dt)
            dq = [dq[j] + alpha * (measured[j] - dq[j]) for j in range(3)]
            q_previous = q[:]

            # 4c. Safety check: detect gross joint drift outside working limits
            # 4c. Kiem tra an toan: phat hien khop troi vuot qua gioi han lam viec
            limits = p['jointLimits'] + [p['zLimits']]
            padding = [0.12, 0.12, 0.002]
            if any(value < lo - pad or value > hi + pad for value, (lo, hi), pad in zip(q, limits, padding)):
                raise RuntimeError('Joint moved outside working limits; inspect gains and physics.')

            # 4d. Evaluate 4-phase trajectory state machine: warmup -> settle -> draw -> hold
            # 4d. Tinh quy dao tham chieu theo 4 giai doan: warmup -> settle -> draw -> hold
            elapsed = robot.getTime() - start_time
            warmup = settings['warmup_seconds']
            settle = settings['settle_seconds']
            trajectory_time = max(0.0, elapsed - warmup - settle)

            if elapsed < warmup:
                u = elapsed / warmup
                delta = [ref.q[0][j] - start_q[j] for j in range(3)]
                qr = [start_q[j] + delta[j] * quintic(u) for j in range(3)]
                dqr = [d * 30 * u * u * (1 - u)**2 / warmup for d in delta]
                ddqr = [d * 60 * u * (1 - 3 * u + 2 * u * u) / warmup**2 for d in delta]
                target = fk(qr, p)
                requested = False
                phase = 'warmup'
            elif elapsed < warmup + settle:
                qr = ref.q[0]
                dqr = [0.0] * 3
                ddqr = [0.0] * 3
                target = ref.xyz[0]
                requested = False
                phase = 'settle'
            else:
                qr, dqr, ddqr, target, requested = ref.sample(trajectory_time)
                phase = 'draw' if trajectory_time <= ref.t[-1] else 'hold'
                if phase == 'hold':
                    requested = False
                    dqr = [0.0] * 3
                    ddqr = [0.0] * 3

            # 4e. Dispatch actuator commands according to selected `control_mode`
            # 4e. Xuat lenh dieu khien dong co theo che do `control_mode` da chon
            if settings['control_mode'] in ('position', 'stepper'):
                if settings['control_mode'] == 'stepper':
                    cmd_q = quantize_stepper_pose(qr, p)
                else:
                    cmd_q = qr
                for motor, value in zip(motors, cmd_q):
                    motor.setPosition(value)
                commands = ['', '', '']  # targets are qr/cmd_q, not commanded efforts
            else:
                commands = pid.command(q, dq, qr, dqr, ddqr, dt)
                # setTorque/setForce disables Webots' internal position controller.
                motors[0].setTorque(commands[0])
                motors[1].setTorque(commands[1])
                motors[2].setForce(commands[2])

            # 4f. Gate virtual ink pen and record telemetry sample
            # 4f. Kiem soat but ve muc va ghi mau du lieu telemetry vao file CSV
            ink = ink_allowed(requested, xyz, force, p, settings)
            pen.write(ink)

            if not completed and step % settings['log_every_steps'] == 0:
                axes, steps = motor_coordinates(q, p)
                feedback = (
                    [motor.getTorqueFeedback() for motor in motors[:2]]
                    + [motors[2].getForceFeedback()]
                )
                xy_err_mm = 1000 * math.hypot(xyz[0] - target[0], xyz[1] - target[1])
                values = [
                    elapsed,
                    min(trajectory_time, ref.t[-1]),
                    phase,
                    *qr,
                    *q,
                    *target,
                    *xyz,
                    deflection.getValue(),
                    force,
                    int(requested),
                    int(ink),
                    xy_err_mm,
                    *commands,
                    *feedback,
                    *axes[:2],
                    *steps,
                ]
                log.row(dict(zip(log.writer.fieldnames, values)))

            second = int(elapsed)
            if second > last_print and not completed:
                last_print = second
                xy_err_mm = 1000 * math.hypot(xyz[0] - target[0], xyz[1] - target[1])
                print(
                    f'[SCARA] {phase} t={trajectory_time:.1f}s | XY error={xy_err_mm:.3f} mm | Fn={force:.3f} N',
                    flush=True,
                )

            if not completed and trajectory_time > ref.t[-1] + 0.5:
                completed = True
                status = 'completed'
                log.close(status)
                if settings['quit_when_done']:
                    break

            step += 1
    except Exception:
        status = 'error'
        raise
    finally:
        pen.write(False)
        log.close(status)
        if status == 'error':
            # [EN] Freeze under position control instead of leaving the last raw effort active.
            # [VI] Khoa vi tri hien tai bang dieu khien vi tri thay vi giu nguyen momen/luc cuoi.
            for motor, sensor in zip(motors, encoders):
                value = sensor.getValue()
                if math.isfinite(value):
                    motor.setPosition(value)


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print(f'[SCARA ERROR] {error}', file=sys.stderr, flush=True)
        raise
