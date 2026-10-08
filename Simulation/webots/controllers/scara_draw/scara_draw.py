"""Webots entry point. Keep device I/O here and edit algorithms in core/."""
import argparse
import json
import math
from pathlib import Path
import sys

from controller import Robot
from core.control import EffortPID, ink_allowed
from core.diagnostics import RunLog
from core.model import fingerprint, fk, ik, motor_coordinates
from core.trajectory import build_reference, quintic

PROJECT = Path(__file__).resolve().parents[2]
SIMULATION = PROJECT.parent


def load_settings():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(PROJECT/'config/demo.json'))
    args = parser.parse_args()
    path = Path(args.config)
    if not path.is_absolute():
        path = PROJECT / path
    settings = json.loads(path.read_text(encoding='utf-8'))
    if settings['control_mode'] not in ('position', 'pid'):
        raise ValueError('control_mode must be position or pid.')
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
    settings = load_settings()
    p = json.loads((SIMULATION/'common/config/scara.json').read_text(encoding='utf-8'))
    robot = Robot()
    if robot.getCustomData() != fingerprint(p):
        raise ValueError('Physical configuration changed. Run python Simulation/webots/tools/build_project.py, then reload the world.')
    timestep = int(round(robot.getBasicTimeStep()))
    dt = timestep / 1000
    if abs(dt-p['dt']) > 1e-10:
        raise ValueError('World timestep differs from shared configuration; rebuild project.')
    csv_path = Path(settings['csv'])
    if not csv_path.is_absolute():
        csv_path = SIMULATION / csv_path
    ref = build_reference(settings['trajectory'], p, dt, settings['time_scale'], csv_path)
    def device(name):
        result = robot.getDevice(name)
        if result is None:
            raise RuntimeError(f'Missing Webots device: {name}')
        return result
    motors = [device(name+'_motor') for name in ('shoulder', 'elbow', 'z')]
    encoders = [device(name+'_sensor') for name in ('shoulder', 'elbow', 'z')]
    contact, deflection, gps, pen = [device(name) for name in
                                    ('brush_contact', 'brush_deflection_sensor', 'tool_gps', 'drawing_pen')]
    for sensor in encoders + [contact, deflection, gps]:
        sensor.enable(timestep)
    for motor in motors[:2]:
        motor.enableTorqueFeedback(timestep)
    motors[2].enableForceFeedback(timestep)
    pen.write(False)
    pen.setInkColor(int(settings['ink_color'][1:], 16), settings['ink_density'])
    pid = EffortPID(p, settings['feedforward'])
    initial = ik([0.034, 0, p['penLift']], p)
    if settings['control_mode'] == 'position':
        for motor, q in zip(motors, initial):
            motor.setPosition(q)
    else:
        motors[0].setTorque(0)
        motors[1].setTorque(0)
        motors[2].setForce(p['zMass']*p['gravity'])
    log = RunLog(PROJECT/'output/runs', p, settings)
    print(f'[SCARA] {settings["trajectory"]} | {settings["control_mode"]} | {dt*1000:g} ms', flush=True)
    print('[SCARA] Edit config/demo.json, then Reset/Reload to run another experiment.', flush=True)
    q_previous = dq = start_q = None
    start_time = None
    step = 0
    status, last_print, completed = 'interrupted', -1, False
    try:
        while robot.step(timestep) != -1:
            q = [sensor.getValue() for sensor in encoders]
            xyz = list(gps.getValues())
            force = abs(contact.getValues()[2])  # sensor Z stays parallel to world Z
            if not all(math.isfinite(v) for v in q+xyz+[force, deflection.getValue()]):
                raise RuntimeError('Non-finite sensor data; simulation stopped.')
            if start_time is None:
                start_time, start_q = robot.getTime(), q[:]
                q_previous, dq = q[:], [0.0]*3
            measured = [(q[j]-q_previous[j])/dt for j in range(3)]
            alpha = 1-math.exp(-2*math.pi*settings['velocity_filter_hz']*dt)
            dq = [dq[j] + alpha*(measured[j]-dq[j]) for j in range(3)]
            q_previous = q[:]
            # Detect gross drift before applying further force commands.
            limits = p['jointLimits'] + [p['zLimits']]
            padding = [0.12, 0.12, 0.002]
            if any(value < lo-pad or value > hi+pad for value, (lo, hi), pad in zip(q, limits, padding)):
                raise RuntimeError('Joint moved outside working limits; inspect gains and physics.')
            elapsed = robot.getTime()-start_time
            warmup, settle = settings['warmup_seconds'], settings['settle_seconds']
            trajectory_time = max(0., elapsed-warmup-settle)
            if elapsed < warmup:
                u = elapsed/warmup
                delta = [ref.q[0][j]-start_q[j] for j in range(3)]
                qr = [start_q[j]+delta[j]*quintic(u) for j in range(3)]
                dqr = [d*30*u*u*(1-u)**2/warmup for d in delta]
                ddqr = [d*60*u*(1-3*u+2*u*u)/warmup**2 for d in delta]
                target, requested, phase = fk(qr,p), False, 'warmup'
            elif elapsed < warmup+settle:
                qr, dqr, ddqr = ref.q[0], [0.]*3, [0.]*3
                target, requested, phase = ref.xyz[0], False, 'settle'
            else:
                qr, dqr, ddqr, target, requested = ref.sample(trajectory_time)
                phase = 'draw' if trajectory_time <= ref.t[-1] else 'hold'
                if phase == 'hold':
                    requested, dqr, ddqr = False, [0.]*3, [0.]*3
            if settings['control_mode'] == 'position':
                for motor, value in zip(motors, qr):
                    motor.setPosition(value)
                commands = ['', '', '']  # targets are qr, not commanded efforts
            else:
                commands = pid.command(q, dq, qr, dqr, ddqr, dt)
                # setTorque/setForce disables Webots' internal position controller.
                motors[0].setTorque(commands[0])
                motors[1].setTorque(commands[1])
                motors[2].setForce(commands[2])
            ink = ink_allowed(requested, xyz, force, p, settings)
            pen.write(ink)
            if not completed and step % settings['log_every_steps'] == 0:
                axes, steps = motor_coordinates(q, p)
                feedback = [motor.getTorqueFeedback() for motor in motors[:2]] + [motors[2].getForceFeedback()]
                values = [elapsed, min(trajectory_time,ref.t[-1]), phase, *qr, *q, *target, *xyz,
                          deflection.getValue(), force, int(requested), int(ink),
                          1000*math.hypot(xyz[0]-target[0],xyz[1]-target[1]), *commands, *feedback,
                          *axes[:2], *steps]
                log.row(dict(zip(log.writer.fieldnames, values)))
            second = int(elapsed)
            if second > last_print and not completed:
                last_print = second
                print(f'[SCARA] {phase} t={trajectory_time:.1f}s | XY error={1000*math.hypot(xyz[0]-target[0],xyz[1]-target[1]):.3f} mm | Fn={force:.3f} N', flush=True)
            if not completed and trajectory_time > ref.t[-1] + 0.5:
                completed, status = True, 'completed'
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
            # Freeze under position control instead of leaving the last raw effort.
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
