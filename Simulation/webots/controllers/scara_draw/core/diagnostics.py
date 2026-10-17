"""CSV + summary.json per run; no overwrite of earlier experiments.

[EN] Manages timestamped experiment directories under `Simulation/webots/output/runs/`,
     recording step-by-step telemetry into `tracking.csv`, configuration snapshots into
     `parameters.json`, and run-level accuracy/force metrics into `summary.json`.

[VI] Quan ly thu muc luu ket qua thuc nghiem theo moc thoi gian trong
     `Simulation/webots/output/runs/`, ghi du lieu cam bien tung buoc vao `tracking.csv`,
     thong so cau hinh vao `parameters.json` va thong ke sai so/luc vao `summary.json`.
"""
import csv
from datetime import datetime
import json
import math
from pathlib import Path

# [EN] Ordered column headers for `tracking.csv`.
# [VI] Danh sach ten cot theo dung thu tu cua file `tracking.csv`.
COLUMNS = [
    'time_s', 'trajectory_time_s', 'phase',
    'S_ref_rad', 'E_ref_rad', 'Z_ref_m',
    'S_rad', 'E_rad', 'Z_m',
    'X_ref_m', 'Y_ref_m', 'free_tip_Z_ref_m',
    'X_tip_m', 'Y_tip_m', 'Z_tip_m',
    'brush_deflection_m', 'normal_force_N',
    'pen_requested', 'ink_enabled', 'xy_error_mm',
    'shoulder_command_Nm', 'elbow_command_Nm', 'z_command_N',
    'shoulder_feedback_Nm', 'elbow_feedback_Nm', 'z_feedback_N',
    'A_rad', 'B_rad', 'A_steps', 'B_steps', 'Z_steps',
]


class RunLog:
    """Experiment logger that streams `tracking.csv` and writes `summary.json` on exit.

    [EN] Creates a unique timestamped subfolder so previous simulation runs are
         never overwritten, and accumulates online RMS / max XY tracking errors.
    [VI] Tao thu muc con mang dau thoi gian duy nhat de khong ghi de cac lan chay truoc,
         dong thoi tich luy sai so RMS / Max XY truc tuyen trong qua trinh ve.
    """

    def __init__(self, output, params, settings):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.folder = Path(output) / timestamp
        self.folder.mkdir(parents=True)

        params_payload = json.dumps({'model': params, 'demo': settings}, indent=2) + '\n'
        (self.folder / 'parameters.json').write_text(params_payload, encoding='utf-8')

        self.file = (self.folder / 'tracking.csv').open('w', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.file, fieldnames=COLUMNS)
        self.writer.writeheader()

        self.count = 0
        self.draw_count = 0
        self.error_square = 0.0
        self.maximum_error = 0.0
        self.maximum_force = 0.0
        self.closed = False

    def row(self, values):
        """Append one telemetry sample row and update running error/force statistics."""
        self.writer.writerow(values)
        self.count += 1

        if values['phase'] == 'draw' and values['pen_requested']:
            self.draw_count += 1
            self.error_square += values['xy_error_mm']**2
            self.maximum_error = max(self.maximum_error, values['xy_error_mm'])

        self.maximum_force = max(self.maximum_force, values['normal_force_N'])

        if self.count % 100 == 0:
            self.file.flush()

    def close(self, status):
        """Flush and close `tracking.csv`, then write final metrics to `summary.json`."""
        if self.closed:
            return
        self.closed = True
        self.file.close()

        if self.draw_count:
            rms_error_mm = math.sqrt(self.error_square / self.draw_count)
            max_error_mm = self.maximum_error
        else:
            rms_error_mm = None
            max_error_mm = None

        result = {
            'status': status,
            'samples': self.count,
            'drawing_samples': self.draw_count,
            'xy_rms_error_mm': rms_error_mm,
            'xy_max_error_mm': max_error_mm,
            'maximum_normal_force_N': self.maximum_force,
            'scope': 'Webots sensor data; primitive collisions and estimated physics',
        }
        summary_payload = json.dumps(result, indent=2) + '\n'
        (self.folder / 'summary.json').write_text(summary_payload, encoding='utf-8')

        print(f'[SCARA] {status}; log: {self.folder}', flush=True)
        print(
            f'[SCARA] XY RMS: {result["xy_rms_error_mm"]} mm; peak force: {self.maximum_force:.4f} N',
            flush=True,
        )
