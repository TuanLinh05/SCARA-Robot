"""CSV + summary.json per run; no overwrite of earlier experiments."""
import csv
from datetime import datetime
import json
import math
from pathlib import Path

COLUMNS = ['time_s', 'trajectory_time_s', 'phase', 'S_ref_rad', 'E_ref_rad', 'Z_ref_m',
           'S_rad', 'E_rad', 'Z_m', 'X_ref_m', 'Y_ref_m', 'free_tip_Z_ref_m',
           'X_tip_m', 'Y_tip_m', 'Z_tip_m', 'brush_deflection_m', 'normal_force_N',
           'pen_requested', 'ink_enabled', 'xy_error_mm', 'shoulder_command_Nm',
           'elbow_command_Nm', 'z_command_N', 'shoulder_feedback_Nm',
           'elbow_feedback_Nm', 'z_feedback_N', 'A_rad', 'B_rad', 'A_steps', 'B_steps', 'Z_steps']


class RunLog:
    def __init__(self, output, params, settings):
        self.folder = Path(output) / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.folder.mkdir(parents=True)
        (self.folder/'parameters.json').write_text(json.dumps({'model': params, 'demo': settings}, indent=2)+'\n', encoding='utf-8')
        self.file = (self.folder/'tracking.csv').open('w', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.file, fieldnames=COLUMNS)
        self.writer.writeheader()
        self.count = self.draw_count = 0
        self.error_square = self.maximum_error = self.maximum_force = 0.0
        self.closed = False

    def row(self, values):
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
        if self.closed:
            return
        self.closed = True
        self.file.close()
        result = {'status': status, 'samples': self.count, 'drawing_samples': self.draw_count,
                  'xy_rms_error_mm': math.sqrt(self.error_square/self.draw_count) if self.draw_count else None,
                  'xy_max_error_mm': self.maximum_error if self.draw_count else None,
                  'maximum_normal_force_N': self.maximum_force,
                  'scope': 'Webots sensor data; primitive collisions and estimated physics'}
        (self.folder/'summary.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
        print(f'[SCARA] {status}; log: {self.folder}', flush=True)
        print(f'[SCARA] XY RMS: {result["xy_rms_error_mm"]} mm; peak force: {self.maximum_force:.4f} N', flush=True)
