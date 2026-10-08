"""Device/CSV integration with a mock, explicitly NOT a physics test."""
import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT/'controllers/scara_draw'))
from core.model import fk, fingerprint, ik
from core.diagnostics import COLUMNS


class FakeMotor:
    def __init__(self):
        self.position = None
        self.effort = 0
        self.position_calls = self.effort_calls = 0
    def setPosition(self,value):
        self.position = value
        self.position_calls += 1
    def setTorque(self,value):
        self.effort = value
        self.effort_calls += 1
    setForce = setTorque
    def enableTorqueFeedback(self,period): pass
    enableForceFeedback = enableTorqueFeedback
    def getTorqueFeedback(self): return self.effort
    getForceFeedback = getTorqueFeedback


class FakeSensor:
    def __init__(self, read): self.read = read
    def enable(self,period): pass
    def getValue(self): return self.read()
    def getValues(self): return self.read()


class FakePen:
    def __init__(self): self.writes = []
    def write(self,enabled): self.writes.append(enabled)
    def setInkColor(self,color,density): pass


class FakeRobot:
    def __init__(self,p):
        self.p, self.time = p, 0
        self.q = ik([.034,0,p['penLift']],p)
        self.devices = {}
        self.motors = [FakeMotor() for _ in range(3)]
        for j, name in enumerate(('shoulder','elbow','z')):
            self.devices[name+'_motor'] = self.motors[j]
            self.devices[name+'_sensor'] = FakeSensor(lambda j=j: self.q[j])
        self.devices['tool_gps'] = FakeSensor(lambda: fk(self.q[:2]+[max(0,self.q[2])],p))
        self.devices['brush_deflection_sensor'] = FakeSensor(lambda: max(0,-self.q[2]))
        self.devices['brush_contact'] = FakeSensor(lambda: [0,0,max(0,-p['brushK']*self.q[2])])
        self.devices['drawing_pen'] = FakePen()
    def getCustomData(self): return fingerprint(self.p)
    def getBasicTimeStep(self): return self.p['dt']*1000
    def getDevice(self,name): return self.devices.get(name)
    def getTime(self): return self.time
    def step(self,timestep):
        self.time += timestep/1000
        # Ideal servo only. Raw effort mode holds sensors fixed to inspect I/O.
        for j, motor in enumerate(self.motors):
            if motor.position is not None:
                self.q[j] = motor.position
        return 0 if self.time < 15 else -1


class MemoryLog:
    def __init__(self,*args):
        self.writer = types.SimpleNamespace(fieldnames=COLUMNS)
        self.rows, self.status = [], None
    def row(self,values):
        if list(values) != COLUMNS:
            raise AssertionError('CSV row has missing or misordered fields.')
        self.rows.append(values)
    def close(self,status): self.status = status


class ControllerTests(unittest.TestCase):
    def test_position_and_effort_modes_devices_logs_and_completion(self):
        p = json.loads((PROJECT.parent/'common/config/scara.json').read_text(encoding='utf-8'))
        defaults = json.loads((PROJECT/'config/demo.json').read_text(encoding='utf-8'))
        for mode in ('position','pid'):
            robot, log = FakeRobot(p), MemoryLog()
            module_path = PROJECT/'controllers/scara_draw/scara_draw.py'
            spec = importlib.util.spec_from_file_location('scara_controller_test',module_path)
            module = importlib.util.module_from_spec(spec)
            with patch.dict(sys.modules,{'controller':types.SimpleNamespace(Robot=lambda:robot)}):
                spec.loader.exec_module(module)
            settings = dict(defaults,control_mode=mode,quit_when_done=True)
            with patch.object(module,'load_settings',return_value=settings), patch.object(module,'RunLog',return_value=log), contextlib.redirect_stdout(io.StringIO()):
                module.main()
            self.assertEqual(log.status,'completed')
            self.assertGreater(len(log.rows),1000)
            self.assertFalse(robot.devices['drawing_pen'].writes[-1])
            for row in log.rows:
                self.assertEqual(len(row),len(COLUMNS))
                self.assertTrue(all(math.isfinite(v) for v in row.values() if isinstance(v,(int,float))))
                if row['phase'] in ('warmup','settle','hold'):
                    self.assertEqual(row['ink_enabled'],0)
            for motor in robot.motors:
                if mode == 'pid':
                    self.assertGreater(motor.effort_calls,1000)
                    self.assertEqual(motor.position_calls,0)
                else:
                    self.assertGreater(motor.position_calls,1000)
                    self.assertEqual(motor.effort_calls,0)
            if mode == 'position':
                self.assertTrue(any(row['ink_enabled'] for row in log.rows))
                self.assertTrue(all(row['shoulder_command_Nm'] == '' for row in log.rows))


if __name__ == '__main__':
    unittest.main()
