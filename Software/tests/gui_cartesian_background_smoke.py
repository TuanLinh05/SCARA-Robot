"""Focus/minimize behavior with a fake MCU; no real COM is opened."""

from pathlib import Path
from dataclasses import replace
import sys, time, tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from cartesian_nc_control import NCApp
from cartesian_nc_demo import DemoLink
from cartesian_nc_model import NCConfig
from cartesian_nc_workspace import Setpoint


class RecordingDemo(DemoLink):
    def __init__(self, port):
        self.sent = []
        self.freeze = False
        super().__init__(port)

    def send(self, line):
        self.sent.append(line)
        super().send(line)

    def advance(self):
        if self.freeze and self.busy and not self.initializing:
            self.ends = time.monotonic() + 1
        super().advance()


base = Path(__file__).resolve().parents[2] / "Firmware/ScaraCartesian/build_host"
temp = tempfile.TemporaryDirectory(dir=base)
assert Path(temp.name).resolve().is_relative_to(base.resolve())
path = Path(temp.name) / "config.json"
NCConfig().save(path)
app = NCApp(factory=RecordingDemo, config_path=path, demo=True)
app._foreground = lambda: True
errors = []
app.report_callback_exception = lambda *e: errors.append(e)


def pump(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.update()
        time.sleep(0.004)
    assert not errors, errors


def until(predicate, seconds=4):
    end = time.monotonic() + seconds
    while not predicate() and time.monotonic() < end:
        pump(0.02)
    assert predicate(), app.message.get()


def home():
    app.initialize()
    until(lambda: app.local_reference, 3)


def focus():
    app._foreground = lambda: False
    app._focus_check()
    app._unmap_check()


def moves():
    return [line for line in app.link.sent if line.startswith("MOVE ")]


try:
    app.toggle_connection()
    pump(0.15)
    home()
    model = app.model
    epoch = app.state.epoch
    pos = app.state.pos
    app.bk_vars["paper_z"].set("10")
    app.confirm_paper()
    assert app.bk_confirmed()
    count = len(app.link.sent)
    focus()
    pump(0.2)
    assert (
        app.local_reference
        and app.model is model
        and app.state.epoch == epoch
        and app.state.pos == pos
    )
    assert app.bk_confirmed() and not any(
        line.startswith("STOP") for line in app.link.sent[count:]
    )
    # A finite MOVE can be validated and sent in the background.
    target = model.fk((8, 50, 22))
    app.start_plan(True, target=target, speed=10)
    until(lambda: app.state.busy, 2)
    focus()
    until(lambda: not app.state.busy and not app.planning, 2)
    assert app.local_reference and app.bk_confirmed() and app.state.epoch == epoch
    # A validated setpoint program also progresses through dwell in background.
    app.points = [
        Setpoint("A", *model.fk((10, 52, 22)), 10, 0.15),
        Setpoint("B", *model.fk((12, 55, 22)), 10, 0.15),
    ]
    app.start_program()
    until(lambda: not app.program_points and not app.planning and not app.state.busy, 3)
    assert app.local_reference and app.bk_confirmed()
    # Manual follow loses the old mouse target, drains the one bounded segment,
    # and preserves the reference instead of sending STOP FOCUS.
    app._foreground = lambda: True
    app.link.freeze = True
    app.follow_enabled.set(True)
    app.toggle_follow()
    app.set_target_pose((16, 59, 22), True)
    app.release_pose()
    until(lambda: app.state.busy, 2)
    count = len(moves())
    count_tx = len(app.link.sent)
    focus()
    pump(0.1)
    assert not app.follow_enabled.get() and app.follow_target is None and app.state.busy
    assert not any(line.startswith("STOP") for line in app.link.sent[count_tx:])
    app.link.freeze = False
    until(lambda: not app.state.busy, 2)
    pump(0.2)
    assert len(moves()) == count and app.local_reference and app.bk_confirmed()
    # Strict foreground mode is still available explicitly.
    app.config = replace(app.config, motion_in_background=False)
    app._foreground = lambda: True
    app.start_plan(True, target=model.fk((20, 60, 22)), speed=10)
    until(lambda: app.state.busy, 2)
    focus()
    pump(0.1)
    assert (
        app.state.reason in ("stop_focus", "stop_minimize") and not app.local_reference
    )
    print(
        "R10 focus: idle/reference/paper retained; background MOVE and program continue; follow drains once; optional strict mode still stops passed"
    )
finally:
    app.close()
    temp.cleanup()
