"""BK GUI/end-to-end COM fixture only: no real port or motor is accessed."""

from pathlib import Path
import sys, time, tempfile, ctypes

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from cartesian_nc_control import NCApp
from cartesian_nc_demo import DemoLink
from cartesian_nc_model import NCConfig


class FastDemo(DemoLink):
    def __init__(self, port):
        self.sent = []
        self.freeze = False
        self.silent = False
        super().__init__(port)

    def send(self, line):
        if line.startswith("MOVE "):
            assert not self.busy, "Overlapping MOVE"
        self.sent.append(line)
        super().send(line)
        if line.startswith("MOVE "):
            self.ends = time.monotonic() + 0.002

    def _read(self):
        while not self.closed_event.wait(0.008):
            self.advance()

    def advance(self):
        if self.freeze and self.path_active:
            self.emit()
            return
        if self.freeze and self.busy and not self.initializing:
            self.ends = time.monotonic() + 1
        super().advance()

    def emit(self):
        if not self.silent:
            super().emit()


base = Path(__file__).resolve().parents[2] / "Firmware/ScaraCartesian/build_host"
temp = tempfile.TemporaryDirectory(dir=base)
assert Path(temp.name).resolve().is_relative_to(base.resolve())
path = Path(temp.name) / "config.json"
NCConfig().save(path)
app = NCApp(factory=FastDemo, config_path=path, demo=True)
app._foreground = lambda: True
errors = []
app.report_callback_exception = lambda *e: errors.append(e)


def pump(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.update()
        time.sleep(0.002)
    assert not errors, errors


def until(condition, seconds=15):
    end = time.monotonic() + seconds
    while not condition() and time.monotonic() < end:
        pump(0.02)
    assert condition(), (app.message.get(), app.bk_note.get())


def moves():
    return [s for s in app.link.sent if s.startswith("MOVE ")]


def home():
    app.initialize()
    until(lambda: app.local_reference, 3)
    pump(0.05)


try:
    app.toggle_connection()
    pump(0.1)
    home()
    app.open_bk()
    pump(0.1)
    assert str(app.bk_window.transient()) == str(app)
    assert (
        app.bk_stop_button.winfo_rooty() + app.bk_stop_button.winfo_height()
        <= app.bk_stop_button.master.winfo_rooty()
        + app.bk_stop_button.master.winfo_height()
    )
    count = len(moves())
    app.start_bk()
    pump(0.1)
    assert len(moves()) == count and not app.bk_confirmed()
    assert "3 mm/s" in app.home_hint.cget("text")
    # Fitting before paper confirmation must not move or invent a paper Z.
    sent = len(app.link.sent)
    epoch = app.state.epoch
    app.fit_bk()
    until(lambda: not app.planning, 4)
    assert float(app.bk_vars["size_mm"].get()) > 40 and not app.bk_confirmed()
    assert not any(
        line.startswith(("MOVE ", "PATH ", "SEG ", "GO "))
        for line in app.link.sent[sent:]
    )
    assert app.local_reference and app.state.epoch == epoch
    app.bk_vars["size_mm"].set("20")
    app.bk_vars["paper_z"].set("10")
    app.confirm_paper()
    assert app.bk_confirmed()
    # An invalid later stroke must never send the initial lift either.
    app.bk_vars["center_y"].set("0")
    app.start_bk()
    until(lambda: not app.planning, 2)
    assert len(moves()) == count and not app.program_points
    app.bk_vars["center_y"].set("165")
    app.preview_bk()
    app.start_bk(True)
    until(lambda: app.bk_active or not app.planning, 2)
    assert app.bk_active and app.bk_active_plan.dry
    until(lambda: not app.bk_active and not app.program_points, 15)
    assert len(moves()) > count and all(not trail for trail in app.bk_trails)
    assert all(
        int(line.split()[3]) == 19200 for line in moves()[count:]
    )  # all target Z=12mm
    # Ink: the same three strokes, no missed reference, final pen is raised.
    count = len(moves())
    tx_start = len(app.link.sent)
    app.start_bk(False)
    until(lambda: app.bk_active or not app.planning, 2)
    assert app.bk_active
    until(lambda: not app.bk_active and not app.program_points, 15)
    assert app.local_reference and all(len(trail) > 2 for trail in app.bk_trails)
    assert abs(app.model.from_steps(app.state.pos)[2] - 12) < 0.001
    assert "Đã vẽ BK" in app.message.get()
    commands = app.link.sent[tx_start:]
    assert sum(line.startswith("PATH ") for line in commands) == 3
    assert sum(line.startswith("GO ") for line in commands) == 3
    assert sum(line.startswith("SEG ") for line in commands) == sum(
        len(r.segments) for r in app.bk_active_plan.runs
    )
    assert len(moves()) - count <= 10  # transfers only, never 134 individual ink MOVEs
    assert all(
        int(line.split()[4]) == 16000 for line in commands if line.startswith("SEG ")
    )
    assert "Tổng thời gian" in app.bk_note.get()
    # Both requested names can finish while Codex/another app has focus.
    model = app.model
    epoch = app.state.epoch
    for word in ("LINH", "THOA"):
        app.bk_vars["pattern"].set(word)
        app.select_drawing_pattern()
        app._foreground = lambda: False
        app._focus_check()
        app._unmap_check()
        app.start_bk(False)
        until(lambda: app.bk_active or not app.planning, 2)
        assert app.bk_active and app.bk_confirmed()
        until(lambda: not app.bk_active and not app.program_points, 18)
        assert app.model is model and app.state.epoch == epoch and app.local_reference
        assert len(app.bk_trails) == 8 and all(
            len(trail) > 2 for trail in app.bk_trails
        )
        assert f"Đã vẽ {word}" in app.message.get() and app.bk_confirmed()
        try:
            from PIL import ImageGrab

            app.bk_window.focus_force()
            pump(0.05)
            hwnd = ctypes.windll.user32.GetAncestor(app.bk_window.winfo_id(), 2)
            ImageGrab.grab(window=hwnd).save(
                base / f"cartesian_nc_R12_{word}_preview.png"
            )
        except (ImportError, OSError) as e:
            print("Optional own-window screenshot unavailable:", e)
    app._foreground = lambda: True
    # Full enlarged THOA has >500 vertices overall, streamed in small strokes.
    app.fit_bk()
    until(lambda: not app.planning, 4)
    assert float(app.bk_vars["size_mm"].get()) > 100 and app.bk_confirmed()
    app.start_bk()
    until(lambda: app.bk_active or not app.planning, 4)
    assert app.bk_active and len(app.bk_active_plan.operations) > 500
    until(lambda: not app.bk_active and not app.program_points, 25)
    assert app.local_reference and all(len(trail) > 2 for trail in app.bk_trails)
    assert abs(app.model.from_steps(app.state.pos)[2] - 12) < 0.001
    app.bk_vars["size_mm"].set("40")
    # Stop during an actual buffered ink stroke, including a pending refill.
    app.start_bk()
    until(lambda: app.path_upload is not None and app.path_upload.started, 5)
    app.link.freeze = True
    sent = len(app.link.sent)
    app.stop()
    pump(0.15)
    assert (
        not app.path_upload
        and not app.bk_active
        and not app.program_points
        and not app.local_reference
    )
    assert not any(
        line.startswith(("SEG ", "GO ", "MOVE ")) for line in app.link.sent[sent:]
    )
    app.link.freeze = False
    home()
    app.confirm_paper()
    try:
        from PIL import ImageGrab

        app.bk_window.focus_force()
        pump(0.05)
        hwnd = ctypes.windll.user32.GetAncestor(app.bk_window.winfo_id(), 2)
        ImageGrab.grab(window=hwnd).save(base / "cartesian_nc_R12_BK_preview.png")
    except (ImportError, OSError) as e:
        print("Optional own-window screenshot unavailable:", e)
    # Cancelling preflight must not execute the resulting immutable program.
    count = len(moves())
    app.start_bk()
    app.stop()
    pump(0.2)
    assert len(moves()) == count and not app.program_points and not app.bk_active
    # A running drawing stops once, drops its remaining strokes, and loses home.
    app.link.freeze = True
    app.start_bk()
    until(lambda: app.state.busy, 2)
    count = len(moves())
    app.stop()
    pump(0.15)
    assert len(moves()) == count and not app.local_reference and not app.bk_active
    app.link.freeze = False
    home()
    assert not app.bk_confirmed()
    app.start_bk()
    pump(0.1)
    assert len(moves()) == count  # paper confirmation cannot survive HOME
    app.confirm_paper()
    assert app.bk_confirmed()
    app.link.freeze = True
    app.start_bk()
    until(lambda: app.state.busy, 2)
    count = len(moves())
    app.link.silent = True
    until(lambda: app.link is None, 2)
    assert not app.bk_active and not app.program_points and not app.local_reference
    print(
        "R12 GUI: read-only fit, enlarged >500-vertex THOA, cubic/quintic strokes, background writing, paper-Z, final lift, buffered STOP and stale abort passed"
    )
finally:
    app.close()
    temp.cleanup()
