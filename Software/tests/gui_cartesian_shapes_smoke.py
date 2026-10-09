"""R13 gallery/layout and complete template execution with fake COM only."""

from pathlib import Path
import sys, time, tempfile, ctypes

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from cartesian_nc_control import NCApp
from cartesian_nc_demo import DemoLink
from cartesian_nc_model import NCConfig
from cartesian_nc_shapes import SHAPES, PATTERNS


class ShapeDemo(DemoLink):
    def __init__(self, port):
        self.sent = []
        self.silent = False
        self.freeze_path = False
        super().__init__(port)

    def send(self, line):
        self.sent.append(line)
        super().send(line)
        if line.startswith("MOVE "):
            self.ends = time.monotonic() + 0.002

    def _read(self):
        while not self.closed_event.wait(0.008):
            self.advance()

    def emit(self):
        if not self.silent:
            super().emit()

    def advance(self):
        if self.freeze_path and self.path_active:
            self.emit()
            return
        super().advance()


base = Path(__file__).resolve().parents[2] / "Firmware/ScaraCartesian/build_host"
temp = tempfile.TemporaryDirectory(dir=base)
path = Path(temp.name) / "config.json"
NCConfig().save(path)
app = NCApp(factory=ShapeDemo, config_path=path, demo=True)
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


def screenshot(window, name):
    try:
        from PIL import ImageGrab

        window.focus_force()
        pump(0.1)
        hwnd = ctypes.windll.user32.GetAncestor(window.winfo_id(), 2)
        ImageGrab.grab(window=hwnd).save(base / name)
    except (ImportError, OSError) as e:
        print("Optional own-window screenshot unavailable:", e)


try:
    app.open_bk()
    pump(0.1)
    assert set(app.bk_tiles) == set(PATTERNS)
    app.choose_pattern("FLOWER")
    pump(0.1)
    assert app.bk_title.get() == "Hoa 6 cánh" and app.bk_vars["size_mm"].get() == "24"
    assert str(app.bk_buttons[4]["state"]) == "disabled"
    app.toggle_connection()
    pump(0.1)
    app.initialize()
    until(lambda: app.local_reference, 3)
    app.bk_vars["paper_z"].set("10")
    app.confirm_paper()
    assert app.bk_confirmed()
    # The global STOP and studio dock remain visible at the minimum layouts.
    for geometry in ("1160x900", "1360x940"):
        app.geometry(geometry)
        pump(0.1)
        assert (
            app.stop_btn.winfo_rooty() + app.stop_btn.winfo_height()
            <= app.winfo_rooty() + app.winfo_height()
        )
        assert (
            app.current_btn.winfo_rooty() + app.current_btn.winfo_height()
            <= app.winfo_rooty() + app.winfo_height()
        )
        for button in (app.preview_btn, app.move_btn, app.current_btn):
            assert button.winfo_ismapped()
            assert (
                button.winfo_rooty() + button.winfo_height()
                <= button.master.winfo_rooty() + button.master.winfo_height()
            ), ("clipped XYZ button", geometry)
    for geometry in ("1080x880", "1200x920"):
        app.bk_window.geometry(geometry)
        pump(0.1)
        assert (
            app.bk_stop_button.winfo_rooty() + app.bk_stop_button.winfo_height()
            <= app.bk_window.winfo_rooty() + app.bk_window.winfo_height()
        )
        for field in app.bk_fields:
            assert (
                field.winfo_rooty() + field.winfo_height()
                <= app.bk_stop_button.winfo_rooty()
            )
    screenshot(app, "cartesian_nc_R13_dashboard.png")
    screenshot(app.bk_window, "cartesian_nc_R13_flower_studio.png")
    model = app.model
    epoch = app.state.epoch
    for pattern in (
        "FLOWER",
        "HEART",
        "STAR",
        "CIRCLE",
        "ELLIPSE",
        "SQUARE",
        "TRIANGLE",
        "DIAMOND",
    ):
        sent = len(app.link.sent)
        app.choose_pattern(pattern)
        pump(0.05)
        assert app.bk_title.get() == PATTERNS[pattern][0] and app.bk_confirmed()
        assert not any(
            s.startswith(("MOVE ", "PATH ", "SEG ", "GO "))
            for s in app.link.sent[sent:]
        )
        app.start_bk()
        until(lambda: app.bk_active or not app.planning, 6)
        assert app.bk_active
        assert all(str(b[1]["state"]) == "disabled" for b in app.bk_tiles.values())
        app.choose_pattern("BK")
        assert app.bk_vars["pattern"].get() == pattern  # cannot change an active job
        until(lambda: not app.bk_active and not app.program_points, 20)
        assert app.local_reference and app.model is model and app.state.epoch == epoch
        assert abs(app.model.from_steps(app.state.pos)[2] - 12) < 0.001
        assert all(len(trail) > 2 for trail in app.bk_trails)
        assert sum(s.startswith("PATH ") for s in app.link.sent[sent:]) == (
            7 if pattern == "FLOWER" else 1
        )
    app.choose_pattern("HEART")
    app.bk_vars["center_y"].set("0")
    sent = len(app.link.sent)
    app.preview_bk()
    assert not app.bk_overlay
    app.start_bk()
    until(lambda: not app.planning, 4)
    assert not any(
        s.startswith(("MOVE ", "PATH ", "SEG ", "GO ")) for s in app.link.sent[sent:]
    )
    app.bk_vars["center_y"].set("165")
    app.preview_bk()
    app.choose_pattern("CIRCLE")
    app.link.freeze_path = True
    app.start_bk()
    until(lambda: app.path_upload is not None and app.path_upload.started, 6)
    app.stop()
    pump(0.15)
    assert not app.path_upload and not app.local_reference and not app.bk_active
    print(
        "R13: gallery/selection, minimum layouts/STOP visibility, all eight templates, paper gate, active-edit block, final lifts, full preflight and buffered STOP passed"
    )
finally:
    app.close()
    temp.cleanup()
