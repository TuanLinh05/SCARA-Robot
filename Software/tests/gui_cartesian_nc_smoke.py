"""Exercises the new GUI without opening any hardware COM port."""

from pathlib import Path
import sys
import ctypes
import time
import json
import tempfile
from dataclasses import replace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from cartesian_nc_control import NCApp
from cartesian_nc_demo import DemoLink
from cartesian_nc_model import NCConfig

base = Path(__file__).resolve().parents[2] / "Firmware/ScaraCartesian/build_host"
temp = tempfile.TemporaryDirectory(dir=base)
assert Path(temp.name).resolve().is_relative_to(base.resolve())
config_path = Path(temp.name) / "cartesian_nc_config.json"
NCConfig().save(config_path)
app = NCApp(factory=DemoLink, config_path=config_path, demo=True)
app.focus_force()
errors = []
app.report_callback_exception = lambda *e: errors.append(e)


def pump(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.update()
        time.sleep(0.005)
    assert not errors, errors


try:
    app.toggle_connection()
    pump(0.2)
    first_log = app.last_log_path
    assert first_log and first_log.is_file()
    assert app.fresh() and not app.local_reference
    assert str(app.move_btn["state"]) == "disabled"
    # Workflow tests do not depend on whichever app the human is using.
    # Explicit foreground-loss cases below still exercise stop policy.
    app._foreground = lambda: True
    original_foreground = app._foreground
    app.initialize()
    pump(0.25)
    assert app.state.busy and app.task_kind == "INIT"
    assert app.state.pol == 3  # startup configuration reaches the USB peer
    app._foreground = lambda: False
    app._focus_check()
    app._unmap_check()
    pump(0.2)
    assert (
        app.state.busy and app.active_job is not None
    )  # autonomous HOME stays alive in background
    app._foreground = original_foreground
    app.focus_force()
    pump(1.5)
    assert (
        app.local_reference
        and app.model
        and app.state.referenced
        and app.state.stage == 8
    )
    assert str(app.move_btn["state"]) == "normal"
    xyz = app.model.fk((30, 30, 20))
    for v, x in zip(app.xyz, xyz):
        v.set(str(x))
    app.preview_target()
    pump(0.3)
    assert app.preview and not app.planning
    app.move()
    pump(0.15)
    assert app.active_job is not None
    original_tk_focus = app.focus_displayof
    app.focus_displayof = lambda: None
    assert app._foreground()
    app._focus_check()
    assert app.active_job is not None
    app.focus_displayof = original_tk_focus
    pump(0.75)
    assert app.active_job is None and app.local_reference
    q = app.model.from_steps(app.state.pos)
    assert abs(q[0] - 30) < 0.1 and abs(q[1] - 30) < 0.1
    for widget in (app.home_btn, app.move_btn, app.hold_btn):
        assert widget.winfo_ismapped()
        assert (
            widget.winfo_rooty() + widget.winfo_height()
            <= app.winfo_rooty() + app.winfo_height()
        )
    app.status_tabs.select(1)
    pump(0.05)
    assert app.fw_label.winfo_ismapped()
    assert (
        app.fw_label.winfo_rooty() + app.fw_label.winfo_height()
        <= app.winfo_rooty() + app.winfo_height()
    )
    app.status_tabs.select(0)
    pump(0.05)
    # Retain the screenshot of this app alone.
    try:
        from PIL import ImageGrab

        hwnd = ctypes.windll.user32.GetAncestor(app.winfo_id(), 2)
        ImageGrab.grab(window=hwnd).save(base / "cartesian_nc_preview.png")
    except (ImportError, OSError) as e:
        print("Optional own-window screenshot unavailable:", e)
    # Planner runs independently of UI; an aborted result cannot issue MOVE.
    xyz = app.model.fk((-20, 70, 20))
    for v, x in zip(app.xyz, xyz):
        v.set(str(x))
    app.start_plan(True)
    app.stop()
    pump(0.3)
    assert app.active_job is None and not app.state.busy
    app.hold(False)
    pump(0.2)
    assert app.state.holding == 1 and not app.local_reference
    app.hold(True)
    pump(0.2)
    assert app.state.holding == 7
    app.config = replace(app.config, home_in_background=False)
    app.initialize()
    pump(0.25)
    assert app.state.busy
    app._foreground = lambda: False
    app._focus_check()
    pump(0.2)
    assert (
        not app.local_reference
        and not app.state.busy
        and app.state.reason == "stop_focus"
    )
    app._foreground = original_foreground
    app.config = replace(app.config, home_in_background=True)
    app.initialize()
    pump(1.9)
    assert app.local_reference
    xyz = app.model.fk((-20, 70, 20))
    for v, x in zip(app.xyz, xyz):
        v.set(str(x))
    app.move()
    pump(0.15)
    assert app.state.busy
    saved_model = app.model
    saved_epoch = app.state.epoch
    app._foreground = lambda: False
    app._focus_check()
    app._unmap_check()
    pump(0.2)
    assert (
        app.local_reference
        and app.model is saved_model
        and app.state.epoch == saved_epoch
    )
    pump(0.6)
    assert not app.state.busy and app.state.reason == "complete" and app.local_reference
    app._foreground = original_foreground
    # No more valid telemetry: stop/disconnect without renewing the lease.
    app.link.closed_event.set()
    app.link.thread.join(timeout=0.2)
    while not app.link.events.empty():
        app.link.events.get_nowait()
    # Show the root fault independently of the currently recovered GPIO.
    app.state = replace(
        app.state,
        busy=0,
        referenced=0,
        stage=9,
        switches=0,
        raw=0,
        input_axis=1,
        input_kind=2,
        input_bits=12,
        input_at=12345,
        conflicts=2,
        reason="both_latched",
        cal_error=(1, 0, 0),
        phase=(11, 0, 0),
        failed_phase=(1, 0, 0),
    )
    app.rx = time.monotonic()
    app.render()
    app.update()
    assert "J1" in app.input_details.get() and "PB0/PB1=1/1" in app.input_details.get()
    assert (
        "hiện tại 0/0" in app.input_details.get()
        and "lỗi đã giữ" in app.switch_text.get()
    )
    assert (
        str(app.home_btn["state"]) == "disabled"
        and str(app.move_btn["state"]) == "disabled"
    )
    assert (
        app.input_label.winfo_rooty() + app.input_label.winfo_height()
        <= app.winfo_rooty() + app.winfo_height()
    )
    try:
        from PIL import ImageGrab

        hwnd = ctypes.windll.user32.GetAncestor(app.winfo_id(), 2)
        ImageGrab.grab(window=hwnd).save(base / "cartesian_nc_R7_fault_preview.png")
    except (ImportError, OSError) as e:
        print("Optional own-window screenshot unavailable:", e)
    # An idle J1 transient is historical input data, not a stopped Z task.
    app.state = replace(
        app.state,
        busy=1,
        referenced=0,
        stage=1,
        input_axis=1,
        input_kind=6,
        conflicts=0,
        reason="initializing",
        idle_ignored=15,
        cal_error=(0, 0, 0),
        phase=(4, 0, 0),
    )
    app.render()
    assert "xung thoáng qua ở trục đứng yên đã lọc" in app.input_details.get()
    assert app.input_label.cget("fg") == "#60d5ae"
    app.rx = time.monotonic() - 1
    app.poll()
    assert app.link is None and not app.local_reference
    records = [
        json.loads(line) for line in first_log.read_text(encoding="utf-8").splitlines()
    ]
    kinds = {r["type"] for r in records}
    assert {
        "connection",
        "home_start",
        "tx",
        "ack",
        "status",
        "stop_sent",
        "disconnect",
        "log_end",
    } <= kinds
    assert any(r.get("command") == "COUPLE 7 1 128" for r in records)
    assert any(r["type"] == "status" and "beta_q" in r["status"] for r in records)
    print(
        "GUI: home sequence, measured model, preview, async planner, MOVE ACK, STOP, release, stale USB passed"
    )
finally:
    app.close()
    temp.cleanup()
