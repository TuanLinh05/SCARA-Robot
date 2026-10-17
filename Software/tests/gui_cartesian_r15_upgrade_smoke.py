"""End-to-end GUI smoke test for R15 upgrades: 3D STL Digital Twin, Step-Jog, Custom Text/Parametric Studio, and Telemetry Analytics & Report Export."""

from pathlib import Path
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from cartesian_nc_control import NCApp
from cartesian_nc_demo import DemoLink
from cartesian_nc_model import NCConfig
from cartesian_nc_3d import build_xscara_3d_parts, export_canvas_to_svg


class FastR15Demo(DemoLink):
    def __init__(self, port):
        self.sent = []
        super().__init__(port)

    def send(self, line):
        self.sent.append(line)
        super().send(line)
        if line.startswith("MOVE "):
            self.ends = time.monotonic() + 0.003

    def _read(self):
        while not self.closed_event.wait(0.008):
            self.advance()


base = Path(__file__).resolve().parents[2] / "Firmware/ScaraCartesian/build_host"
base.mkdir(parents=True, exist_ok=True)
temp = tempfile.TemporaryDirectory(dir=base)
cfg_path = Path(temp.name) / "config.json"
NCConfig().save(cfg_path)

app = NCApp(factory=FastR15Demo, config_path=cfg_path, demo=True)
app._foreground = lambda: True
errors = []
app.report_callback_exception = lambda *e: errors.append(e)


def pump(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.update()
        time.sleep(0.003)
    assert not errors, errors


def until(condition, seconds=10):
    end = time.monotonic() + seconds
    while not condition() and time.monotonic() < end:
        pump(0.02)
    assert condition(), (app.message.get(), app.bk_note.get())


try:
    # 1. Verify 3D STL mesh loader loads actual Hardware/arm/STL parts
    parts = build_xscara_3d_parts(150.0, 135.0)
    total_faces = sum(len(faces) for _grp, _rgb, faces in parts)
    assert len(parts) >= 15 and total_faces > 400, (len(parts), total_faces)

    # 2. Connect and run HOME + CALIB
    app.toggle_connection()
    pump(0.1)
    app.initialize()
    until(lambda: app.local_reference and not app.planning, 4)
    pump(0.1)

    # Verify all 6 live metric cards & 6 visual switch LEDs are populated
    assert app.joints_readout.get() != "— / —"
    assert "|sin J2|=" in app.joints_sub_readout.get()
    assert "FIFO:" in app.pps_fifo_sub.get()
    assert len(app.switch_leds) == 6
    assert all("OK" in badge.cget("text") for _name, badge in app.switch_leds)

    # 3. Test 3D Digital Twin View on Main Canvas & Standalone 3D Window
    app.set_workspace_mode("3D")
    pump(0.1)
    poly_count = sum(
        1 for item in app.canvas.find_all() if app.canvas.type(item) == "polygon"
    )
    assert poly_count > 400, f"Expected >400 3D polygons on main canvas, got {poly_count}"
    app.set_3d_preset(90, 85, for_window=False)
    pump(0.05)
    svg_3d_path = Path(temp.name) / "scara_3d_test.svg"
    export_canvas_to_svg(app.canvas, svg_3d_path)
    assert svg_3d_path.is_file() and svg_3d_path.stat().st_size > 10000

    app.open_3d_window()
    pump(0.1)
    assert app.win_3d.winfo_exists()
    win3d_polys = sum(
        1 for item in app.canvas_3d.find_all() if app.canvas_3d.type(item) == "polygon"
    )
    assert win3d_polys > 400
    app.set_workspace_mode("2D")
    pump(0.05)

    # 4. Test Step-Jog Pad & Quick Pose Presets ("Jog & Tùy biến" tab)
    app.workspace_tabs.select(2)
    pump(0.05)
    app.jog_step.set("5")
    x_before = float(app.xyz[0].get())
    app.jog_by(dx=1)
    until(lambda: not app.state.busy and not app.planning, 3)
    x_after = app.model.fk(app.model.from_steps(app.state.pos))[0]
    assert abs((x_after - x_before) - 5.0) < 0.25, (x_before, x_after)

    app.preset_paper_center()
    until(lambda: not app.state.busy and not app.planning, 4)
    xy_now = app.model.fk(app.model.from_steps(app.state.pos))[:2]
    assert abs(xy_now[0] - 0.0) < 0.3 and abs(xy_now[1] - 165.0) < 0.3, xy_now
    app.workspace_tabs.select(0)
    pump(0.05)

    # 5. Test Custom Text (A-Z, 0-9) & Parametric Generators in Drawing Studio
    app.open_bk()
    pump(0.1)
    app.bk_vars["paper_z"].set("10")
    app.confirm_paper()
    assert app.bk_confirmed()

    app.custom_text_var.set("SCARA 26")
    app.custom_rot_var.set("10")
    app.apply_custom_text()
    pump(0.1)
    assert app.bk_vars["pattern"].get() == "CUSTOM"
    assert len(app.bk_overlay) >= 7

    for kind in ("SPIRAL", "GRID", "ROSE", "POLYGON"):
        app.custom_param_var.set("4")
        app.apply_parametric_pattern(kind)
        pump(0.05)
        assert app.bk_vars["pattern"].get() == "CUSTOM" and len(app.bk_overlay) >= 1

    # Run a full ink drawing of the custom parametric polygon
    app.bk_vars["size_mm"].set("22")
    app.apply_parametric_pattern("POLYGON")
    pump(0.05)
    app.start_bk(False)
    until(lambda: app.bk_active or not app.planning, 4)
    assert app.bk_active
    until(lambda: not app.bk_active and not app.program_points, 15)
    assert app.local_reference

    # 6. Test Telemetry Oscilloscope & Academic Report Studio
    app.open_analytics()
    pump(0.1)
    assert app.analytics_window.winfo_exists()
    assert len(app.telemetry_samples) >= 5
    for tab_idx in range(4):
        app.select_analytics_tab(tab_idx)
        pump(0.05)
        assert len(app.analytics_canvas.find_all()) > 10

    summary = dict(app.compute_telemetry_summary())
    assert "Số mẫu thu thập (Samples)" in summary
    assert "Quãng đường đầu bút 3D (Total Path)" in summary

    chart_svg = Path(temp.name) / "telemetry_chart.svg"
    app.export_analytics_svg(chart_svg)
    assert chart_svg.is_file() and chart_svg.stat().st_size > 1000

    csv_path = Path(temp.name) / "telemetry_data.csv"
    app.export_telemetry_csv(csv_path)
    assert csv_path.is_file() and "manipulability" in csv_path.read_text(encoding="utf-8-sig")

    md_path = Path(temp.name) / "Bao_cao_SCARA.md"
    app.export_markdown_report(md_path)
    md_text = md_path.read_text(encoding="utf-8")
    assert "BÁO CÁO KỸ THUẬT & SỐ LIỆU THỰC NGHIỆM" in md_text
    assert "\\theta_B = \\theta_2 + k \\cdot \\theta_1" in md_text

    # 7. Test Quick Re-Home & 3-Tab Advanced Calibration Wizard
    app.quick_initialize()
    until(lambda: app.local_reference and not app.planning, 3)
    assert any(line.endswith(" QUICK") and line.startswith("INIT ") for line in app.link.sent)

    app.open_calibration_wizard()
    pump(0.1)
    assert app.calibration_window.winfo_exists()
    app._fill_estimated_backlash()
    assert app.cal_bl_j1_var.get() == "4" and app.cal_bl_j2_var.get() == "6"
    app._apply_backlash_calibration()
    pump(0.05)
    assert app.config.backlash_pulses == (4, 6)

    app._capture_switch_sample(0)
    app._solve_and_apply_switch_offsets()
    assert "Thành công!" in app.cal_sw_result_var.get()

    app.cal_bed_vars[0][0].set("-90.0")
    app.cal_bed_vars[0][1].set("150.0")
    app.cal_bed_vars[0][2].set("11.5")
    app.cal_bed_vars[1][0].set("-40.0")
    app.cal_bed_vars[1][1].set("150.0")
    app.cal_bed_vars[1][2].set("12.5")
    app.cal_bed_vars[2][0].set("-65.0")
    app.cal_bed_vars[2][1].set("185.0")
    app.cal_bed_vars[2][2].set("12.0")
    app._solve_and_apply_bed_leveling()
    assert abs(app.paper_tilt[0] - 0.02) < 1e-4
    app._reset_bed_leveling()
    assert app.paper_tilt == (0.0, 0.0)

    print(
        "R15 GUI: 3D STL Digital Twin, 6 live metric cards, 6 switch LEDs, Step-Jog pad, Custom A-Z/0-9 & Parametric Drawing Studio, 4-channel Telemetry Oscilloscope, Quick Re-Home, and 3-Tab Calibration Wizard passed"
    )
finally:
    app.close()
    temp.cleanup()
