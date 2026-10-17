"""Unit tests for R15 GUI & CAD/CAM features: full A-Z/0-9 stroke font, parametric pattern generators, and 3D STL Digital Twin mesh assembly."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from cartesian_nc_font import (
    GLYPHS,
    custom_text_strokes,
    parametric_generator_strokes,
    word_geometry,
)
from cartesian_nc_3d import Camera3D, build_xscara_3d_parts


class TestCartesianR15Features(unittest.TestCase):
    def test_word_geometry_backward_compatibility(self):
        """Ensure LINH and THOA stroke geometry remain identical for existing drawing plans."""
        for key in ("LINH", "THOA"):
            w_mm, h_mm, strokes = word_geometry(key, 65.0, 0.0, 165.0)
            self.assertAlmostEqual(w_mm, 65.0)
            self.assertGreater(h_mm, 5.0)
            self.assertGreaterEqual(len(strokes), 4)

    def test_full_alphanumeric_stroke_font(self):
        """Verify all A-Z, 0-9, and symbols exist in GLYPHS and generate valid strokes."""
        for ch in "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-+./":
            self.assertIn(ch, GLYPHS)
            self.assertGreaterEqual(len(GLYPHS[ch]), 1)

        strokes = custom_text_strokes("SCARA 2026-R15", width_mm=70.0, rotation_deg=15.0)
        self.assertGreater(len(strokes), 15)
        for s in strokes:
            self.assertGreaterEqual(len(s), 2)

    def test_parametric_generators(self):
        """Verify SPIRAL, GRID, ROSE, and POLYGON parametric generators."""
        for kind in ("SPIRAL", "GRID", "ROSE", "POLYGON"):
            strokes = parametric_generator_strokes(
                kind, param=5, width_mm=50.0, rotation_deg=20.0
            )
            self.assertGreaterEqual(len(strokes), 1)
            for s in strokes:
                self.assertGreaterEqual(len(s), 2)

    def test_3d_stl_mesh_loader_and_camera(self):
        """Verify STL mesh loading and decimation from Hardware/arm/STL/ and 3D camera projection."""
        parts = build_xscara_3d_parts(150.0, 135.0)
        self.assertGreaterEqual(len(parts), 15)
        groups = {grp for grp, _rgb, _faces in parts}
        self.assertEqual(groups, {0, 1, 2, 3, 4})
        total_faces = sum(len(faces) for _grp, _rgb, faces in parts)
        self.assertGreater(total_faces, 400)
        self.assertLess(total_faces, 3500)

        cam = Camera3D(800, 600, azimuth_deg=135.0, elevation_deg=28.0, zoom=1.45)
        sx, sy, depth = cam.project(0.0, 165.0, 10.0)
        self.assertTrue(0 < sx < 800)
        self.assertTrue(0 < sy < 600)
        self.assertIsInstance(depth, float)

    def test_improvement1_quick_rehome_demo(self):
        """Verify DemoLink supports INIT <session> <job> QUICK with faster completion."""
        import time
        from cartesian_nc_demo import DemoLink

        demo = DemoLink("DEMO")
        try:
            demo.send("HELLO 1")
            t0 = time.monotonic()
            demo.send("INIT 1 1 QUICK")
            self.assertTrue(demo.initializing)
            self.assertLess(demo.ends - t0, 0.7)
        finally:
            demo.close()

    def test_improvement2_auto_backlash_estimation_and_commands(self):
        """Verify estimate_backlash_from_status and NCConfig.backlash_pulses command emission."""
        from dataclasses import replace
        from cartesian_nc_model import NCConfig, estimate_backlash_from_status
        from support import fixture

        status = replace(
            fixture(),
            n1=(40000, 4808, 7214),
            n2=(40000, 4796, 7194),
        )
        est = estimate_backlash_from_status(status)
        self.assertEqual(est, (6, 10))

        cfg_zero = NCConfig().validate()
        self.assertEqual(cfg_zero.backlash_pulses, (0, 0))
        self.assertTrue(all(not c.startswith("BACKLASH") for c in cfg_zero.commands(7)))

        cfg_bl = replace(cfg_zero, backlash_pulses=est).validate()
        self.assertEqual(cfg_bl.backlash_pulses, (6, 10))
        self.assertIn("BACKLASH 7 6 10", cfg_bl.commands(7))

    def test_improvement3_two_point_switch_angle_calibration(self):
        """Verify 2-point switch-angle calibration recovers exact physical switch offsets."""
        from cartesian_nc_model import NCConfig
        from cartesian_nc_calibration import solve_switch_angle_offsets
        from support import fixture

        cfg = NCConfig().validate()
        model = cfg.model(fixture())

        # Suppose physical switches are shifted by (+1.75 deg on J1, -1.25 deg on J2)
        true_d1, true_d2 = 1.75, -1.25
        samples = []
        for q_rep in ((-25.0, 50.0, 12.0), (20.0, 35.0, 12.0)):
            steps = model.to_steps(q_rep)
            q_actual = model.from_steps(steps)
            true_xyz = model.fk((q_actual[0] + true_d1, q_actual[1] + true_d2, q_actual[2]))
            samples.append((steps, (true_xyz[0], true_xyz[1])))

        res = solve_switch_angle_offsets(model, cfg, samples)
        self.assertAlmostEqual(res.delta_deg[0], true_d1, places=2)
        self.assertAlmostEqual(res.delta_deg[1], true_d2, places=2)
        self.assertGreater(res.rms_error_before_mm, 3.0)
        self.assertLess(res.rms_error_after_mm, 0.01)
        self.assertAlmostEqual(res.new_joint_limits_deg[0][0], -90.0 + true_d1, places=2)
        self.assertAlmostEqual(res.new_joint_limits_deg[1][0], -90.0 + true_d2, places=2)

    def test_improvement5_three_point_paper_bed_leveling(self):
        """Verify 3-point paper plane fit and per-stroke Z tilt compensation in compile_bk."""
        from cartesian_nc_model import NCConfig
        from cartesian_nc_calibration import fit_paper_plane
        from cartesian_nc_drawing import BKSettings, compile_bk
        from support import fixture

        # True tilted plane: Z(x, y) = 12.0 + 0.02*(x - 0.0) - 0.015*(y - 165.0)
        cx, cy = 0.0, 165.0
        kx_true, ky_true, z0_true = 0.02, -0.015, 12.0
        pts = [
            (x, y, z0_true + kx_true * (x - cx) + ky_true * (y - cy))
            for x, y in ((-30.0, 150.0), (30.0, 150.0), (0.0, 185.0))
        ]
        fit = fit_paper_plane(pts, ref_xy=(cx, cy))
        self.assertAlmostEqual(fit.z_ref_mm, z0_true, places=3)
        self.assertAlmostEqual(fit.paper_tilt[0], kx_true, places=5)
        self.assertAlmostEqual(fit.paper_tilt[1], ky_true, places=5)
        self.assertLess(fit.max_residual_mm, 1e-4)

        model = NCConfig().validate().model(fixture())
        settings = BKSettings(
            pattern="BK",
            size_mm=24.0,
            center_x=-65.0,
            center_y=165.0,
            paper_z=12.0,
            paper_tilt=(0.03, -0.02),
        ).validate(paper=True)
        prog = compile_bk(model, (35200, 0, 1800), settings, dry=False)
        lower_zs = [op.point.xyz[2] for op in prog.operations if op.kind == "lower"]
        self.assertGreaterEqual(len(lower_zs), 2)
        # Different strokes across X have distinct compensated Z heights
        self.assertGreater(max(lower_zs) - min(lower_zs), 0.1)
        # Within every continuous StrokeRun, Z steps remain constant for firmware SEG
        for run in prog.runs:
            z_targets = {seg.steps[0] for seg in run.segments}
            self.assertEqual(len(z_targets), 1)


if __name__ == "__main__":
    unittest.main()

