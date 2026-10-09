from pathlib import Path
import sys
import unittest
import math
from dataclasses import replace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from support import fixture
from cartesian_nc_model import NCConfig
from cartesian_nc_drawing import (
    BKSettings,
    bk_strokes,
    pattern_strokes,
    pattern_dimensions,
    compile_bk,
    ink_distance,
    fit_drawing,
    segment_distance,
)
from cartesian_nc_curves import flatten_cubic


class DrawingTests(unittest.TestCase):
    def setUp(self):
        self.model = NCConfig().model(fixture())

    def test_cubic_subdivision_bounds_chord_length_and_curve_error(self):
        control = ((0, 0), (0, 10), (10, 10), (10, 0))
        points = ((0, 0), *flatten_cubic(control, 0.75, 0.025))
        self.assertEqual(points[-1], (10, 0))
        self.assertTrue(
            all(math.dist(a, b) <= 0.75 for a, b in zip(points, points[1:]))
        )
        for i in range(1001):
            u = i / 1000
            v = 1 - u
            p = tuple(
                v**3 * a + 3 * v * v * u * b + 3 * v * u * u * c + u**3 * d
                for a, b, c, d in zip(*control)
            )
            self.assertLessEqual(
                min(segment_distance(p, a, b) for a, b in zip(points, points[1:])),
                0.025,
            )
        with self.assertRaises(ValueError):
            flatten_cubic(control, 0.75, 0.025, cancelled=lambda: True)

    def test_fit_enlarges_all_patterns_and_preflights_more_than_500_total_vertices(
        self,
    ):
        for pattern, size in (("BK", 20), ("LINH", 40), ("THOA", 40)):
            settings = BKSettings(pattern=pattern, size_mm=size, paper_z=10)
            fitted = fit_drawing(self.model, (35200, 0, 1800), settings)
            self.assertGreater(fitted.size_mm, size * 2)
            self.assertEqual(fitted.center_x, settings.center_x)
            self.assertEqual(fitted.center_y, settings.center_y)
            self.assertEqual(fitted.paper_z, 10)
            program = compile_bk(self.model, (35200, 0, 1800), fitted)
            self.assertLessEqual(program.max_model_error_mm, 0.25)
            self.assertTrue(all(len(run.segments) <= 500 for run in program.runs))
            if pattern != "BK":
                self.assertGreater(len(program.operations), 500)
            self.assertEqual(program.points[-1].z, 12)
        with self.assertRaises(ValueError):
            fit_drawing(self.model, (35200, 0, 1800), BKSettings(center_y=0))
        with self.assertRaisesRegex(ValueError, "hủy"):
            fit_drawing(
                self.model, (35200, 0, 1800), BKSettings(), cancelled=lambda: True
            )

    def test_same_bk_geometry_as_reference_sketch(self):
        settings = BKSettings()
        strokes = bk_strokes(settings)
        self.assertEqual(len(strokes), 3)
        self.assertEqual(strokes[0][0].xy, (-10, 175))
        self.assertAlmostEqual(strokes[0][-1].xy[1], 175)
        for stroke in strokes:
            for vertex in stroke:
                self.assertLess(
                    ink_distance(vertex.xy, vertex.part, settings),
                    settings.size_mm * 0.0003 / 4 + 1e-10,
                )
                self.assertTrue(
                    -10 - 1e-8 <= vertex.xy[0] <= 10 + 1e-8
                    and 155 - 1e-8 <= vertex.xy[1] <= 175 + 1e-8
                )
        self.assertEqual(strokes[1][0].xy, (0, 175))
        self.assertEqual(strokes[1][-1].xy, (0, 155))
        self.assertEqual(strokes[2][0].xy, (10, 175))
        self.assertEqual(strokes[2][-1].xy, (10, 155))

    def test_uniform_scaling_and_small_chords(self):
        settings = BKSettings(size_mm=24, center_x=5, center_y=164, step_mm=0.5)
        for stroke in bk_strokes(settings):
            for a, b in zip(stroke, stroke[1:]):
                self.assertLessEqual(math.dist(a.xy, b.xy), 0.5 + 1e-8)

    def test_pen_transfers_and_first_lift_have_no_xy_contact(self):
        program = compile_bk(self.model, (35200, 0, 1800), BKSettings(paper_z=10))
        self.assertLessEqual(len(program.operations), 500)
        self.assertLessEqual(program.max_model_error_mm, 0.25)
        start = self.model.fk(self.model.from_steps(program.start_steps))
        self.assertEqual(program.operations[0].kind, "lift")
        self.assertEqual(program.operations[0].point.xyz[:2], start[:2])
        self.assertEqual(sum(op.kind == "lower" for op in program.operations), 3)
        self.assertEqual(sum(op.kind == "travel" for op in program.operations), 3)
        last = program.operations[0]
        for op in program.operations[1:]:
            if op.kind == "travel":
                self.assertEqual(last.point.z, 12)
                self.assertEqual(op.point.z, 12)
            if op.kind in ("lift", "lower"):
                self.assertEqual(op.point.xyz[:2], last.point.xyz[:2])
            if op.kind == "ink":
                self.assertEqual(op.point.z, 10)
            last = op
        self.assertEqual(program.operations[-1].kind, "lift")
        self.assertEqual(program.points[-1].z, 12)

    def test_dry_run_never_lowers_pen(self):
        program = compile_bk(self.model, (35200, 0, 1800), BKSettings(paper_z=10), True)
        self.assertEqual({p.z for p in program.points}, {12})
        self.assertFalse(any(op.kind in ("lower", "ink") for op in program.operations))

    def test_invalid_geometry_z_and_precision_rejected(self):
        for settings in (
            BKSettings(),
            BKSettings(paper_z=24),
            BKSettings(paper_z=10, center_y=0),
            BKSettings(paper_z=10, tolerance_mm=0.05),
            BKSettings(paper_z=10, draw_speed=0.05),
        ):
            with self.assertRaises(ValueError):
                compile_bk(self.model, (35200, 0, 1800), settings)
        with self.assertRaisesRegex(ValueError, "hủy"):
            compile_bk(
                self.model,
                (35200, 0, 1800),
                BKSettings(paper_z=10),
                cancelled=lambda: True,
            )

    def test_calibrated_scale_and_negative_branch(self):
        status = replace(
            fixture(), factor=(1638400, 40960, 40960), range=(40000, 7200, 7200)
        )
        model = NCConfig().model(status)
        program = compile_bk(model, (35200, 0, 1800), BKSettings(paper_z=10))
        self.assertLess(program.max_model_error_mm, 0.1)
        model = replace(NCConfig(), park_deg=(0, -45)).model(
            replace(fixture(), pos=(35200, 0, -1800))
        )
        self.assertLessEqual(
            compile_bk(
                model, (35200, 0, -1800), BKSettings(paper_z=10)
            ).max_model_error_mm,
            0.25,
        )

    def test_linh_thoa_uniform_scale_and_eight_separate_strokes(self):
        for word in ("LINH", "THOA"):
            settings = BKSettings(pattern=word, size_mm=40, paper_z=10)
            width, height = pattern_dimensions(settings)
            self.assertEqual(width, 40)
            self.assertAlmostEqual(height, 40 / 3.7)
            strokes = pattern_strokes(settings)
            self.assertEqual(len(strokes), 8)
            vertices = [v for stroke in strokes for v in stroke]
            self.assertAlmostEqual(min(v.xy[0] for v in vertices), -20)
            self.assertAlmostEqual(max(v.xy[0] for v in vertices), 20)
            for vertex in vertices:
                allowed = (
                    height * 0.0003 / 2 + 1e-10 if vertex.part == "O_curve" else 1e-10
                )
                self.assertLess(ink_distance(vertex.xy, vertex.part, settings), allowed)

    def test_words_complete_checked_program_and_lift_per_stroke(self):
        for word in ("LINH", "THOA"):
            program = compile_bk(
                self.model,
                (35200, 0, 1800),
                BKSettings(pattern=word, size_mm=40, paper_z=10),
            )
            self.assertLessEqual(program.max_model_error_mm, 0.25)
            self.assertLess(len(program.points), 500)
            self.assertEqual(sum(op.kind == "lower" for op in program.operations), 8)
            self.assertEqual(program.operations[-1].kind, "lift")
            self.assertTrue(
                all(point.name.startswith(word) for point in program.points)
            )


if __name__ == "__main__":
    unittest.main()
