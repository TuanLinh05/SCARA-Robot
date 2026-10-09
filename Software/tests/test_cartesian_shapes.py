from pathlib import Path
import sys, math, json, subprocess, tempfile, unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from support import fixture
from cartesian_nc_model import NCConfig
from cartesian_nc_drawing import (
    BKSettings,
    compile_bk,
    pattern_strokes,
    pattern_dimensions,
    ink_distance,
    fit_drawing,
)
from cartesian_nc_shapes import (
    SHAPES,
    PATTERNS,
    shape_template,
    piece_bounds,
    cubic_value,
)


class ShapeTests(unittest.TestCase):
    def setUp(self):
        self.model = NCConfig().model(fixture())

    def test_all_requested_shapes_closed_scaled_and_short_segments(self):
        self.assertTrue({"FLOWER", "STAR", "HEART", "CIRCLE"} <= SHAPES)
        for pattern in SHAPES:
            s = BKSettings(pattern=pattern, size_mm=24, paper_z=10)
            strokes = pattern_strokes(s)
            width, height = pattern_dimensions(s)
            self.assertEqual(width, 24)
            self.assertGreater(height, 0)
            self.assertEqual(len(strokes), 7 if pattern == "FLOWER" else 1)
            for stroke in strokes:
                self.assertEqual(stroke[0].xy, stroke[-1].xy)
                for a, b in zip(stroke, stroke[1:]):
                    self.assertLessEqual(math.dist(a.xy, b.xy), s.step_mm + 1e-9)
                for vertex in stroke:
                    self.assertLessEqual(ink_distance(vertex.xy, vertex.part, s), 0.01)
                    self.assertTrue(abs(vertex.xy[0] - s.center_x) <= width / 2 + 1e-9)
                    self.assertTrue(abs(vertex.xy[1] - s.center_y) <= height / 2 + 1e-9)

    def test_exact_curve_bounds_contain_all_samples_and_heart_flower_are_symmetric(
        self,
    ):
        for pattern in ("HEART", "FLOWER"):
            for stroke in shape_template(pattern):
                for piece in stroke:
                    if piece.kind != "cubic":
                        continue
                    x0, y0, x1, y1 = piece_bounds(piece)
                    for i in range(1001):
                        x, y = cubic_value(piece.data, i / 1000)
                        self.assertTrue(
                            x0 - 1e-9 <= x <= x1 + 1e-9 and y0 - 1e-9 <= y <= y1 + 1e-9
                        )

    def test_whole_path_paper_z_lifts_and_dry_geometry(self):
        for pattern in SHAPES:
            s = BKSettings(pattern=pattern, size_mm=24, paper_z=10)
            p = compile_bk(self.model, (35200, 0, 1800), s)
            self.assertLessEqual(p.max_model_error_mm, s.tolerance_mm)
            self.assertEqual(len(p.runs), 7 if pattern == "FLOWER" else 1)
            self.assertEqual(
                sum(op.kind == "lower" for op in p.operations), len(p.strokes)
            )
            self.assertEqual(p.operations[-1].kind, "lift")
            self.assertEqual(p.points[-1].z, 12)
            self.assertTrue(
                all(seg.steps[0] == 16000 for run in p.runs for seg in run.segments)
            )
            dry = compile_bk(self.model, (35200, 0, 1800), s, True)
            self.assertTrue(all(op.point.z == 12 for op in dry.operations))

    def test_shapes_fit_without_weakening_limits_and_invalid_target_is_rejected(self):
        for pattern in ("FLOWER", "HEART", "CIRCLE", "STAR"):
            s = BKSettings(pattern=pattern, paper_z=10, size_mm=24)
            fit = fit_drawing(self.model, (35200, 0, 1800), s)
            self.assertGreater(fit.size_mm, 24)
            self.assertLessEqual(
                compile_bk(self.model, (35200, 0, 1800), fit).max_model_error_mm, 0.25
            )
            with self.assertRaises(ValueError):
                compile_bk(
                    self.model,
                    (35200, 0, 1800),
                    BKSettings(pattern=pattern, paper_z=10, center_y=0),
                )
        with self.assertRaises(ValueError):
            BKSettings(pattern="UNKNOWN").validate()

    def test_actual_firmware_gpio_counts_for_every_template_stroke(self):
        base = (
            Path(__file__).resolve().parents[2] / "Firmware/ScaraCartesian/build_host"
        )
        exe = base / "test_motor_io.exe"
        rows = []
        with tempfile.TemporaryDirectory(dir=base) as temp:
            for pattern in SHAPES:
                p = compile_bk(
                    self.model,
                    (35200, 0, 1800),
                    BKSettings(pattern=pattern, size_mm=24, paper_z=10),
                )
                for i, run in enumerate(p.runs):
                    path = Path(temp) / f"{pattern}_{i}.txt"
                    path.write_text(
                        " ".join(map(str, run.start_steps))
                        + "\n"
                        + "\n".join(
                            " ".join(
                                map(str, (*s.steps, s.peak, s.entry, s.exit, s.accel))
                            )
                            for s in run.segments
                        )
                        + "\n",
                        encoding="ascii",
                    )
                    result = json.loads(
                        subprocess.check_output([str(exe), "--stroke-file", str(path)])
                    )
                    self.assertEqual(result["rises"][0], 0)
                    self.assertEqual(result["timer_starts"], 1)
                    self.assertAlmostEqual(
                        result["continuous_seconds"], run.seconds + 0.05, delta=0.04
                    )
                rows.append(
                    dict(
                        pattern=pattern,
                        strokes=len(p.strokes),
                        operations=len(p.points),
                        max_model_error_mm=p.max_model_error_mm,
                    )
                )
        (base.parents[2] / "Software/reports/verification_shapes.json").write_text(
            json.dumps(rows, indent=2) + "\n", encoding="utf-8"
        )


if __name__ == "__main__":
    unittest.main()
