import json
import math
from dataclasses import replace, asdict
from pathlib import Path
import subprocess
import sys
import unittest
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from cartesian_nc_model import NCConfig
from cartesian_nc_protocol import NCStatus, NCDecoder, parse_message
from cartesian_nc_demo import DemoLink
from cartesian_nc_upgrade import (
    upgrade_r3,
    upgrade_r4,
    upgrade_r5,
    upgrade_r6,
    upgrade_r7,
    upgrade_r10,
)
from cartesian_nc_log import SessionLog

from support import fixture


class NCModelTests(unittest.TestCase):
    def test_tool_offset_measured_model_fk_ik_and_limits(self):
        for offset in ((0, 0), (12, 18), (-10, 0), (0, 20)):
            for park in ((0, 45), (0, -45)):
                cfg = replace(
                    NCConfig(), tool_offset_mm=offset, park_deg=park
                ).validate()
                model = cfg.model(fixture())
                for a in (-35, 0, 25):
                    q = (a, 55 if park[1] > 0 else -55, 10)
                    self.assertTrue(
                        all(
                            abs(x - y) < 1e-8
                            for x, y in zip(model.ik(model.fk(q), q), q)
                        )
                    )
                self.assertAlmostEqual(
                    cfg.reach_mm, 98 + math.hypot(offset[0], 98 + offset[1])
                )
        cfg = NCConfig()
        old = asdict(cfg)
        old.pop("tool_offset_mm")
        self.assertEqual(tuple(NCConfig.from_dict(old).tool_offset_mm), (0, 0))
        with self.assertRaises(ValueError):
            replace(cfg, tool_offset_mm=(201, 0)).validate()

    def test_z_three_mm_s_conversion_migration_and_rate_status(self):
        cfg = NCConfig()
        self.assertEqual(cfg.z_home_speed_mm_s, 3)
        self.assertEqual(cfg.z_home_pps, 4800)
        self.assertEqual(replace(cfg, lead_mm_rev=4).validate().z_home_pps, 2400)
        self.assertEqual(replace(cfg, microsteps=(8, 8, 8)).validate().z_home_pps, 2400)
        with self.assertRaises(ValueError):
            replace(cfg, lead_mm_rev=1).validate()
        old = asdict(cfg)
        old.pop("z_home_speed_mm_s")
        old["z_home_pps"] = 1600
        self.assertEqual(NCConfig.from_dict(old).z_home_pps, 4800)
        current = asdict(cfg)
        current["z_home_speed_mm_s"] = 2.5
        self.assertEqual(NCConfig.from_dict(current).z_home_pps, 4000)
        state = replace(
            fixture(), busy=1, referenced=0, stage=1, mode=2, selected=0, pps=4784
        )
        data = asdict(state)
        data.update(type="status", protocol=5, fw="SCARA_CARTESIAN_NC_V5")
        self.assertIsInstance(parse_message(json.dumps(data)), NCStatus)
        for changes in (dict(mode=1), dict(selected=2), dict(pps=6401)):
            self.assertIsNone(parse_message(json.dumps({**data, **changes})))

    def test_background_configuration_upgrade_keeps_mechanics(self):
        cfg = NCConfig().validate()
        self.assertTrue(cfg.motion_in_background)
        old = asdict(cfg)
        old.pop("motion_in_background")
        upgraded = upgrade_r10(old)
        self.assertTrue(upgraded["motion_in_background"])
        for name in old:
            self.assertEqual(old[name], upgraded[name])
        self.assertFalse(
            upgrade_r10({**old, "motion_in_background": False})["motion_in_background"]
        )
        with self.assertRaises(ValueError):
            replace(cfg, motion_in_background=1).validate()

    def test_actual_c_frame_and_decoder(self):
        path = (
            Path(__file__).resolve().parents[2]
            / "Firmware/ScaraCartesian/build_host/test_motor_io.exe"
        )
        raw = subprocess.check_output([str(path), "--report"])
        status = parse_message(raw)
        self.assertIsInstance(status, NCStatus)
        self.assertEqual(status.pos, (20031, -11, 1819))
        self.assertEqual(status.ready, 7)
        self.assertEqual(status.pol, 3)  # HIGH-positive Z/J1, LOW-positive J2
        decoder = NCDecoder()
        out = []
        for i in range(0, len(raw), 7):
            out += decoder.feed(raw[i : i + 7])
        self.assertEqual(out, [status])
        self.assertEqual(NCDecoder().feed(b"x" * 1600 + b"\n" + raw), [status])
        old = json.loads(raw)
        old["protocol"] = 4
        self.assertIsNone(parse_message(json.dumps(old)))
        bad = json.loads(raw)
        bad["referenced"] = 1
        bad["factor"] = [0, 0, 0]
        self.assertIsNone(parse_message(json.dumps(bad)))

    def test_homing_status_not_referenced_is_valid(self):
        status = replace(fixture(), referenced=0, busy=1, stage=4, phase=(10, 4, 10))
        data = asdict(status)
        data.update(type="status", protocol=5, fw="SCARA_CARTESIAN_NC_V5")
        for k, v in list(data.items()):
            if isinstance(v, tuple):
                data[k] = list(v)
        self.assertIsInstance(parse_message(json.dumps(data)), NCStatus)

    def test_input_wait_and_root_diagnostics(self):
        status = replace(
            fixture(),
            referenced=0,
            busy=1,
            stage=1,
            input_wait=1,
            raw=12,
            switches=12,
            input_axis=1,
            input_kind=1,
            input_bits=12,
            input_at=12345,
            brief=(0, 2, 0),
        )
        data = asdict(status)
        data.update(type="status", protocol=5, fw="SCARA_CARTESIAN_NC_V5")
        for k, v in list(data.items()):
            if isinstance(v, tuple):
                data[k] = list(v)
        parsed = parse_message(json.dumps(data))
        self.assertEqual(parsed.input_axis, 1)
        self.assertEqual(parsed.input_bits, 12)
        data["busy"] = 0
        self.assertIsNone(parse_message(json.dumps(data)))
        data["busy"] = 1
        data["input_axis"] = 3
        self.assertIsNone(parse_message(json.dumps(data)))

    def test_maximum_diagnostics_frame(self):
        path = (
            Path(__file__).resolve().parents[2]
            / "Firmware/ScaraCartesian/build_host/test_motor_io.exe"
        )
        # The harness prints test summaries before its final JSON in this mode.
        raw = subprocess.check_output([str(path), "--max-report"]).splitlines()[-1]
        self.assertLess(len(raw), 1536)
        status = parse_message(raw)
        self.assertIsInstance(status, NCStatus)
        self.assertEqual(status.total, (0xFFFFFFFFFFFFFFFF,) * 3)
        self.assertEqual(status.input_at, 0xFFFFFFFF)

    def test_j2_rates_and_filtered_idle_status(self):
        config = NCConfig().validate()
        self.assertEqual(config.arm_home_pps, 200)
        self.assertEqual((config.j2_home_pps, config.j2_latch_pps), (800, 200))
        old = asdict(config)
        old.pop("j2_home_pps")
        old.pop("j2_latch_pps")
        new = upgrade_r4(old)
        for key in old:
            self.assertEqual(new[key], old[key])
        self.assertEqual((new["j2_home_pps"], new["j2_latch_pps"]), (800, 200))
        status = replace(
            fixture(),
            referenced=0,
            busy=1,
            stage=1,
            input_kind=6,
            input_axis=1,
            input_bits=12,
            idle_ignored=15,
            gate_wait=0,
            false_hits=(0, 0, 0),
        )
        data = asdict(status)
        data.update(type="status", protocol=5, fw="SCARA_CARTESIAN_NC_V5")
        for k, v in list(data.items()):
            if isinstance(v, tuple):
                data[k] = list(v)
        self.assertIsInstance(parse_message(json.dumps(data)), NCStatus)
        data["gate_wait"] = 1
        data["busy"] = 0
        self.assertIsNone(parse_message(json.dumps(data)))
        with self.assertRaises(ValueError):
            replace(config, j2_latch_pps=900).validate()

    def test_passive_arm_observation_and_compensation(self):
        model = NCConfig().validate().model(fixture())
        for sign in (-1, 1):
            p1 = round(sign * 90 * model.factors[1])
            q = model.from_steps((20000, p1, 0))
            self.assertAlmostEqual(q[0], sign * 90, delta=0.08)
            self.assertAlmostEqual(q[0] + q[1], -sign * 30, delta=0.08)
            self.assertAlmostEqual(q[1], -sign * 120, delta=0.08)
            held = model.from_steps((20000, p1, round(sign * 120 * model.factors[2])))
            self.assertAlmostEqual(
                held[1], 0, delta=0.08
            )  # hold relative elbow during J1 calibration
            heading_fixed = model.from_steps(
                (20000, p1, round(sign * 30 * model.factors[2]))
            )
            self.assertAlmostEqual(heading_fixed[0] + heading_fixed[1], 0, delta=0.08)
            self.assertAlmostEqual(
                abs(sign * 120 * model.factors[2] / p1), 4, delta=0.01
            )
        # Migration preserves the separately confirmed direction signs.
        legacy = asdict(NCConfig())
        legacy["coupling"] = 0.666667
        revised = upgrade_r5(legacy)
        self.assertEqual(revised["coupling"], 1.333333)
        for key in legacy:
            if key != "coupling":
                self.assertEqual(revised[key], legacy[key])

    def test_fk_ik_and_measured_quantization(self):
        model = NCConfig().model(fixture())
        for q in ((0, 45, 10), (30, 30, 15), (-50, 70, 5), (60, 15, 20)):
            xyz = model.fk(q)
            back = model.ik(xyz, q)
            for a, b in zip(q, back):
                self.assertAlmostEqual(a, b, places=7)
            roundtrip = model.from_steps(model.to_steps(q))
            for a, b in zip(q, roundtrip):
                self.assertAlmostEqual(a, b, delta=0.08)
        plan = model.plan((35200, 0, 1800), model.fk((30, 30, 20)), 5)
        self.assertGreater(plan.rate, 50)
        self.assertAlmostEqual(plan.joints[0], 30, delta=0.08)
        self.assertAlmostEqual(plan.joints[1], 30, delta=0.08)

    def test_opposite_motor_signs_and_config_upgrade(self):
        config = NCConfig().validate()
        self.assertEqual(config.positive_high, (True, True, False))
        self.assertIn("POL 7 J1 1", config.commands(7))
        self.assertIn("POL 7 J2 0", config.commands(7))
        # Preserve every parameter except the normalized J2 sign. Deterministic
        # migration must also respect a user-inverted J1 and survive rebuilding.
        for j1 in (True, False):
            old = asdict(config)
            old["positive_high"] = [True, j1, j1]
            new = upgrade_r6(old)
            self.assertEqual(new["positive_high"], [True, j1, not j1])
            self.assertEqual(upgrade_r6(new), new)
            for key in old:
                if key != "positive_high":
                    self.assertEqual(new[key], old[key])
        self.assertEqual(fixture().pol, 3)
        demo = DemoLink("DEMO")
        try:
            demo.send("POL 7 J2 1")
            self.assertEqual(demo.pol, 7)
            demo.send("POL 7 J2 0")
            self.assertEqual(demo.pol, 3)
        finally:
            demo.close()

    def test_measured_coupling_and_probe_configuration(self):
        config = NCConfig().validate()
        self.assertIn("COUPLE 7 1 128", config.commands(7))
        status = replace(
            fixture(),
            coupling_ppm=666667,
            beta_q=2097152,
            coupling_ready=1,
            probe_da=128,
            probe_db=256,
        )
        model = config.model(status)
        self.assertAlmostEqual(model.coupling, 0.666667)
        q = model.from_steps(model.to_steps((30, 30, 10)))
        self.assertAlmostEqual(q[1], 30, delta=0.08)
        for cfg in (
            replace(config, coupling_probe_pulses=0),
            replace(config, auto_coupling=1),
        ):
            with self.assertRaises(ValueError):
                cfg.validate()
        old = asdict(config)
        old.pop("auto_coupling")
        old.pop("coupling_probe_pulses")
        old.update(coupling=0.75, positive_high=[True, False, True])
        new = upgrade_r7(old)
        for key in old:
            self.assertEqual(new[key], old[key])
        self.assertTrue(new["auto_coupling"])
        self.assertEqual(new["coupling_probe_pulses"], 128)
        self.assertEqual(upgrade_r7(new), new)

    def test_trace_file_and_rejected_usb_frame(self):
        base = (
            Path(__file__).resolve().parents[2] / "Firmware/ScaraCartesian/build_host"
        )
        with tempfile.TemporaryDirectory(dir=base) as temp:
            self.assertTrue(Path(temp).resolve().is_relative_to(base.resolve()))
            trace = SessionLog(Path(temp) / "logs")
            trace.record("connection", config=asdict(NCConfig()))
            rejected = []
            decoder = NCDecoder(rejected.append)
            self.assertEqual(decoder.feed(b'{"build":"OLD_FIRMWARE"}\n'), [])
            trace.record("rx_rejected", raw=rejected[0].decode())
            trace.record("status", status=asdict(fixture()))
            trace.record("stop_sent", source="USER")
            trace.close()
            self.assertIsNone(trace.error)
            self.assertFalse(trace.thread.is_alive())
            records = [
                json.loads(line)
                for line in trace.path.read_text(encoding="utf-8").splitlines()
            ]
            self.assertEqual(
                [r["type"] for r in records],
                ["connection", "rx_rejected", "status", "stop_sent", "log_end"],
            )
            self.assertEqual(records[2]["status"]["dir_levels"], 0)
            self.assertIn("coupling_ppm", records[2]["status"])
            self.assertEqual(records[0]["config"]["positive_high"], [True, True, False])

    def test_bounds_unreachable_and_singular(self):
        model = NCConfig().model(fixture())
        for xyz in ((0, 300, 10), (0, 196, 10), (*model.fk((0, 45, 30))[:2], 30)):
            with self.assertRaises(ValueError):
                model.plan((35200, 0, 1800), xyz)
        with self.assertRaises(ValueError):
            model.to_steps((0, 89, 10))

    def test_z_span_and_negative_branch(self):
        config = replace(NCConfig(), z_span_mm=80, park_deg=(0, -45))
        status = replace(
            fixture(), factor=(512000, 13653, 40960), pos=(35200, 0, -1800)
        )
        model = config.model(status)
        self.assertAlmostEqual(model.fk(model.from_steps(status.pos))[2], 70.4)
        q = (-20, -40, 60)
        recovered = model.ik(model.fk(q))
        for a, b in zip(q, recovered):
            self.assertAlmostEqual(a, b, places=7)

    def test_bad_configuration(self):
        for config in (
            replace(NCConfig(), coupling=2.1),
            replace(NCConfig(), z_span_mm=float("nan")),
            replace(NCConfig(), park_deg=(0, 0)),
            replace(NCConfig(), microsteps=(16, 3, 8)),
            replace(NCConfig(), arm_home_pps=0),
            replace(NCConfig(), ratios=(1, 100)),
        ):
            with self.assertRaises(ValueError):
                config.validate()
        commands = NCConfig().commands(7)
        self.assertIn("GEOM 7 1333333 1638400 13653 40960 16 8 8", commands)
        self.assertEqual(commands[1], "SPAN 7 -90000 90000 -90000 90000 0")

    def test_z_one_mm_s_and_config_upgrade(self):
        config = NCConfig().validate()
        model = config.model(fixture())
        q = list(model.from_steps((35200, 0, 1800)))
        q[2] -= 10
        plan = model.plan((35200, 0, 1800), model.fk(q), 5)
        self.assertEqual(plan.rate, 1600)
        self.assertEqual(plan.steps[1:], (0, 1800))
        self.assertIn(
            "TUNE 7 200 4800 40000 300000 512 1024 400 800 200", config.commands(7)
        )
        old = asdict(config)
        old.pop("z_latch_pps")
        old.pop("home_in_background")
        old.update(
            z_home_pps=800, max_master_rate=800, max_z_speed_mm_s=0.5, lead_mm_rev=2.0
        )
        new = upgrade_r3(old)
        self.assertEqual(new["z_home_pps"], 1600)
        self.assertEqual(new["max_z_speed_mm_s"], 1.0)
        self.assertEqual(new["z_latch_pps"], 400)
        for key in (
            "l1_mm",
            "l2_mm",
            "ratios",
            "microsteps",
            "positive_high",
            "coupling",
        ):
            self.assertEqual(new[key], old[key])
        base = (
            Path(__file__).resolve().parents[2] / "Firmware/ScaraCartesian/build_host"
        )
        with tempfile.TemporaryDirectory(dir=base) as temp:
            self.assertTrue(Path(temp).resolve().is_relative_to(base.resolve()))
            path = Path(temp) / "config.json"
            path.write_text(json.dumps(old), encoding="utf-8")
            loaded = NCConfig.load(path)
            self.assertTrue(loaded.home_in_background)
            self.assertEqual(
                loaded.z_home_pps, 4800
            )  # old PPS replaced by the requested 3mm/s home setting
        with self.assertRaises(ValueError):
            replace(config, z_latch_pps=5000).validate()


if __name__ == "__main__":
    unittest.main()
