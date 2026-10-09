"""Headless acceptance tests for NC session, cancellation and motion gates."""

from dataclasses import asdict, replace
import json
from pathlib import Path
import queue
import sys
import threading
import time
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from cartesian_nc_connection import ConnectionController
from cartesian_nc_geometry import segment_distance, segment_distance_squared
from cartesian_nc_model import NCConfig
from cartesian_nc_motion import MotionController
from cartesian_nc_planning import PlanningController, PlanningResult
from cartesian_nc_policy import (
    background_allowed,
    hardware_ready,
    status_expired,
    status_fresh,
)
from cartesian_nc_protocol import Ack, BUILD, NCDecoder, NCLink
from cartesian_nc_safety import SafetyController
from protocol import LineDecoder
from support import fixture


class Value:
    def __init__(self, value=None):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class Runtime(
    ConnectionController, MotionController, PlanningController, SafetyController
):
    """Use production controllers with a recording link and no Tk interpreter."""

    def __init__(self, state, directory):
        self.sent = []
        self.link = SimpleNamespace(
            session=state.session,
            events=queue.Queue(),
            send=self.sent.append,
            close=lambda: None,
        )
        self.state = state
        self.rx = self.opened = time.monotonic()
        self.config = NCConfig()
        self.config_error = None
        self.model = self.config.model(state)
        self.local_reference = True
        self.plan_token = 0
        self.plan_results = queue.Queue()
        self.planning = False
        self.job = state.job
        self.active_job = self.pending = self.expect_epoch = self.task_kind = None
        self.commands = []
        self.last_keep = self.started = 0
        self.trace = None
        self.demo = True
        self.event_log_path = directory / "events.jsonl"
        self.message = Value()
        self.log_text = Value()
        self.bk_note = Value()
        self.follow_enabled = Value(False)
        self.follow_target = None
        self.follow_revision = 0
        self.program_points = ()
        self.program_index = self.program_wait = 0
        self.path_upload = self.path_run = None
        self.active_purpose = self.preview = None
        self.bk_active = self.bk_pending = False
        self._foreground = lambda: True

    def render(self):
        pass

    def sync_editor(self):
        pass

    def record_bk(self, *args):
        pass

    def set_target_pose(self, pose):
        self.target_pose = pose


class RuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.state = fixture()

    def setUp(self):
        self.directory = Path(__file__).resolve().parents[1] / "build/refactor/runtime"
        self.app = Runtime(self.state, self.directory)

    def result(self, **changes):
        plan = SimpleNamespace(steps=(35200, 1, 1800), rate=200, joints=(1, 45, 22))
        fields = dict(
            token=self.app.plan_token,
            epoch=self.app.state.epoch,
            start=self.app.state.pos,
            model=self.app.model,
            moving=True,
            result=plan,
            purpose="manual",
            revision=0,
        )
        fields.update(changes)
        return PlanningResult(**fields)

    def test_snapshot_worker_does_not_send_or_mutate_reference(self):
        original = self.app.model
        worker = self.app.submit_planning(
            lambda model, start, cancelled: (model, start, cancelled()),
            purpose="drawing_fit",
        )
        worker.join(1)
        self.assertFalse(worker.is_alive())
        result = self.app.plan_results.get_nowait()
        self.assertEqual(
            len(result), 8
        )  # Existing tuple queue consumers remain compatible.
        self.assertEqual(result.result, (original, self.state.pos, False))
        self.assertEqual(result.epoch, self.state.epoch)
        self.assertEqual(self.app.sent, [])
        self.assertTrue(self.app.planning)
        self.assertIs(self.app.model, original)

    def test_old_worker_cannot_clear_or_send_newer_request(self):
        release = threading.Event()
        started = threading.Event()

        def calculate(model, start, cancelled):
            started.set()
            release.wait(1)
            return cancelled()

        worker = self.app.submit_planning(calculate, purpose="manual", moving=True)
        self.assertTrue(started.wait(1))
        self.app.cancel_planning()
        self.app.plan_token += 1
        self.app.planning = True  # Represents the next calculation still running.
        release.set()
        worker.join(1)
        self.app._poll_planning(time.monotonic())
        self.assertTrue(self.app.planning)
        self.assertEqual(self.app.sent, [])

    def test_changed_reference_or_start_rejects_completed_plan(self):
        variants = (
            dict(epoch=self.state.epoch + 1),
            dict(start=(35201, 0, 1800)),
            dict(model=object()),
        )
        for changes in variants:
            with self.subTest(changes=changes):
                self.app.planning = True
                self.app.plan_results.put(self.result(**changes))
                self.app._poll_planning(time.monotonic())
                self.assertEqual(self.app.sent, [])
                self.assertIn("đã đổi", self.app.message.get())
        self.app.local_reference = False
        self.app.plan_results.put(self.result())
        self.app._poll_planning(time.monotonic())
        self.assertEqual(self.app.sent, [])

    def test_valid_result_sends_exact_checked_steps_and_job(self):
        self.app.planning = True
        self.app.plan_results.put(self.result())
        self.app._poll_planning(time.monotonic())
        self.assertEqual(
            self.app.sent,
            [f"MOVE {self.state.session} {self.state.job + 1} 35200 1 1800 200"],
        )
        self.assertEqual(self.app.pending[0], "MOVE")
        self.assertEqual(self.app.active_job, self.state.job + 1)
        self.assertFalse(self.app.planning)

    def test_follow_revision_and_strict_focus_gate_move(self):
        self.app.follow_enabled.set(True)
        self.app.follow_revision = 2
        self.app.plan_results.put(self.result(purpose="follow", revision=1))
        self.app._poll_planning(time.monotonic())
        self.assertEqual(self.app.sent, [])
        self.app.config = replace(self.app.config, motion_in_background=False)
        self.app._foreground = lambda: False
        self.app.plan_results.put(self.result())
        self.app._poll_planning(time.monotonic())
        self.assertEqual(self.app.sent, ["STOP FOCUS"])

    def test_failed_drawing_worker_clears_pending_and_reports_error(self):
        self.app.bk_pending = True
        self.app.plan_results.put(
            self.result(purpose="drawing_check", result=ValueError("bad paper"))
        )
        self.app._poll_planning(time.monotonic())
        self.assertFalse(self.app.bk_pending)
        self.assertEqual(self.app.bk_note.get(), "bad paper")
        self.assertEqual(self.app.sent, [])

    def test_foreign_session_ack_does_not_advance_sequence(self):
        self.app.pending = ("POL", time.monotonic())
        self.app.commands = ["HOLD 7 J1 1"]
        self.app._handle_ack(Ack(9, 0, "POL", 1, "ok"), 1)
        self.assertEqual(self.app.pending[0], "POL")
        self.assertEqual(self.app.sent, [])
        self.app._handle_ack(Ack(7, 0, "POL", 1, "ok"), 2)
        self.assertEqual(self.app.sent, ["HOLD 7 J1 1"])

    def test_rejected_ack_aborts_sequence_and_invalidates_reference(self):
        self.app.pending = ("MOVE", time.monotonic())
        self.app.commands = ["MOVE 7 2 0 0 0 200"]
        self.app.active_job = 1
        self.app._handle_ack(Ack(7, 1, "MOVE", 0, "soft_limit"), 1)
        self.assertFalse(self.app.commands)
        self.assertIsNone(self.app.active_job)
        self.assertFalse(self.app.local_reference)
        self.assertEqual(self.app.sent, [])

    def test_status_and_ack_timeouts_precede_keepalive(self):
        self.app.active_job = 1
        self.app.rx = time.monotonic() - 1
        self.app._maintain_connection(time.monotonic())
        self.assertIsNone(self.app.link)
        self.assertEqual(self.app.sent, ["STOP STALE"])
        app = Runtime(self.state, self.directory)
        app.active_job = 1
        app.pending = ("MOVE", time.monotonic() - 2)
        app._maintain_connection(time.monotonic())
        self.assertEqual(app.sent, ["STOP ACK"])
        self.assertFalse(app.local_reference)

    def test_usb_failure_while_advancing_ack_stops_event_drain(self):
        def fail(line):
            raise OSError("unplugged")

        self.app.link.send = fail
        self.app.pending = ("POL", time.monotonic())
        self.app.commands = ["HOLD 7 J1 1"]
        self.app.link.events.put(("ack", time.monotonic(), Ack(7, 0, "POL", 1, "ok")))
        self.app._drain_events(time.monotonic())
        self.assertIsNone(self.app.link)
        self.assertFalse(self.app.local_reference)
        self.assertIn("unplugged", self.app.message.get())

    def test_status_reference_is_accepted_only_for_expected_session_epoch(self):
        self.app.local_reference = False
        self.app.model = None
        self.app.expect_epoch = self.state.epoch + 1
        foreign = replace(self.state, session=8, epoch=self.app.expect_epoch)
        self.app._handle_status(foreign, time.monotonic(), time.monotonic())
        self.assertIsNone(self.app.model)
        self.app._handle_status(self.state, time.monotonic(), time.monotonic())
        self.assertFalse(self.app.local_reference)
        expected = replace(self.state, epoch=self.app.expect_epoch)
        self.app._handle_status(expected, time.monotonic(), time.monotonic())
        self.assertTrue(self.app.local_reference)
        self.assertIsNotNone(self.app.model)


class PolicyAndFramingTests(unittest.TestCase):
    def test_status_age_and_hardware_gates(self):
        state = fixture()
        link = SimpleNamespace(session=state.session)
        self.assertTrue(status_fresh(link, state, 0, 0.79))
        self.assertFalse(status_fresh(link, state, 0, 0.8))
        self.assertFalse(status_fresh(SimpleNamespace(session=8), state, 0, 0))
        self.assertFalse(status_fresh(None, state, 0, 0))
        self.assertFalse(status_expired(None, 0, 0, 3))
        self.assertTrue(status_expired(None, 0, 0, 3.01))
        self.assertFalse(status_expired(state, 0, 0, 0.8))
        self.assertTrue(status_expired(state, 0, 0, 0.81))
        self.assertTrue(hardware_ready(state))
        for changes in (
            dict(fault=1),
            dict(ready=6),
            dict(conflicts=1),
            dict(errors=4),
        ):
            self.assertFalse(hardware_ready(replace(state, **changes)))
        self.assertFalse(hardware_ready(state, "bad config"))
        self.assertFalse(hardware_ready(None))
        config = replace(
            NCConfig(), home_in_background=False, motion_in_background=True
        )
        self.assertFalse(background_allowed("INIT", config))
        self.assertTrue(background_allowed("MOVE", config))

    def test_shared_framer_accepts_boundary_and_recovers_after_overflow(self):
        rejected = []
        decoder = LineDecoder(
            parser=lambda raw: raw,
            max_frame_bytes=4,
            on_rejected=rejected.append,
            overflow_marker=b"too_long",
        )
        self.assertEqual(decoder.feed(b"ab"), [])
        self.assertEqual(decoder.feed(b"cd\nabcde\nxy\n"), [b"abcd", b"xy"])
        self.assertEqual(rejected, [b"too_long"])
        self.assertEqual(decoder.buffer, bytearray())
        self.assertFalse(decoder.discard)

    def test_nc_framer_keeps_ack_contract_and_rejection_diagnostics(self):
        raw = json.dumps(
            dict(type="ack", protocol=5, session=7, job=1, op="MOVE", ok=1, reason="ok")
        ).encode()
        padded = raw + b" " * (1536 - len(raw))
        rejected = []
        decoder = NCDecoder(rejected.append)
        out = []
        for byte in padded + b"\n":
            out += decoder.feed(bytes([byte]))
        self.assertEqual(out, [Ack(7, 1, "MOVE", 1, "ok")])
        self.assertEqual(decoder.feed(b"x" * 1537 + b"\n{}\n" + raw + b"\r\n"), out)
        self.assertEqual(rejected, [b"frame_larger_than_1536_bytes", b"{}"])
        self.assertEqual(NCLink._message_kind(None, out[0]), "ack")
        self.assertEqual(NCLink._message_kind(None, fixture()), "status")

    def test_text_and_shape_segment_distances_agree_at_endpoints_and_degenerate_edges(
        self,
    ):
        for start, end in (((0, 0), (3, 4)), ((2, 3), (2, 3)), ((-5, 1), (5, 1))):
            for point in (start, end, (0, 0), (-10, 8), (10, -8)):
                self.assertAlmostEqual(
                    segment_distance(point, start, end) ** 2,
                    segment_distance_squared(point, start, end),
                )


class FakeSerial:
    """A blocking byte stream for the production reader; never opens a COM port."""

    def __init__(self, **options):
        self.options = options
        self.input = queue.Queue()
        self.writes = []
        self.is_closed = False
        self.short_write = False
        self.reset_count = 0

    def reset_input_buffer(self):
        self.reset_count += 1

    def reset_output_buffer(self):
        self.reset_count += 1

    def read(self, size):
        try:
            data = self.input.get(timeout=0.01)
        except queue.Empty:
            return b""
        if isinstance(data, Exception):
            raise data
        return data

    def write(self, payload):
        self.writes.append(payload)
        return len(payload) - int(self.short_write)

    def close(self):
        self.is_closed = True


class SerialTransportTests(unittest.TestCase):
    def setUp(self):
        self.serial = FakeSerial()
        self.link = NCLink("FAKE", serial_factory=lambda **options: self.serial)
        self.addCleanup(self.link.close)

    def test_shared_reader_tags_status_ack_and_rejected_frames(self):
        self.assertTrue(self.serial.dtr)
        self.assertEqual(self.serial.reset_count, 2)
        self.assertEqual(
            self.serial.writes, [b"STOP\n", f"HELLO {self.link.session}\n".encode()]
        )
        state = replace(fixture(), session=self.link.session)
        status = {
            **asdict(state),
            "type": "status",
            "protocol": 5,
            "fw": "SCARA_CARTESIAN_NC_V5",
            "build": BUILD,
        }
        ack = dict(
            type="ack",
            protocol=5,
            session=self.link.session,
            job=1,
            op="MOVE",
            ok=1,
            reason="ok",
        )
        self.serial.input.put(
            (json.dumps(status) + "\n" + json.dumps(ack) + "\n{}\n").encode()
        )
        events = [self.link.events.get(timeout=1) for _ in range(3)]
        messages = {event[0]: event[2] for event in events}
        self.assertEqual(messages["status"], state)
        self.assertEqual(messages["ack"], Ack(self.link.session, 1, "MOVE", 1, "ok"))
        self.assertEqual(messages["rx_rejected"], "{}")

    def test_short_write_and_closed_link_raise_without_hiding_failure(self):
        self.serial.short_write = True
        with self.assertRaisesRegex(OSError, "đầy đủ"):
            self.link.send("STATUS")
        self.serial.short_write = False
        self.link.close()
        self.assertEqual(self.serial.writes[-1], b"STOP\n")
        self.assertTrue(self.serial.is_closed)
        self.assertFalse(self.link.thread.is_alive())
        with self.assertRaisesRegex(OSError, "đã đóng"):
            self.link.send("STATUS")

    def test_reader_reports_disconnect_and_close_is_idempotent(self):
        self.serial.input.put(OSError("unplugged"))
        self.assertEqual(self.link.events.get(timeout=1), ("error", "unplugged"))
        self.link.close()
        count = len(self.serial.writes)
        self.link.close()
        self.assertEqual(len(self.serial.writes), count)


if __name__ == "__main__":
    unittest.main()
