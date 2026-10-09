"""Protocol 5: one coherent six-switch snapshot, no Hall or manual HOME."""

from dataclasses import dataclass
import json
import time
from protocol import SerialLink, LineDecoder

BUILD = "SCARA_CART_NC_HOME_V5_R9"


@dataclass(frozen=True)
class NCStatus:
    session: int
    job: int
    up_ms: int
    busy: int
    referenced: int
    epoch: int
    stage: int
    pos: tuple
    goal: tuple
    total: tuple
    range: tuple
    factor: tuple
    n1: tuple
    n2: tuple
    phase: tuple
    cal_error: tuple
    failed_phase: tuple
    switches: int
    ready: int
    holding: int
    pol: int
    conflicts: int
    tick: int
    ticks: int
    motor_error: int
    fault: int
    timer_late: int
    timer_errno: int
    raw: int
    errors: int
    input_wait: int
    recovered: int
    input_axis: int
    input_kind: int
    input_bits: int
    input_at: int
    brief: tuple
    idle_ignored: int
    gate_wait: int
    gate_axis: int
    false_hits: tuple
    mode: int
    selected: int
    pps: int
    direction: tuple
    dir_levels: int
    coupling_ppm: int
    beta_q: int
    probe_da: int
    probe_db: int
    coupling_ready: int
    reason: str
    build: str
    path: tuple = (0, 0, 0, 32)
    starved: int = 0

    @classmethod
    def parse(cls, v):
        if v.get("fw") != "SCARA_CARTESIAN_NC_V5" or v.get("build") != BUILD:
            return None
        arrays = (
            "pos",
            "goal",
            "total",
            "range",
            "factor",
            "n1",
            "n2",
            "phase",
            "cal_error",
            "failed_phase",
            "brief",
            "false_hits",
            "direction",
            "path",
        )
        for k in cls.__dataclass_fields__:
            if k in arrays:
                if (
                    not isinstance(v.get(k), list)
                    or len(v[k]) != (4 if k == "path" else 3)
                    or any(type(x) is not int for x in v[k])
                ):
                    return None
            elif k in ("reason", "build"):
                if (
                    not isinstance(v.get(k), str)
                    or not v[k]
                    or len(v[k]) > 64
                    or any(
                        not (c.isascii() and (c.isalnum() or c == "_")) for c in v[k]
                    )
                ):
                    return None
            elif type(v.get(k)) is not int:
                return None
        if any(
            v[k] not in (0, 1)
            for k in ("busy", "referenced", "fault", "input_wait", "gate_wait")
        ):
            return None
        if not 0 <= v["stage"] <= 12 or not 0 <= v["switches"] <= 63:
            return None
        if (
            not 0 <= v["mode"] <= 3
            or not 0 <= v["selected"] <= 2
            or not 0 <= v["pps"] <= 6400
        ):
            return None
        if v["pps"] > 1600 and not (
            v["mode"] == 2
            and v["selected"] == 0
            and v["stage"] == 1
            and v["busy"]
            and not v["referenced"]
        ):
            return None
        if (
            any(x not in (-1, 0, 1) for x in v["direction"])
            or not 0 <= v["dir_levels"] <= 7
        ):
            return None
        if not -2000000 <= v["coupling_ppm"] <= 2000000 or abs(v["beta_q"]) > 67108864:
            return None
        if (
            abs(v["probe_da"]) > 512
            or abs(v["probe_db"]) > 8000000
            or v["coupling_ready"] not in (0, 1)
        ):
            return None
        if any(
            not 0 <= v[k] <= 7
            for k in ("ready", "holding", "pol", "conflicts", "errors")
        ) or not (v["holding"] & 1):
            return None
        if any(
            not 0 <= v[k] <= 0xFFFFFFFF
            for k in ("session", "job", "up_ms", "epoch", "timer_late")
        ):
            return None
        if any(not 0 <= v[k] <= 0xFFFFFFFF for k in ("recovered", "input_at")) or any(
            not 0 <= x <= 0xFFFFFFFF for x in v["brief"]
        ):
            return None
        if (
            not -1 <= v["input_axis"] <= 2
            or not 0 <= v["input_kind"] <= 7
            or any(not 0 <= v[k] <= 63 for k in ("raw", "input_bits"))
        ):
            return None
        if (
            not 0 <= v["gate_axis"] <= 2
            or not 0 <= v["idle_ignored"] <= 0xFFFFFFFF
            or any(not 0 <= x <= 0xFFFFFFFF for x in v["false_hits"])
        ):
            return None
        if v["gate_wait"] and (not v["busy"] or v["referenced"]):
            return None
        if v["input_wait"] and (
            not v["busy"] or v["referenced"] or v["stage"] in (0, 8, 9)
        ):
            return None
        if any(abs(x) > 8000000 for k in ("pos", "goal") for x in v[k]):
            return None
        if any(not 0 <= x <= 0xFFFFFFFFFFFFFFFF for x in v["total"]):
            return None
        if any(not 0 <= x <= 1000000 for k in ("range", "n1", "n2") for x in v[k]):
            return None
        if any(not 0 <= x <= 100000000 for x in v["factor"]):
            return None
        if any(
            not 0 <= x <= 11 for k in ("phase", "failed_phase") for x in v[k]
        ) or any(not 0 <= x <= 9 for x in v["cal_error"]):
            return None
        if (
            not 0 <= v["motor_error"] <= 5
            or not 0 <= v["tick"] <= v["ticks"] <= 1000000
        ):
            return None
        total, received, done, free = v["path"]
        if (
            not 0 <= done <= received <= total <= 500
            or not 0 <= free <= 32
            or not 0 <= v["starved"] <= 0xFFFFFFFF
        ):
            return None
        if v["referenced"] and (
            v["stage"] != 8
            or v["fault"]
            or v["conflicts"]
            or v["errors"]
            or v["holding"] != 7
            or any(x < 64 for x in v["range"])
            or any(x <= 0 for x in v["factor"])
        ):
            return None
        if v["busy"] and (not v["session"] or v["fault"] or v["holding"] != 7):
            return None
        return cls(
            **{
                k: tuple(v[k]) if k in arrays else v[k]
                for k in cls.__dataclass_fields__
            }
        )


@dataclass(frozen=True)
class Ack:
    session: int
    job: int
    op: str
    ok: int
    reason: str


def parse_message(raw):
    try:
        v = json.loads(raw)
        if (
            not isinstance(v, dict)
            or type(v.get("protocol")) is not int
            or v["protocol"] != 5
        ):
            return None
        if v.get("type") == "status":
            return NCStatus.parse(v)
        if v.get("type") == "ack":
            if any(type(v.get(k)) is not int for k in ("session", "job", "ok")) or v[
                "ok"
            ] not in (0, 1):
                return None
            if any(not 0 <= v[k] <= 0xFFFFFFFF for k in ("session", "job")):
                return None
            if any(
                not isinstance(v.get(k), str)
                or not v[k]
                or len(v[k]) > 64
                or any(not (c.isascii() and (c.isalnum() or c == "_")) for c in v[k])
                for k in ("op", "reason")
            ):
                return None
            return Ack(**{k: v[k] for k in Ack.__dataclass_fields__})
    except (ValueError, TypeError, KeyError, UnicodeError):
        pass
    return None


class NCDecoder(LineDecoder):
    def __init__(self, on_rejected=None):
        super().__init__(
            parser=parse_message,
            max_frame_bytes=1536,
            on_rejected=on_rejected,
            overflow_marker=b"frame_larger_than_1536_bytes",
        )


class NCLink(SerialLink):
    def _make_decoder(self):
        return NCDecoder(
            lambda raw: self._event(
                ("rx_rejected", time.monotonic(), raw.decode("utf-8", errors="replace"))
            )
        )

    def _message_kind(self, message):
        return "status" if isinstance(message, NCStatus) else "ack"
