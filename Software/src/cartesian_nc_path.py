"""Offline stroke lookahead and bounded, acknowledged firmware FIFO uploads."""

from dataclasses import dataclass
import math

CAPACITY = 32
ACCEL_MM_S2 = 20.0
JUNCTION_MM = 0.03


def ramp_ticks(peak, end, accel):
    return (15 * (peak * peak - end * end) + 16 * accel - 1) // (16 * accel)


def smooth_square(low, high, tick, length):
    if not length or tick >= length:
        return high * high
    u = tick * 1048576 // length

    def blend(a, b):
        return a + ((b - a) * u >> 20)

    p = blend(0, u)
    q = blend(u, 1048576)
    a = blend(0, p)
    b = blend(p, q)
    d = blend(q, 1048576)
    s = blend(blend(a, b), blend(b, d))
    return low * low + ((high * high - low * low) * s >> 20)


@dataclass(frozen=True)
class PathSegment:
    steps: tuple
    peak: int
    entry: int
    exit: int
    accel: int
    ticks: int

    @property
    def ramp_up(self):
        return ramp_ticks(self.peak, self.entry, self.accel)

    @property
    def ramp_down(self):
        return ramp_ticks(self.peak, self.exit, self.accel)

    def command(self, session, job, index):
        return (
            f"SEG {session} {job} {index} "
            + " ".join(map(str, self.steps))
            + f" {self.peak} {self.entry} {self.exit} {self.accel}"
        )

    def seconds(self):
        up = self.ramp_up
        down = self.ramp_down
        return sum(
            1
            / max(
                50,
                math.isqrt(
                    min(
                        smooth_square(self.entry, self.peak, t, up),
                        smooth_square(self.exit, self.peak, self.ticks - t, down),
                    )
                ),
            )
            for t in range(self.ticks)
        )


@dataclass(frozen=True)
class StrokeRun:
    first: int
    end: int
    start_steps: tuple
    segments: tuple
    seconds: float


def lookahead(plans, starts, points):
    """No corner cutting: same validated DDA positions, only retime the edges.
    Speeds are capped by the existing measured-joint/Cartesian planner.
    A forward/backward pass makes each junction reachable under acceleration.
    """
    lengths = []
    vectors = []
    ticks = []
    densities = []
    caps = []
    for plan, start, (a, b) in zip(plans, starts, points):
        length = math.dist(a, b)
        n = max(abs(x - y) for x, y in zip(plan.steps, start))
        if n == 0:
            continue
        if length <= 0:
            raise ValueError("Nét có đoạn không có chiều dài XY.")
        lengths.append(length)
        vectors.append(tuple((y - x) / length for x, y in zip(a, b)))
        ticks.append(n)
        densities.append(n / length)
        caps.append(plan.rate * length / n)
    if len(ticks) != len(plans):
        raise ValueError("Đoạn nét trùng xung; giảm mật độ điểm hoặc tăng cỡ chữ.")
    count = len(plans)
    velocity = [0.0] * (count + 1)
    for i in range(1, count):
        cosine = max(-1, min(1, sum(a * b for a, b in zip(vectors[i - 1], vectors[i]))))
        if cosine < math.cos(math.radians(35)):
            continue  # sharp vertex: minimum rate
        half = math.sqrt((1 + cosine) / 2)
        junction = (
            math.inf
            if 1 - half < 1e-9
            else math.sqrt(ACCEL_MM_S2 * JUNCTION_MM * half / (1 - half))
        )
        velocity[i] = min(caps[i - 1], caps[i], junction)
    # Quintic v^2(distance) has zero first/second derivative at each end.
    # 16/15 replaces 2 so the peak acceleration remains <= ACCEL_MM_S2.
    for i in range(count):
        velocity[i + 1] = min(
            velocity[i + 1],
            math.sqrt(velocity[i] ** 2 + (16 / 15) * ACCEL_MM_S2 * lengths[i]),
        )
    for i in reversed(range(count)):
        velocity[i] = min(
            velocity[i],
            math.sqrt(velocity[i + 1] ** 2 + (16 / 15) * ACCEL_MM_S2 * lengths[i]),
        )
    segments = []
    for i, plan in enumerate(plans):
        peak = plan.rate
        entry = max(50, min(peak, math.floor(velocity[i] * densities[i])))
        exit = max(50, min(peak, math.floor(velocity[i + 1] * densities[i])))
        accel = max(50, min(200000, math.ceil(ACCEL_MM_S2 * densities[i])))
        if entry > exit:
            entry = min(entry, math.isqrt(exit * exit + 16 * accel * ticks[i] // 15))
        else:
            exit = min(exit, math.isqrt(entry * entry + 16 * accel * ticks[i] // 15))
        lo = max(entry, exit)
        hi = peak
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if ramp_ticks(mid, entry, accel) + ramp_ticks(mid, exit, accel) <= ticks[i]:
                lo = mid
            else:
                hi = mid - 1
        peak = lo
        segments.append(PathSegment(plan.steps, peak, entry, exit, accel, ticks[i]))
    return tuple(segments)


class PathUpload:
    """One outstanding SEG, credits from status minus any later sent packets.
    Upload 32 segments before GO; continue refilling below the high water mark.
    Queue starvation is a firmware fault, never a silent pause/resume.
    """

    def __init__(self, session, job, segments):
        self.session = session
        self.job = job
        self.segments = segments
        self.sent = 0
        self.acked = 0
        self.pending = None
        self.go_sent = False
        self.started = False
        self.free = CAPACITY
        self.status_received = 0

    def status(self, state):
        if state.job != self.job:
            return
        self.status_received = state.path[1]
        self.free = state.path[3]

    def ack(self, ack):
        if ack.session != self.session or ack.job != self.job or ack.op != self.pending:
            return False
        self.pending = None
        if not ack.ok:
            raise ValueError(ack.reason)
        if ack.op == "SEG":
            self.acked += 1
        if ack.op == "GO":
            self.started = True
        return True

    def next(self):
        if self.pending:
            return None
        if not self.go_sent and self.acked >= min(CAPACITY, len(self.segments)):
            self.pending = "GO"
            self.go_sent = True
            return f"GO {self.session} {self.job}"
        credits = self.free - max(0, self.sent - self.status_received)
        if self.sent < len(self.segments) and credits > 0:
            line = self.segments[self.sent].command(self.session, self.job, self.sent)
            self.sent += 1
            self.pending = "SEG"
            return line
        return None
