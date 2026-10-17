"""
================================================================================
FILE: cartesian_nc_drawing.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Drawing Studio compiler (`compile_bk`), stroke ordering/orientation optimizer
     (`optimize_strokes`), exact DDA step-by-step ink-deviation verifier (`dda_xyz`),
     and automatic bounding-size fitter (`fit_drawing`).
     Compiles shapes, stroke-font words (`BK`, `LINH`, `THOA`), and imported
     CAD/CAM vectors into verified `DrawingProgram` objects containing both
     point-to-point operations (lifts, travels, lowers) and continuous lookahead
     `StrokeRun` FIFO segments.

[VI] MỤC ĐÍCH:
     Trình biên dịch bản vẽ (`compile_bk`), bộ tối ưu hóa thứ tự và chiều nét vẽ
     (`optimize_strokes`), bộ kiểm tra sai lệch từng xung DDA (`dda_xyz`), và
     công cụ tự động tìm kích thước lớn nhất vừa vùng làm việc (`fit_drawing`).
     Biên dịch các hình cơ bản, chữ viết (`BK`, `LINH`, `THOA`) và file vector
     nhập ngoài thành đối tượng `DrawingProgram` đã kiểm duyệt an toàn, bao gồm
     cả các lệnh nâng/chuyển/hạ bút và các chuỗi nét vẽ liên tục `StrokeRun`.

[DEBUG TIP / MẸO GỠ LỖI]:
     - In `compile_bk()`, the very first operation is ALWAYS a pure vertical
       lift `append((*start[:2], up), "lift", 0)` at the robot's current XY
       before any lateral travel. Never remove this initial lift—it prevents
       the pen from dragging across the paper from its starting position!
     - Trong hàm `compile_bk()`, thao tác đầu tiên LUÔN LUÔN là nhấc bút thẳng đứng
       `append((*start[:2], up), "lift", 0)` tại đúng tọa độ XY hiện tại trước khi
       di chuyển ngang. Tuyệt đối không xóa bước này để tránh làm quệt bút lên giấy!
================================================================================
"""

from dataclasses import dataclass, replace
from functools import lru_cache
import math
from cartesian_nc_workspace import Setpoint
from cartesian_nc_font import word_geometry
from cartesian_nc_path import lookahead, StrokeRun
from cartesian_nc_curves import ellipse_arc, ellipse_radial_error
from cartesian_nc_shapes import (
    PATTERNS,
    SHAPES,
    shape_dimensions,
    shape_vertices,
    shape_ink_distance,
)
from cartesian_nc_geometry import segment_distance
from cartesian_nc_vector import (
    CUSTOM_PATTERNS,
    custom_dimensions,
    custom_strokes,
    custom_ink_distance,
    load_vector_strokes,
    register_custom_pattern,
)


# ==============================================================================
# 1. DRAWING SETTINGS & VERTEX MODEL / CẤU HÌNH BẢN VẼ & MÔ HÌNH ĐỈNH NÉT VẼ
# ==============================================================================


@dataclass(frozen=True)
class BKSettings:
    """
    [EN] Immutable configuration parameters for a Drawing Studio job.
         - `paper_z`: confirmed Z height (mm) where the pen tip touches the paper.
         - `lift_mm`: height (mm) above `paper_z` used for rapid pen-up travels.
         - `step_mm`: maximum Cartesian chord length (mm) when subdividing curves.
         - `tolerance_mm`: maximum allowed deviation (mm) between every emitted
           stepper pulse's FK position and the ideal mathematical curve.
    [VI] Bộ thông số cấu hình bất biến cho một tác vụ vẽ trong Drawing Studio.
         - `paper_z`: cao độ Z (mm) đã xác nhận tại mặt giấy.
         - `lift_mm`: độ nâng bút (mm) tính từ mặt giấy `paper_z` khi di chuyển không vẽ.
         - `step_mm`: bước chia đoạn thẳng tối đa (mm) khi xấp xỉ đường cong.
         - `tolerance_mm`: sai lệch tối đa cho phép (mm) giữa vị trí động học thuận
           của từng xung bước DDA so với đường hình học lý tưởng.
    """

    size_mm: float = 20.0
    center_x: float = 0.0
    center_y: float = 165.0
    paper_z: float | None = None
    lift_mm: float = 2.0
    draw_speed: float = 12.0
    travel_speed: float = 12.0
    step_mm: float = 0.75
    tolerance_mm: float = 0.25
    pattern: str = "BK"
    settle_s: float = 0.12
    paper_tilt: tuple = (0.0, 0.0)

    def paper_z_at(self, x_mm, y_mm):
        """
        [EN] Evaluate the tilted paper Z height (in mm) at `(x_mm, y_mm)` relative
             to `(self.center_x, self.center_y)`:
                 Z(x, y) = paper_z + kx * (x - center_x) + ky * (y - center_y)
             When `paper_tilt == (0.0, 0.0)`, returns `self.paper_z` unchanged.
        [VI] Tính cao độ mặt giấy nghiêng Z (mm) tại `(x_mm, y_mm)` so với tâm
             `(self.center_x, self.center_y)`:
                 Z(x, y) = paper_z + kx * (x - center_x) + ky * (y - center_y)
             Khi `paper_tilt == (0.0, 0.0)`, trả về đúng `self.paper_z`.
        """
        if self.paper_z is None:
            return None
        kx, ky = self.paper_tilt
        if kx == 0.0 and ky == 0.0:
            return self.paper_z
        dx = float(x_mm) - self.center_x
        dy = float(y_mm) - self.center_y
        return self.paper_z + kx * dx + ky * dy

    def validate(self, paper=False):
        """
        [EN] Validate all drawing settings. If `paper=True`, requires `paper_z`
             to be explicitly set (not `None`). Returns `self`.
        [VI] Kiểm tra tính hợp lệ của toàn bộ thông số vẽ. Nếu `paper=True`, bắt buộc
             phải có giá trị `paper_z` (khác `None`). Trả về `self`.
        """
        # [EN] Step 1: Verify the pattern code exists in built-in or custom catalogs.
        # [VI] Bước 1: Kiểm tra mã mẫu vẽ có tồn tại trong thư viện có sẵn hoặc tùy chỉnh.
        if self.pattern not in PATTERNS and self.pattern not in CUSTOM_PATTERNS:
            raise ValueError("Chọn một mẫu trong thư viện hình/chữ.")

        # [EN] Step 2: Ensure all numeric parameters are finite int/float numbers.
        # [VI] Bước 2: Đảm bảo mọi thông số số học đều là số thực hữu hạn.
        values = (
            self.size_mm,
            self.center_x,
            self.center_y,
            self.lift_mm,
            self.draw_speed,
            self.travel_speed,
            self.step_mm,
            self.tolerance_mm,
            self.settle_s,
        )
        for v in values:
            if type(v) not in (int, float) or not math.isfinite(v):
                raise ValueError("Thông số BK cần số hữu hạn.")

        # [EN] Step 3: Validate 2D paper tilt slopes (kx, ky) in [-0.25, 0.25].
        # [VI] Bước 3: Kiểm tra độ nghiêng mặt giấy (kx, ky) nằm trong [-0.25, 0.25].
        tilt_valid = (
            isinstance(self.paper_tilt, (tuple, list)) and len(self.paper_tilt) == 2
        )
        if tilt_valid:
            for v in self.paper_tilt:
                if type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 0.25:
                    tilt_valid = False
                    break
        if not tilt_valid:
            raise ValueError("Độ nghiêng mặt giấy kx, ky cần trong khoảng -0,25 đến 0,25.")

        object.__setattr__(
            self, "paper_tilt", (float(self.paper_tilt[0]), float(self.paper_tilt[1]))
        )

        # [EN] Step 4: Check physical bounds for size, center, lift, speeds, step, tolerance, settle.
        # [VI] Bước 4: Kiểm tra giới hạn vật lý của kích thước, tâm, độ nhấc, tốc độ, bước chia, dung sai.
        if not 4 <= self.size_mm <= 180 or abs(self.center_x) > 1000 or abs(self.center_y) > 1000:
            raise ValueError("Chiều rộng cần4–180mm; tâm XY ngoài phạm vi.")
        if (
            not 0.25 <= self.lift_mm <= 10
            or not 0.05 <= self.draw_speed <= 50
            or not 0.05 <= self.travel_speed <= 50
        ):
            raise ValueError("Nhấc bút cần0,25–10mm; tốc độ cần0,05–50mm/s.")
        if not 0.2 <= self.step_mm <= 2 or not 0.05 <= self.tolerance_mm <= 1:
            raise ValueError("Bước nét cần0,2–2mm; sai lệch mô hình cần0,05–1mm.")
        if not 0 <= self.settle_s <= 1:
            raise ValueError("Thời gian chờ bút cần 0–1 giây.")

        # [EN] Step 5: Validate paper_z finiteness and presence (when paper=True).
        # [VI] Bước 5: Kiểm tra cao độ Z mặt giấy hữu hạn và bắt buộc xác nhận khi paper=True.
        if self.paper_z is not None and (
            type(self.paper_z) not in (int, float) or not math.isfinite(self.paper_z)
        ):
            raise ValueError("Z giấy cần số hữu hạn.")
        if paper and self.paper_z is None:
            raise ValueError("Cần xác nhận Z đầu bút vừa chạm giấy.")
        return self


@dataclass(frozen=True)
class Vertex:
    """
    [EN] A 2D point `xy = (x, y)` tagged with the identifier `part` of the
         geometric segment that ends at this vertex.
    [VI] Một điểm 2D `xy = (x, y)` gắn nhãn `part` định danh phần tử hình học
         kết thúc tại đỉnh này.
    """

    xy: tuple
    part: str


# ==============================================================================
# 2. TSP STROKE ORDERING & ORIENTATION DP / TỐI ƯU HÓA THỨ TỰ & CHIỀU NÉT VẼ
# ==============================================================================


def reverse_stroke(stroke):
    """
    [EN] Reverse a stroke's vertex sequence while shifting `part` labels so each
         directed edge `(stroke[k-1].xy -> stroke[k].xy)` retains its original
         geometric `part` identifier for ink-distance verification.
         Why shift `part`?
         In the forward stroke `[V0, V1, ..., V_m]`, the segment between `V_i`
         and `V_{i+1}` is stored on `V_{i+1}.part`. When traversing backward from
         `V_{i+1}` to `V_i`, that exact same physical segment now ends at `V_i`,
         so the reversed vertex at `stroke[i].xy` must carry `stroke[i + 1].part`!

    [VI] Đảo ngược thứ tự đỉnh của một nét vẽ đồng thời dịch chuyển nhãn `part`
         sao cho mỗi cạnh nối `(stroke[k-1].xy -> stroke[k].xy)` vẫn giữ đúng
         mã phần tử hình học gốc để kiểm tra khoảng cách sai lệch mực.
         Tại sao phải dịch nhãn `part`?
         Ở chiều thuận `[V0, V1, ..., V_m]`, đoạn nối giữa `V_i` và `V_{i+1}`
         mang nhãn `V_{i+1}.part`. Khi vẽ ngược từ `V_{i+1}` về `V_i`, đoạn thẳng
         đó kết thúc tại `V_i`, nên đỉnh đảo tại `stroke[i].xy` phải nhận nhãn
         `stroke[i + 1].part`!
    """
    if len(stroke) <= 1:
        return tuple(stroke)

    reversed_vertices = [Vertex(stroke[-1].xy, stroke[-1].part)]
    for i in range(len(stroke) - 2, -1, -1):
        shifted_part = stroke[i + 1].part
        reversed_vertices.append(Vertex(stroke[i].xy, shifted_part))
    return tuple(reversed_vertices)


def pen_up_travel_distance(strokes, start_xy=None):
    """
    [EN] Compute total XY pen-up rapid travel distance across `strokes` from `start_xy`.
    [VI] Tính tổng quãng đường di chuyển nhấc bút trên mặt phẳng XY qua tất cả các nét.
    """
    if not strokes:
        return 0.0
    curr = strokes[0][0].xy if start_xy is None else start_xy
    total = 0.0
    for stroke in strokes:
        total += math.dist(curr, stroke[0].xy)
        curr = stroke[-1].xy
    return total


def _orient_strokes_dp(strokes, origin):
    """
    [EN] Use O(N) Dynamic Programming to find the globally optimal forward/reverse
         orientation for each open stroke in a fixed stroke sequence:
         - State `rev = 0`: draw stroke `i` forward (enters at `st[0]`, exits at `st[-1]`).
         - State `rev = 1`: draw stroke `i` reversed (enters at `st[-1]`, exits at `st[0]`).
         - Closed loops (`st[0] == st[-1]`) only allow `rev = 0` so their winding
           direction is preserved.
         - A tiny `1e-7` tie-breaking bias prefers keeping original forward orientation
           when forward and reverse travel distances are equal.

    [VI] Dùng Quy hoạch động O(N) để tìm chiều vẽ (thuận/ngược) tối ưu toàn cục
         cho từng nét hở trong một thứ tự nét cố định:
         - Trạng thái `rev = 0`: vẽ nét `i` theo chiều thuận (vào `st[0]`, ra `st[-1]`).
         - Trạng thái `rev = 1`: vẽ nét `i` theo chiều ngược (vào `st[-1]`, ra `st[0]`).
         - Các nét khép kín (`st[0] == st[-1]`) chỉ dùng `rev = 0` để giữ nguyên chiều.
         - Cộng trọng số nhỏ `1e-7` để ưu tiên giữ chiều thuận gốc khi quãng đường bằng nhau.
    """
    n = len(strokes)
    if n == 0:
        return (), 0.0

    # [EN] Step 1: Base case for stroke 0 starting from `origin`.
    # [VI] Bước 1: Cơ sở quy hoạch động cho nét đầu tiên (chỉ số 0) xuất phát từ `origin`.
    dp = []
    back = []
    st0 = strokes[0]
    open0 = math.dist(st0[0].xy, st0[-1].xy) > 1e-6
    c0 = math.dist(origin, st0[0].xy)
    if open0:
        c1 = math.dist(origin, st0[-1].xy) + 1e-7
    else:
        c1 = math.inf
    dp.append((c0, c1))
    back.append((0, 0))

    # [EN] Step 2: Forward DP pass across strokes 1 .. n-1.
    # [VI] Bước 2: Lan truyền quy hoạch động qua các nét từ 1 đến n-1.
    for i in range(1, n):
        prev_st = strokes[i - 1]
        # [EN] Exit point of previous stroke when forward (0) vs reversed (1).
        # [VI] Điểm kết thúc của nét trước đó ứng với chiều thuận (0) và chiều ngược (1).
        prev_exits = (prev_st[-1].xy, prev_st[0].xy)

        cur_st = strokes[i]
        is_open = math.dist(cur_st[0].xy, cur_st[-1].xy) > 1e-6
        # [EN] Entry point of current stroke when forward (0) vs reversed (1).
        # [VI] Điểm bắt đầu của nét hiện tại ứng với chiều thuận (0) và chiều ngược (1).
        cur_entries = (cur_st[0].xy, cur_st[-1].xy)

        row_cost = [math.inf, math.inf]
        row_back = [0, 0]
        allowed_orientations = (0, 1) if is_open else (0,)
        for cur_rev in allowed_orientations:
            bias = 1e-7 if cur_rev else 0.0
            for prev_rev in (0, 1):
                if math.isinf(dp[i - 1][prev_rev]):
                    continue
                transition_dist = math.dist(prev_exits[prev_rev], cur_entries[cur_rev])
                cand = dp[i - 1][prev_rev] + transition_dist + bias
                if cand < row_cost[cur_rev]:
                    row_cost[cur_rev] = cand
                    row_back[cur_rev] = prev_rev
        dp.append((row_cost[0], row_cost[1]))
        back.append((row_back[0], row_back[1]))

    # [EN] Step 3: Backtrack from the last stroke to recover optimal flip flags.
    # [VI] Bước 3: Truy vết ngược từ nét cuối cùng để lấy danh sách cờ đảo chiều tối ưu.
    state = 1 if dp[-1][1] < dp[-1][0] else 0
    best_cost = dp[-1][state]
    flips = [0] * n
    for i in range(n - 1, -1, -1):
        flips[i] = state
        state = back[i][state]

    # [EN] Step 4: Apply `reverse_stroke` to each stroke marked with rev == 1.
    # [VI] Bước 4: Đảo chiều các nét vẽ có cờ `rev == 1`.
    oriented_list = []
    for st, rev in zip(strokes, flips):
        if rev:
            oriented_list.append(reverse_stroke(st))
        else:
            oriented_list.append(st)
    return tuple(oriented_list), best_cost


def optimize_strokes(strokes, start_xy=None):
    """
    [EN] Minimize non-drawing (pen-up) air travel across strokes using multi-start
         Nearest-Neighbor seeding + 2-opt local search + exact orientation DP.
    [VI] Tối thiểu hóa quãng đường di chuyển nhấc bút trên không giữa các nét bằng
         thuật toán Láng giềng gần nhất đa điểm khởi đầu + tìm kiếm cục bộ 2-opt
         kết hợp Quy hoạch động chọn chiều nét vẽ.
    """
    strokes = tuple(strokes)
    n = len(strokes)
    if n <= 1:
        return strokes
    origin = strokes[0][0].xy if start_xy is None else start_xy

    # [EN] Step 1: Build a greedy Nearest-Neighbor (NN) stroke order from `origin`.
    # [VI] Bước 1: Xây dựng thứ tự nét tham lam theo Láng giềng gần nhất (NN) từ `origin`.
    remaining = list(range(n))
    nn_order = []
    curr = origin
    while remaining:
        best_choice = None
        best_cost = math.inf
        for idx in remaining:
            st = strokes[idx]
            is_open = math.dist(st[0].xy, st[-1].xy) > 1e-6
            orientations = (False, True) if is_open else (False,)
            for rev in orientations:
                entry = st[-1].xy if rev else st[0].xy
                cost = math.dist(curr, entry) + (1e-6 if rev else 0.0)
                if cost < best_cost:
                    best_cost = cost
                    best_choice = (idx, rev)
        idx, rev = best_choice
        remaining.remove(idx)
        nn_order.append(idx)
        curr = strokes[idx][0].xy if rev else strokes[idx][-1].xy

    # [EN] Step 2: Evaluate candidate seed orders (forward, reversed, and NN).
    # [VI] Bước 2: Đánh giá 3 thứ tự hạt giống khởi đầu (thuận gốc, đảo ngược, và NN).
    seeds = [
        list(range(n)),
        list(reversed(range(n))),
        nn_order,
    ]
    best_strokes = strokes
    best_total = pen_up_travel_distance(strokes, origin)

    # [EN] Step 3: Refine each seed with 2-opt segment reversals and exact DP orientation.
    # [VI] Bước 3: Tinh chỉnh từng hạt giống bằng hoán đổi 2-opt kết hợp quy hoạch động.
    for order in seeds:
        cur_order = list(order)
        ordered_strokes = [strokes[idx] for idx in cur_order]
        cur_strokes, cur_cost = _orient_strokes_dp(ordered_strokes, origin)

        if n <= 40:
            improved = True
            while improved:
                improved = False
                for i in range(n - 1):
                    for k in range(i + 1, n):
                        # [EN] 2-opt move: reverse the sub-sequence cur_order[i : k + 1].
                        # [VI] Phép thử 2-opt: đảo ngược đoạn con cur_order[i : k + 1].
                        cand_order = (
                            cur_order[:i]
                            + list(reversed(cur_order[i : k + 1]))
                            + cur_order[k + 1 :]
                        )
                        cand_sequence = [strokes[idx] for idx in cand_order]
                        cand_strokes, cand_cost = _orient_strokes_dp(
                            cand_sequence, origin
                        )
                        if cand_cost < cur_cost - 1e-4:
                            cur_order = cand_order
                            cur_strokes = cand_strokes
                            cur_cost = cand_cost
                            improved = True

        if cur_cost < best_total - 1e-4:
            best_total = cur_cost
            best_strokes = cur_strokes

    return best_strokes


# ==============================================================================
# 3. PATTERN GEOMETRY & INK DEVIATION / HÌNH HỌC MẪU VẼ & SAI LỆCH ĐƯỜNG MỰC
# ==============================================================================


def bk_strokes(settings):
    """
    [EN] Generate the 3 strokes (`B` stem + loops, `K` stem, `K` diagonals) for
         the classic `"BK"` emblem pattern in normalized `[0..0.5] x [0..0.5]` space
         scaled to `settings.size_mm`.
    [VI] Sinh 3 nét vẽ (thân + 2 bụng chữ `B`, thân chữ `K`, hai nét xiên chữ `K`)
         cho mẫu biểu tượng `"BK"` từ khung chuẩn `[0..0.5] x [0..0.5]` phóng theo
         kích thước `settings.size_mm`.
    """
    s = settings.validate()
    scale = s.size_mm / 0.5

    def xy(x, y):
        """[EN] Map normalized [0..0.5] coords to world XY / [VI] Đổi tọa độ chuẩn sang XY thực."""
        world_x = s.center_x + (x - 0.25) * scale
        world_y = s.center_y + (y - 0.25) * scale
        return (world_x, world_y)

    def line(start, end, part):
        """
        [EN] Subdivide normalized segment `start -> end` into chords <= `s.step_mm`.
        [VI] Chia nhỏ đoạn thẳng chuẩn hóa `start -> end` thành các bước <= `s.step_mm`.
        """
        x0, y0 = start
        x1, y1 = end
        n = max(1, math.ceil(math.dist(start, end) * scale / s.step_mm))
        vertices = []
        for i in range(1, n + 1):
            px = x0 + (x1 - x0) * i / n
            py = y0 + (y1 - y0) * i / n
            vertices.append(Vertex(xy(px, py), part))
        return vertices

    # [EN] Stroke 1: Letter "B" vertical stem + lower loop + upper loop.
    # [VI] Nét 1: Thân thẳng đứng chữ "B" nối liền bụng dưới và bụng trên.
    b = [Vertex(xy(0, 0.5), "B_stem")] + line((0, 0.5), (0, 0), "B_stem")
    for cy, part in ((0.125, "B_lower"), (0.375, "B_upper")):
        center_x, center_y = xy(0, cy)
        radius = s.size_mm / 4
        arc_points = ellipse_arc(
            center_x,
            center_y,
            radius,
            radius,
            -math.pi / 2,
            math.pi / 2,
            s.step_mm,
            s.tolerance_mm / 10,
        )
        for point in arc_points:
            b.append(Vertex(point, part))

    # [EN] Stroke 2: Letter "K" vertical stem.
    # [VI] Nét 2: Thân thẳng đứng của chữ "K".
    stem = [Vertex(xy(0.25, 0.5), "K_stem")] + line((0.25, 0.5), (0.25, 0), "K_stem")

    # [EN] Stroke 3: Letter "K" upper and lower diagonals meeting at (0.25, 0.25).
    # [VI] Nét 3: Hai nhánh xiên trên và dưới của chữ "K" gặp nhau tại tâm (0.25, 0.25).
    diagonal = (
        [Vertex(xy(0.5, 0.5), "K_upper")]
        + line((0.5, 0.5), (0.25, 0.25), "K_upper")
        + line((0.25, 0.25), (0.5, 0), "K_lower")
    )
    return (tuple(b), tuple(stem), tuple(diagonal))


def pattern_dimensions(settings):
    """
    [EN] Return `(width_mm, height_mm)` for any built-in shape, word, or custom vector.
    [VI] Trả về kích thước `(rộng_mm, cao_mm)` cho bất kỳ hình mẫu, chữ hoặc vector tùy chỉnh nào.
    """
    s = settings.validate()
    if s.pattern in CUSTOM_PATTERNS:
        return custom_dimensions(s)
    if s.pattern in SHAPES:
        return shape_dimensions(s)
    if s.pattern == "BK":
        return (s.size_mm, s.size_mm)
    width_mm, height_mm, _strokes = word_geometry(
        s.pattern, s.size_mm, s.center_x, s.center_y
    )
    return (width_mm, height_mm)


@lru_cache(maxsize=32)
def text_edges(settings):
    """
    [EN] Map `"TXT_i_j"` part labels to their exact 2D line segment endpoints `(a, b)`.
    [VI] Ánh xạ nhãn `"TXT_i_j"` sang tọa độ hai đầu mút đoạn thẳng `(a, b)` tương ứng.
    """
    polylines = word_geometry(
        settings.pattern, settings.size_mm, settings.center_x, settings.center_y
    )[2]
    edges = {}
    for i, line in enumerate(polylines):
        for j, (a, b) in enumerate(zip(line, line[1:])):
            edges[f"TXT_{i}_{j}"] = (a, b)
    return edges


def pattern_strokes(settings):
    """
    [EN] Generate subdivided `Vertex` strokes for the selected pattern in `settings`.
    [VI] Sinh danh sách các nét gồm các đỉnh `Vertex` đã chia nhỏ theo `settings`.
    """
    s = settings.validate()
    if s.pattern in CUSTOM_PATTERNS:
        return custom_strokes(s, Vertex)

    if s.pattern in SHAPES:
        converted_strokes = []
        for stroke in shape_vertices(s):
            vertex_list = []
            for xy, part in stroke:
                vertex_list.append(Vertex(xy, part))
            converted_strokes.append(tuple(vertex_list))
        return tuple(converted_strokes)

    if s.pattern == "BK":
        return bk_strokes(s)

    # [EN] Built-in stroke words ("LINH", "THOA"): subdivide each polyline segment.
    # [VI] Mẫu chữ tích hợp ("LINH", "THOA"): chia nhỏ từng đoạn thẳng trong nét chữ.
    lines = word_geometry(s.pattern, s.size_mm, s.center_x, s.center_y)[2]
    strokes = []
    for index, line in enumerate(lines):
        # [EN] Special case: stroke index 5 of "THOA" is the smooth elliptical letter "O".
        # [VI] Trường hợp đặc biệt: nét thứ 5 của chữ "THOA" là đường cong elip chữ "O".
        if s.pattern == "THOA" and index == 5:
            scale = s.size_mm / 3.7
            cx = s.center_x - s.size_mm / 2 + 2.35 * scale
            vertices = [Vertex((cx, s.center_y + scale / 2), "O_curve")]
            arc_pts = ellipse_arc(
                cx,
                s.center_y,
                0.35 * scale,
                0.5 * scale,
                math.pi / 2,
                -3 * math.pi / 2,
                s.step_mm,
                s.tolerance_mm / 10,
            )
            for point in arc_pts:
                vertices.append(Vertex(point, "O_curve"))
            strokes.append(tuple(vertices))
            continue

        vertices = [Vertex(line[0], f"TXT_{index}_0")]
        for j, (a, b) in enumerate(zip(line, line[1:])):
            x0, y0 = a
            x1, y1 = b
            n = max(1, math.ceil(math.dist(a, b) / s.step_mm))
            part_label = f"TXT_{index}_{j}"
            for i in range(1, n + 1):
                px = x0 + (x1 - x0) * i / n
                py = y0 + (y1 - y0) * i / n
                vertices.append(Vertex((px, py), part_label))
        strokes.append(tuple(vertices))
    return tuple(strokes)


def ink_distance(xy, part, s):
    """
    [EN] Compute the geometric deviation (mm) from 2D point `xy` to the ideal
         primitive curve/line identified by `part`.
    [VI] Tính khoảng cách sai lệch hình học (mm) từ điểm 2D `xy` tới phần tử
         đường cong/đoạn thẳng lý tưởng được định danh bởi `part`.
    """
    if s.pattern in CUSTOM_PATTERNS:
        return custom_ink_distance(xy, part, s)
    if s.pattern in SHAPES:
        return shape_ink_distance(xy, part, s)
    if part == "O_curve":
        scale = s.size_mm / 3.7
        cx = s.center_x - s.size_mm / 2 + 2.35 * scale
        return ellipse_radial_error(xy, cx, s.center_y, 0.35 * scale, 0.5 * scale)
    if s.pattern != "BK":
        edge_start, edge_end = text_edges(s)[part]
        return segment_distance(xy, edge_start, edge_end)

    # [EN] Ideal primitive geometry for the "BK" pattern:
    # [VI] Hình học chuẩn lý tưởng của các phần tử trong mẫu chữ "BK":
    x0 = s.center_x - s.size_mm / 2
    y0 = s.center_y - s.size_mm / 2
    y1 = y0 + s.size_mm
    if part == "B_stem":
        return segment_distance(xy, (x0, y0), (x0, y1))
    if part == "K_stem":
        return segment_distance(xy, (s.center_x, y0), (s.center_x, y1))
    if part in ("K_upper", "K_lower"):
        end_y = y1 if part == "K_upper" else y0
        end = (s.center_x + s.size_mm / 2, end_y)
        return segment_distance(xy, (s.center_x, s.center_y), end)

    # [EN] Right-half semicircular loops of "B" centered at (x0, cy) with radius r.
    # [VI] Nửa vòng tròn bên phải bụng chữ "B" có tâm tại (x0, cy) và bán kính r.
    direction = -1 if part == "B_lower" else 1
    cy = s.center_y + direction * s.size_mm / 4
    r = s.size_mm / 4
    if xy[0] >= x0:
        return abs(math.dist(xy, (x0, cy)) - r)
    return min(math.dist(xy, (x0, cy - r)), math.dist(xy, (x0, cy + r)))


def dda_xyz(model, start, plan):
    """
    [EN] Simulate the exact integer Bresenham/DDA pulse accumulator from `start`
         to `plan.steps` and yield the resulting Forward Kinematics `(x, y, z)`
         after every single tick.
         Algorithm:
         - `delta = (dx_steps, dy_steps, dz_steps)` is the signed step count on each axis.
         - `ticks = max(|dx|, |dy|, |dz|)` is the dominant axis step count.
         - On each tick, each axis `a` adds `|delta[a]|` to `accum[a]`. Whenever
           `accum[a] >= ticks`, we subtract `ticks` and emit one step (`+1` or `-1`).

    [VI] Mô phỏng chính xác bộ cộng dồn xung nguyên Bresenham/DDA từ `start` tới
         `plan.steps` và sinh ra tọa độ động học thuận `(x, y, z)` tại từng nhịp xung.
         Thuật toán:
         - `delta = (dx_steps, dy_steps, dz_steps)` là số xung dịch chuyển trên 3 trục.
         - `ticks = max(|dx|, |dy|, |dz|)` là tổng số nhịp xung theo trục dài nhất.
         - Ở mỗi nhịp, trục `a` cộng dồn `|delta[a]|` vào `accum[a]`. Khi `accum[a] >= ticks`,
           trừ đi `ticks` và phát 1 xung bước (`+1` hoặc `-1`) trên trục đó.
    """
    # [EN] Step 1: Compute signed pulse deltas and total dominant-axis ticks.
    # [VI] Bước 1: Tính độ lệch số xung có dấu trên 3 trục và tổng số nhịp `ticks`.
    delta_list = []
    for a, b in zip(start, plan.steps):
        delta_list.append(b - a)
    delta = tuple(delta_list)

    ticks = max(abs(delta[0]), abs(delta[1]), abs(delta[2]))
    if not ticks:
        return

    # [EN] Step 2: Run the integer accumulator tick-by-tick and yield FK coordinates.
    # [VI] Bước 2: Chạy bộ cộng dồn số nguyên qua từng nhịp và trả về tọa độ FK.
    position = list(start)
    accum = [0, 0, 0]
    for _ in range(ticks):
        for axis in range(3):
            accum[axis] += abs(delta[axis])
            if accum[axis] >= ticks:
                accum[axis] -= ticks
                step_dir = 1 if delta[axis] > 0 else -1
                position[axis] += step_dir
        joint_coords = model.from_steps(position)
        yield model.fk(joint_coords)


# ==============================================================================
# 4. DRAWING COMPILER & AUTO-FITTER / TRÌNH BIÊN DỊCH BẢN VẼ & TỰ ĐỘNG CĂN CỠ
# ==============================================================================


@dataclass(frozen=True)
class DrawingOperation:
    """
    [EN] One compiled waypoint operation (`kind` in `"lift"`, `"travel"`, `"lower"`,
         `"ink"`, `"dry"`) belonging to stroke index `stroke`.
    [VI] Một thao tác điểm đặt đã biên dịch (`kind` thuộc `"lift"`, `"travel"`,
         `"lower"`, `"ink"`, `"dry"`) thuộc về nét vẽ thứ `stroke`.
    """

    point: Setpoint
    kind: str
    stroke: int
    part: str = ""


@dataclass(frozen=True)
class DrawingProgram:
    """
    [EN] Fully validated drawing program containing discrete operations,
         optimized strokes, maximum model deviation (mm), and lookahead `StrokeRun`s.
    [VI] Chương trình vẽ đã kiểm duyệt hoàn chỉnh gồm danh sách thao tác, các nét vẽ
         đã tối ưu, sai lệch mô hình lớn nhất (mm) và các chuỗi chạy liên tục `StrokeRun`.
    """

    settings: BKSettings
    dry: bool
    start_steps: tuple
    operations: tuple
    strokes: tuple
    max_model_error_mm: float
    runs: tuple = ()

    @property
    def points(self):
        pts = []
        for op in self.operations:
            pts.append(op.point)
        return tuple(pts)


def _stroke_centroid_xy(stroke):
    """
    [EN] Compute the average `(mean_x, mean_y)` of all vertices in `stroke`.
    [VI] Tính tọa độ trung bình `(mean_x, mean_y)` của tất cả các đỉnh trong `stroke`.
    """
    sum_x = 0
    sum_y = 0
    for v in stroke:
        sum_x += v.xy[0]
        sum_y += v.xy[1]
    count = len(stroke)
    return (sum_x / count, sum_y / count)


def compile_bk(
    model, start_steps, settings, dry=False, cancelled=lambda: False, optimize=True
):
    """
    [EN] Read-only full preflight compiler:
         1. Validates Z paper and lift heights against calibrated Z travel.
         2. Optimizes stroke ordering/orientation (`optimize_strokes`).
         3. Simulates every DDA pulse (`model.plan` + `dda_xyz`) and verifies that
            every emitted step stays within `settings.tolerance_mm` of the ideal curve.
         4. Groups contiguous pen-down segments into retimed `StrokeRun` FIFO batches.
    [VI] Trình biên dịch kiểm duyệt trước toàn diện (chỉ đọc):
         1. Kiểm tra cao độ Z mặt giấy và Z nâng bút nằm trong hành trình Z đã hiệu chuẩn.
         2. Tối ưu hóa thứ tự và chiều các nét vẽ (`optimize_strokes`).
         3. Mô phỏng từng xung DDA (`model.plan` + `dda_xyz`) và đảm bảo mọi bước xung
            đều bám sát đường lý tưởng trong phạm vi `settings.tolerance_mm`.
         4. Gom các đoạn hạ bút liên tiếp thành các lô `StrokeRun` có quy hoạch vận tốc.
    """
    # [EN] Step 1: Validate settings and generate raw pattern strokes.
    # [VI] Bước 1: Kiểm tra thông số cấu hình và sinh danh sách nét vẽ ban đầu.
    s = settings.validate(paper=True)
    strokes = pattern_strokes(s)
    lo, hi = model.z_limits_mm

    # [EN] Step 2: Evaluate tilted paper Z height at each stroke's centroid and verify Z limits.
    # [VI] Bước 2: Tính cao độ Z mặt giấy tại trọng tâm mỗi nét và kiểm tra giới hạn trục Z.
    stroke_z_vals = []
    for st in strokes:
        mean_x, mean_y = _stroke_centroid_xy(st)
        stroke_z_vals.append(s.paper_z_at(mean_x, mean_y))

    all_paper_z = [s.paper_z, *stroke_z_vals]
    up = max(all_paper_z) + s.lift_mm
    if not lo <= min(all_paper_z) < up <= hi:
        raise ValueError("Z giấy hoặc Z nhấc bút nằm ngoài hành trình đã home.")
    if s.lift_mm * model.factors[0] < 1:
        raise ValueError("Độ nhấc bút nhỏ hơn một xung Z; kiểm tra thông số Z.")

    # [EN] Step 3: Verify Inverse Kinematics reachability for every vertex before planning.
    # [VI] Bước 3: Kiểm tra động học ngược (IK) cho từng đỉnh trước khi lập kế hoạch quỹ đạo.
    for stroke in strokes:
        mean_x, mean_y = _stroke_centroid_xy(stroke)
        z_st = s.paper_z_at(mean_x, mean_y)
        for v in stroke:
            model.ik((*v.xy, z_st))

    position = start_steps
    operations = []
    max_error = 0.0
    plans = []
    starts = []

    def append(xyz, kind, stroke, part=""):
        """
        [EN] Plan one motion step from `position` to `xyz`, verify DDA ink deviation
             for drawing moves, and record the resulting `DrawingOperation`.
        [VI] Lập kế hoạch cho một bước di chuyển từ `position` tới `xyz`, kiểm tra
             sai lệch từng xung DDA khi vẽ, và lưu vào danh sách `DrawingOperation`.
        """
        nonlocal position, max_error
        if cancelled():
            raise ValueError("Tác vụ vẽ đã hủy.")
        if len(operations) >= 2000:
            raise ValueError("Mẫu vượt2000đoạn; tăng bước nét hoặc giảm cỡ chữ.")

        speed = s.draw_speed if kind in ("ink", "dry") else s.travel_speed
        dwell = s.settle_s if kind == "lower" else 0
        point = Setpoint(
            f"{s.pattern}{stroke+1}_{kind}_{len(operations)+1}",
            *xyz,
            speed,
            dwell,
        ).validate()

        try:
            plan = model.plan(position, point.xyz, point.speed_mm_s)
        except ValueError as e:
            raise ValueError(f"{point.name}: {e}") from e

        if kind in ("ink", "dry"):
            # [EN] Check geometric deviation across both the ideal path and every DDA pulse.
            # [VI] Kiểm tra sai lệch hình học trên cả đường nội suy và từng xung bước DDA.
            dda_points = tuple(dda_xyz(model, position, plan))
            sample_points = (*plan.path, *dda_points)
            error = 0
            for p in sample_points:
                dist = ink_distance(p[:2], part, s)
                if dist > error:
                    error = dist

            max_error = max(max_error, error)
            if error > s.tolerance_mm:
                raise ValueError(
                    f"Sai lệch mô hình{error:.3f}mm vượt{s.tolerance_mm:g}mm tại{part}; kiểm tra độ phân giải/bước nét."
                )
            # [EN] A finer curve can produce repeated rounded motor coordinates.
            #      Skip the stationary vertex only after checking its ink error.
            # [VI] Đường cong chia mịn có thể tạo ra 2 điểm làm tròn trùng tọa độ xung.
            #      Chỉ bỏ qua đỉnh đứng yên sau khi đã kiểm tra sai số nét mực.
            if plan.steps == position:
                return

        plans.append(plan)
        starts.append(position)
        operations.append(DrawingOperation(point, kind, stroke, part))
        position = plan.steps

    # [EN] Step 4: Optimize stroke sequence/orientation starting from current FK pose.
    # [VI] Bước 4: Tối ưu hóa thứ tự và chiều các nét vẽ bắt đầu từ vị trí hiện tại.
    start = model.fk(model.from_steps(start_steps))
    if optimize:
        strokes = optimize_strokes(strokes, start[:2])

    # [EN] Step 5: Emit initial vertical lift at current XY, then compile each stroke.
    #      Never combine the initial lift with a lateral transfer from an arbitrary pose.
    # [VI] Bước 5: Nhấc bút thẳng đứng tại đúng tọa độ XY hiện tại, sau đó biên dịch từng nét.
    append((*start[:2], up), "lift", 0)
    for index, stroke in enumerate(strokes):
        mean_x, mean_y = _stroke_centroid_xy(stroke)
        z_stroke = s.paper_z_at(mean_x, mean_y)

        append((*stroke[0].xy, up), "travel", index)
        if not dry:
            append((*stroke[0].xy, z_stroke), "lower", index)

        draw_z = up if dry else z_stroke
        draw_kind = "dry" if dry else "ink"
        for vertex in stroke[1:]:
            append(
                (*vertex.xy, draw_z),
                draw_kind,
                index,
                vertex.part,
            )

        if not dry:
            append((*stroke[-1].xy, up), "lift", index)

    # [EN] Step 6: Group contiguous ink/dry operations of the same stroke into lookahead StrokeRuns.
    # [VI] Bước 6: Gom các đoạn vẽ liên tiếp của cùng một nét thành các chuỗi StrokeRun chạy FIFO.
    runs = []
    i = 0
    while i < len(operations):
        if operations[i].kind not in ("ink", "dry"):
            i += 1
            continue
        first = i
        stroke = operations[i].stroke
        while (
            i < len(operations)
            and operations[i].kind in ("ink", "dry")
            and operations[i].stroke == stroke
        ):
            i += 1
        if i - first > 500:
            raise ValueError("Một nét vượt500đoạn; tăng bước nét hoặc giảm cỡ chữ.")

        xy_chords = []
        for j in range(first, i):
            prev_xy = operations[j - 1].point.xyz[:2]
            curr_xy = operations[j].point.xyz[:2]
            xy_chords.append((prev_xy, curr_xy))

        segments = lookahead(
            plans[first:i],
            starts[first:i],
            xy_chords,
        )

        run_duration = 0
        for seg in segments:
            run_duration += seg.seconds()

        runs.append(
            StrokeRun(
                first,
                i,
                tuple(starts[first]),
                segments,
                run_duration,
            )
        )

    return DrawingProgram(
        s, dry, tuple(start_steps), tuple(operations), strokes, max_error, tuple(runs)
    )


def fit_drawing(model, start_steps, settings, cancelled=lambda: False):
    """
    [EN] Binary-search the largest pattern width (in [4, 180] mm) that fits inside
         the calibrated workspace at `(settings.center_x, settings.center_y)`,
         apply a 4% safety margin, and verify full dry-run preflight compilation.
         Search is read-only and never confirms paper Z.
    [VI] Tìm kiếm nhị phân chiều rộng mẫu lớn nhất (trong khoảng [4, 180] mm)
         nằm gọn trong vùng làm việc tại tâm `(settings.center_x, settings.center_y)`,
         trừ biên dự phòng 4%, và kiểm duyệt toàn bộ đường chạy thử.
         Quá trình tìm kiếm chỉ đọc và không tự ý xác nhận cao độ Z mặt giấy.
    """
    settings.validate()

    def geometry_ok(width):
        """
        [EN] Test whether all vertices at `size_mm = width` are IK-reachable.
        [VI] Kiểm tra tất cả các đỉnh ở kích thước `size_mm = width` có thỏa mãn động học ngược (IK).
        """
        if cancelled():
            raise ValueError("Tìm kích thước đã hủy.")
        candidate = replace(settings, size_mm=width)
        try:
            for stroke in pattern_strokes(candidate):
                for vertex in stroke:
                    model.ik((*vertex.xy, model.z_limits_mm[0]))
            return True
        except ValueError:
            return False

    # [EN] Step 1: Ensure even the minimum 4 mm pattern fits at the chosen XY center.
    # [VI] Bước 1: Kiểm tra kích thước tối thiểu 4 mm có nằm trong vùng làm việc tại tâm đã chọn.
    if not geometry_ok(4):
        raise ValueError(
            "Tâm hiện tại không chứa được mẫu; chỉnh tâm XY vào vùng làm việc."
        )

    # [EN] Step 2: 18-iteration binary search on [4.0, 180.0] mm for maximum reachable width.
    # [VI] Bước 2: Tìm kiếm nhị phân 18 bước trên đoạn [4.0, 180.0] mm để tìm chiều rộng tối đa.
    low = 4.0
    high = 180.0
    for _ in range(18):
        mid = (low + high) / 2
        if geometry_ok(mid):
            low = mid
        else:
            high = mid

    # [EN] Step 3: Apply a 4% safety margin (multiply by 0.96) and round down to 0.01 mm.
    # [VI] Bước 3: Áp dụng biên an toàn 4% (nhân 0.96) và làm tròn xuống tới 0.01 mm.
    width = max(4, math.floor(low * 0.96 * 100) / 100)

    zlo, zhi = model.z_limits_mm
    if settings.lift_mm >= zhi - zlo:
        raise ValueError("Độ nhấc bút vượt hành trình Z.")
    trial_z = (zlo + zhi - settings.lift_mm) / 2

    # [EN] Step 4: Verify full preflight compilation (`compile_bk`), shrinking by 10% if needed.
    # [VI] Bước 4: Kiểm duyệt toàn trình với `compile_bk`, giảm dần 10% nếu vướng giới hạn đoạn/dung sai.
    for _ in range(12):
        candidate = replace(settings, size_mm=width)
        try:
            compile_bk(
                model, start_steps, replace(candidate, paper_z=trial_z), True, cancelled
            )
            return candidate
        except ValueError:
            if cancelled():
                raise ValueError("Tìm kích thước đã hủy.")
            if width <= 4:
                break
            width = max(4, math.floor(width * 0.9 * 100) / 100)

    raise ValueError(
        "Chưa tìm được cỡ chữ đạt dung sai; chỉnh tâm, bước nét hoặc kiểm tra độ phân giải."
    )
