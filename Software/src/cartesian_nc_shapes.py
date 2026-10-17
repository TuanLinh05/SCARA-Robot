"""
================================================================================
FILE: cartesian_nc_shapes.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     Built-in parametric vector shape templates (`CIRCLE`, `ELLIPSE`, `SQUARE`,
     `TRIANGLE`, `DIAMOND`, `STAR`, `HEART`, `FLOWER`), exact bounding-box
     calculation (including cubic Bézier derivative roots), adaptive curve
     tessellation, and bounding-volume-hierarchy (BVH) distance queries for
     preflight ink-deviation verification.

[VI] MỤC ĐÍCH:
     Thư viện mẫu hình học vector tham số tích hợp sẵn (`CIRCLE`, `ELLIPSE`,
     `SQUARE`, `TRIANGLE`, `DIAMOND`, `STAR`, `HEART`, `FLOWER`), tính toán
     khung bao chính xác (bao gồm nghiệm đạo hàm đường cong Bézier bậc 3),
     chia nhỏ đoạn thích ứng, và cây phân cấp hộp bao (BVH) để truy vấn khoảng
     cách sai lệch nét mực trong bước kiểm duyệt đường chạy.

[DEBUG TIP / MẸO GỠ LỖI]:
     - In `shape_vertices()`, notice `vertices[-1] = (vertices[0][0], vertices[-1][1])`:
       every built-in geometric motif is a closed loop, so snapping the final
       vertex to the exact first vertex prevents floating-point seam gaps!
     - Trong hàm `shape_vertices()`, lưu ý dòng `vertices[-1] = (vertices[0][0], vertices[-1][1])`:
       mọi mẫu hình học tích hợp đều là đường khép kín, việc gán điểm cuối trùng
       khít điểm đầu giúp triệt tiêu khe hở do sai số dấu phẩy động!
================================================================================
"""

from dataclasses import dataclass
from functools import lru_cache
import math
from cartesian_nc_curves import flatten_cubic, ellipse_arc, ellipse_radial_error
from cartesian_nc_geometry import segment_distance_squared as line_distance_squared

# ==============================================================================
# 1. PATTERN CATALOG & PRIMITIVE PIECE / DANH MỤC MẪU & PHẦN TỬ HÌNH HỌC CƠ SỞ
# ==============================================================================

# [EN] Built-in pattern dictionary: code -> (Vietnamese label, group, default width mm).
# [VI] Từ điển mẫu tích hợp sẵn: mã -> (Tên hiển thị tiếng Việt, nhóm, chiều rộng mặc định mm).
PATTERNS = {
    "FLOWER": ("Hoa 6 cánh", "Hình vẽ", 24),
    "STAR": ("Ngôi sao", "Hình vẽ", 24),
    "HEART": ("Trái tim", "Hình vẽ", 24),
    "CIRCLE": ("Hình tròn", "Hình vẽ", 24),
    "ELLIPSE": ("Elip", "Hình vẽ", 24),
    "SQUARE": ("Hình vuông", "Hình vẽ", 24),
    "TRIANGLE": ("Tam giác", "Hình vẽ", 24),
    "DIAMOND": ("Hình thoi", "Hình vẽ", 24),
    "BK": ("BK", "Chữ", 20),
    "LINH": ("LINH", "Chữ", 40),
    "THOA": ("THOA", "Chữ", 40),
}

# [EN] Collect all pattern keys that belong to the geometric shape group ("Hình vẽ").
# [VI] Tập hợp tất cả các mã mẫu thuộc nhóm hình học ("Hình vẽ").
_shape_keys = []
for _code, _info in PATTERNS.items():
    if _info[1] == "Hình vẽ":
        _shape_keys.append(_code)
SHAPES = frozenset(_shape_keys)


@dataclass(frozen=True)
class Piece:
    """
    [EN] A single geometric curve primitive (`kind` in `"line"`, `"ellipse"`, `"cubic"`).
    [VI] Một phần tử hình học cơ sở (`kind` thuộc `"line"`, `"ellipse"`, `"cubic"`).
    """

    kind: str
    data: tuple


# ==============================================================================
# 2. EXACT BOUNDING BOX & TEMPLATES / TÍNH KHUNG BAO CHÍNH XÁC & MẪU CHUẨN HÓA
# ==============================================================================


def cubic_value(p, t):
    """
    [EN] Evaluate 2D cubic Bézier curve `p = (P0, P1, P2, P3)` at parameter `t` in [0, 1].
         Bernstein polynomial formula:
             B(t) = (1 - t)^3 * P0 + 3*(1 - t)^2 * t * P1 + 3*(1 - t) * t^2 * P2 + t^3 * P3
    [VI] Tính tọa độ điểm trên đường cong Bézier bậc 3 `p = (P0, P1, P2, P3)` tại tham số `t`.
         Công thức đa thức Bernstein:
             B(t) = (1 - t)^3 * P0 + 3*(1 - t)^2 * t * P1 + 3*(1 - t) * t^2 * P2 + t^3 * P3
    """
    v = 1 - t
    coords = []
    for a, b, c, d in zip(*p):
        coord_val = v**3 * a + 3 * v * v * t * b + 3 * v * t * t * c + t**3 * d
        coords.append(coord_val)
    return tuple(coords)


def piece_bounds(piece):
    """
    [EN] Compute the exact axis-aligned bounding box `(xmin, ymin, xmax, ymax)`
         of a `Piece`, solving the quadratic derivative `B'(t) = 0` for cubic curves.
         Derivative of a 1D cubic Bézier B(t) with control values (a, b, c, d):
             B'(t) = qa * t^2 + qb * t + qc = 0
         where:
             qa = 3 * (-a + 3b - 3c + d)
             qb = 2 * (3a - 6b + 3c)
             qc = 3 * (b - a)
         Any real root `t` strictly inside `(0, 1)` is a local extremum candidate.

    [VI] Tính khung bao chữ nhật chính xác `(xmin, ymin, xmax, ymax)` của một `Piece`,
         giải phương trình bậc hai đạo hàm `B'(t) = 0` đối với đường cong Bézier bậc 3.
         Đạo hàm của đường Bézier bậc 3 trên từng trục với 4 giá trị điều khiển (a, b, c, d):
             B'(t) = qa * t^2 + qb * t + qc = 0
         với:
             qa = 3 * (-a + 3b - 3c + d)
             qb = 2 * (3a - 6b + 3c)
             qc = 3 * (b - a)
         Mọi nghiệm thực `t` nằm trong khoảng `(0, 1)` đều là điểm cực trị tiềm năng.
    """
    # [EN] Case 1: Full ellipse centered at (cx, cy) with semi-axes (rx, ry).
    # [VI] Trường hợp 1: Hình elip tâm (cx, cy) với bán trục (rx, ry).
    if piece.kind == "ellipse":
        cx, cy, rx, ry = piece.data
        return (cx - rx, cy - ry, cx + rx, cy + ry)

    # [EN] Case 2: Line segment or cubic Bézier — start with the endpoints.
    # [VI] Trường hợp 2: Đoạn thẳng hoặc Bézier bậc 3 — khởi tạo danh sách bằng 2 điểm đầu mút.
    if piece.kind == "line":
        points = list(piece.data)
    else:
        points = [piece.data[0], piece.data[3]]

    # [EN] For cubic Bézier curves, also include any interior extrema where B'(t) == 0.
    # [VI] Với đường cong Bézier bậc 3, thêm các điểm cực trị nội bộ nơi đạo hàm B'(t) == 0.
    if piece.kind == "cubic":
        for values in zip(*piece.data):
            a, b, c, d = values
            qa = 3 * (-a + 3 * b - 3 * c + d)
            qb = 2 * (3 * a - 6 * b + 3 * c)
            qc = 3 * (b - a)

            # [EN] Solve qa * t^2 + qb * t + qc = 0 (handling linear degeneration qa ~ 0).
            # [VI] Giải phương trình qa * t^2 + qb * t + qc = 0 (xử lý cả suy biến bậc nhất qa ~ 0).
            if abs(qa) < 1e-12:
                if abs(qb) < 1e-12:
                    roots = ()
                else:
                    roots = (-qc / qb,)
            else:
                disc = qb * qb - 4 * qa * qc
                if disc < 0:
                    roots = ()
                else:
                    sqrt_disc = math.sqrt(disc)
                    root1 = (-qb + sqrt_disc) / (2 * qa)
                    root2 = (-qb - sqrt_disc) / (2 * qa)
                    roots = (root1, root2)

            for t in roots:
                if 0 < t < 1:
                    points.append(cubic_value(piece.data, t))

    # [EN] Extract bounding box limits across all candidate points.
    # [VI] Tìm giá trị nhỏ nhất và lớn nhất trên trục X và trục Y.
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs), min(ys), max(xs), max(ys))


def polygon(points):
    """
    [EN] Convert a sequence of 2D vertices into a single-stroke tuple of `"line"` pieces.
    [VI] Chuyển chuỗi các đỉnh 2D thành một nét gồm các phần tử đoạn thẳng `"line"`.
    """
    line_pieces = []
    for a, b in zip(points, points[1:]):
        line_pieces.append(Piece("line", (a, b)))
    return (tuple(line_pieces),)


@lru_cache(maxsize=8)
def shape_template(pattern):
    """
    [EN] Return the normalized unit-scale strokes for a built-in geometric `pattern`.
    [VI] Trả về danh sách các nét vẽ ở tỷ lệ đơn vị chuẩn hóa cho mẫu hình học `pattern`.
    """
    if pattern == "CIRCLE":
        return ((Piece("ellipse", (0, 0, 0.5, 0.5)),),)
    if pattern == "ELLIPSE":
        return ((Piece("ellipse", (0, 0, 0.5, 0.33)),),)
    if pattern == "SQUARE":
        return polygon(
            ((-0.5, 0.5), (0.5, 0.5), (0.5, -0.5), (-0.5, -0.5), (-0.5, 0.5))
        )
    if pattern == "TRIANGLE":
        return polygon(
            (
                (0, math.sqrt(3) / 4),
                (0.5, -math.sqrt(3) / 4),
                (-0.5, -math.sqrt(3) / 4),
                (0, math.sqrt(3) / 4),
            )
        )
    if pattern == "DIAMOND":
        return polygon(((0, 0.5), (0.5, 0), (0, -0.5), (-0.5, 0), (0, 0.5)))
    if pattern == "STAR":
        # [EN] 5-pointed star: 10 alternating outer (r = 0.5) and inner (r = 0.205) vertices.
        # [VI] Ngôi sao 5 cánh: 10 đỉnh đan xen giữa đỉnh ngoài (r = 0.5) và đỉnh trong (r = 0.205).
        star_points = []
        for i in range(10):
            r = 0.5 if i % 2 == 0 else 0.205
            angle = math.pi / 2 - i * math.pi / 5
            px = r * math.cos(angle)
            py = r * math.sin(angle)
            star_points.append((px, py))
        points = tuple(star_points)
        return polygon((*points, points[0]))
    if pattern == "HEART":
        # [EN] Closed heart outline formed by 4 smooth cubic Bézier curves.
        # [VI] Đường viền trái tim khép kín tạo bởi 4 đường cong Bézier bậc 3 nối tiếp.
        return (
            (
                Piece("cubic", ((0, -0.5), (-0.12, -0.32), (-0.5, -0.12), (-0.5, 0.1))),
                Piece("cubic", ((-0.5, 0.1), (-0.5, 0.62), (-0.07, 0.58), (0, 0.25))),
                Piece("cubic", ((0, 0.25), (0.07, 0.58), (0.5, 0.62), (0.5, 0.1))),
                Piece("cubic", ((0.5, 0.1), (0.5, -0.12), (0.12, -0.32), (0, -0.5))),
            ),
        )
    if pattern == "FLOWER":
        # [EN] 6-petal flower: each petal is 2 cubic Bézier curves rotated by 60 deg steps,
        #      plus a small circular pistil at the center.
        # [VI] Hoa 6 cánh: mỗi cánh gồm 2 đường Bézier bậc 3 xoay cách nhau 60 độ,
        #      kèm nhụy tròn nhỏ ở chính giữa.
        strokes = []
        base_petal_halves = (
            ((0.1, 0), (0.2, 0.18), (0.5, 0.14), (0.5, 0)),
            ((0.5, 0), (0.5, -0.14), (0.2, -0.18), (0.1, 0)),
        )
        for i in range(6):
            angle = math.pi / 2 + i * math.pi / 3
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)

            petal_pieces = []
            for control_pts in base_petal_halves:
                rotated_ctrl = []
                for p in control_pts:
                    rx = p[0] * cos_a - p[1] * sin_a
                    ry = p[0] * sin_a + p[1] * cos_a
                    rotated_ctrl.append((rx, ry))
                petal_pieces.append(Piece("cubic", tuple(rotated_ctrl)))
            strokes.append(tuple(petal_pieces))

        strokes.append((Piece("ellipse", (0, 0, 0.07, 0.07)),))
        return tuple(strokes)
    raise ValueError("Mẫu hình không hợp lệ.")


# ==============================================================================
# 3. SCALING, TESSELLATION & BVH DISTANCE / TỶ LỆ, CHIA ĐOẠN & KHOẢNG CÁCH BVH
# ==============================================================================


@lru_cache(maxsize=32)
def scaled_pieces(pattern, width, cx, cy):
    """
    [EN] Scale `pattern` to exact horizontal `width` (mm) centered at `(cx, cy)`.
         Returns `(width_mm, height_mm, scaled_strokes)`.
    [VI] Phóng/thu mẫu `pattern` theo đúng chiều rộng `width` (mm) đặt tâm tại `(cx, cy)`.
         Trả về `(chiều_rộng_mm, chiều_cao_mm, danh_sách_nét_đã_tỷ_lệ)`.
    """
    strokes = shape_template(pattern)

    # [EN] Step 1: Compute bounding boxes of all primitive pieces to find template bounds.
    # [VI] Bước 1: Tính khung bao của mọi phần tử cơ sở để xác định kích thước gốc của mẫu.
    boxes = []
    for line in strokes:
        for p in line:
            boxes.append(piece_bounds(p))

    x0 = min(b[0] for b in boxes)
    x1 = max(b[2] for b in boxes)
    y0 = min(b[1] for b in boxes)
    y1 = max(b[3] for b in boxes)

    # [EN] Step 2: Compute uniform scale factor and template center (ox, oy).
    # [VI] Bước 2: Tính hệ số phóng đại đồng nhất và tọa độ tâm gốc (ox, oy) của mẫu.
    scale = width / (x1 - x0)
    ox = (x0 + x1) / 2
    oy = (y0 + y1) / 2

    def point(p):
        """[EN] Transform normalized point `p` to world XY / [VI] Đổi điểm chuẩn hóa sang XY thực."""
        return (cx + (p[0] - ox) * scale, cy + (p[1] - oy) * scale)

    def convert(piece):
        """[EN] Scale and translate one primitive `piece` / [VI] Phóng tỷ lệ và dịch chuyển 1 phần tử."""
        if piece.kind == "ellipse":
            x, y, rx, ry = piece.data
            center_xy = point((x, y))
            return Piece("ellipse", (center_xy[0], center_xy[1], rx * scale, ry * scale))
        transformed_pts = []
        for pt in piece.data:
            transformed_pts.append(point(pt))
        return Piece(piece.kind, tuple(transformed_pts))

    # [EN] Step 3: Transform every stroke in the template.
    # [VI] Bước 3: Biến đổi toàn bộ các nét vẽ trong mẫu.
    scaled_strokes = []
    for line in strokes:
        scaled_line = []
        for p in line:
            scaled_line.append(convert(p))
        scaled_strokes.append(tuple(scaled_line))

    height = (y1 - y0) * scale
    return (width, height, tuple(scaled_strokes))


def shape_dimensions(settings):
    """
    [EN] Return `(width_mm, height_mm)` for the shape specified in `settings`.
    [VI] Trả về `(chiều_rộng_mm, chiều_cao_mm)` của hình vẽ trong `settings`.
    """
    width_mm, height_mm, _strokes = scaled_pieces(
        settings.pattern, settings.size_mm, settings.center_x, settings.center_y
    )
    return (width_mm, height_mm)


def piece_vertices(piece, step, error):
    """
    [EN] Tessellate a single `Piece` into `(start_xy, tuple_of_end_xys)` with
         maximum chord length `step` and curve flatness bound `error`.
    [VI] Chia nhỏ một phần tử `Piece` thành `(điểm_đầu, chuỗi_điểm_tiếp_theo)` với
         bước chia tối đa `step` và sai số độ phẳng đường cong `error`.
    """
    if piece.kind == "ellipse":
        cx, cy, rx, ry = piece.data
        start = (cx, cy + ry)
        ends = ellipse_arc(cx, cy, rx, ry, math.pi / 2, -3 * math.pi / 2, step, error)
        return start, ends

    if piece.kind == "cubic":
        return piece.data[0], flatten_cubic(piece.data, step, error)

    # [EN] Straight line segment: subdivide evenly so no chord exceeds `step` mm.
    # [VI] Đoạn thẳng: chia đều thành `n` phần sao cho không đoạn con nào dài quá `step` mm.
    a, b = piece.data
    x0, y0 = a
    x1, y1 = b
    n = max(1, math.ceil(math.dist(a, b) / step))
    end_points = []
    for i in range(1, n + 1):
        px = x0 + (x1 - x0) * i / n
        py = y0 + (y1 - y0) * i / n
        end_points.append((px, py))
    return a, tuple(end_points)


def shape_vertices(settings):
    """
    [EN] Generate labeled vertex strokes `(((x, y), "SH_i_j"), ...)` for `settings`.
    [VI] Sinh danh sách các nét gồm các đỉnh kèm nhãn `(((x, y), "SH_i_j"), ...)`.
    """
    strokes = scaled_pieces(
        settings.pattern, settings.size_mm, settings.center_x, settings.center_y
    )[2]
    result = []
    for i, line in enumerate(strokes):
        vertices = []
        for j, piece in enumerate(line):
            start, ends = piece_vertices(
                piece, settings.step_mm, settings.tolerance_mm / 10
            )
            part = f"SH_{i}_{j}"
            if not vertices:
                vertices.append((start, part))
            for p in ends:
                vertices.append((p, part))

        # [EN] All supplied motifs are closed outlines; snap final vertex to initial
        #      vertex to avoid a floating-point seam gap.
        # [VI] Tất cả các mẫu hình học có sẵn đều khép kín; gán tọa độ điểm cuối trùng
        #      khít điểm đầu để tránh khe hở do sai số dấu phẩy động.
        vertices[-1] = (vertices[0][0], vertices[-1][1])
        result.append(tuple(vertices))
    return tuple(result)


def box_distance_squared(p, box):
    """
    [EN] Lower-bound squared Euclidean distance from 2D point `p = (px, py)` to
         axis-aligned bounding box `box = (xmin, ymin, xmax, ymax)`.
         If `p` is inside the box, both `dx` and `dy` are 0, so distance is 0.
    [VI] Khoảng cách bình phương ngắn nhất từ điểm 2D `p = (px, py)` tới hộp bao
         chữ nhật `box = (xmin, ymin, xmax, ymax)`.
         Nếu `p` nằm trong hộp bao, cả `dx` và `dy` đều bằng 0 nên khoảng cách bằng 0.
    """
    px, py = p
    xmin, ymin, xmax, ymax = box

    if px < xmin:
        dx = xmin - px
    elif px > xmax:
        dx = px - xmax
    else:
        dx = 0

    if py < ymin:
        dy = ymin - py
    elif py > ymax:
        dy = py - ymax
    else:
        dy = 0

    return dx * dx + dy * dy


def distance_tree(segments):
    """
    [EN] Build a binary Bounding Volume Hierarchy (BVH) tree over 2D line segments.
         Each node is a 4-tuple `(box, leaf_segments, left_child, right_child)`.
         Leaves hold up to 6 segments directly; internal nodes split the list in half.
    [VI] Xây dựng cây nhị phân phân cấp hộp bao (BVH) cho tập các đoạn thẳng 2D.
         Mỗi nút là bộ 4 phần tử `(hộp_bao, các_đoạn_lá, nhánh_trái, nhánh_phải)`.
         Nút lá chứa tối đa 6 đoạn thẳng; nút trong chia đôi danh sách thành 2 nhánh.
    """
    xmin = min(min(a[0], b[0]) for a, b in segments)
    ymin = min(min(a[1], b[1]) for a, b in segments)
    xmax = max(max(a[0], b[0]) for a, b in segments)
    ymax = max(max(a[1], b[1]) for a, b in segments)
    box = (xmin, ymin, xmax, ymax)

    if len(segments) <= 6:
        return box, segments, None, None

    mid = len(segments) // 2
    left_child = distance_tree(segments[:mid])
    right_child = distance_tree(segments[mid:])
    return box, (), left_child, right_child


def tree_distance(p, node, best=math.inf):
    """
    [EN] Branch-and-bound nearest-segment squared distance query on a BVH `node`:
         1. Prune this subtree immediately if its bounding box is farther than `best`.
         2. At a leaf node, check all segments in the leaf and update `best`.
         3. At an internal node, visit the closer child box first to tighten `best`
            before visiting the farther child box.
    [VI] Truy vấn khoảng cách bình phương ngắn nhất trên cây BVH bằng nhánh-và-cận:
         1. Cắt tỉa ngay nhánh này nếu hộp bao của nó xa hơn kỷ lục `best` hiện tại.
         2. Nếu là nút lá, tính khoảng cách tới từng đoạn thẳng trong lá để cập nhật `best`.
         3. Nếu là nút trong, ưu tiên duyệt nhánh con có hộp bao gần điểm `p` hơn trước
            để hạ thấp ngưỡng `best`, giúp cắt tỉa nhánh còn lại hiệu quả hơn.
    """
    box, segments, left, right = node

    # [EN] Pruning check: if the point is farther from the bounding box than `best`, skip.
    # [VI] Cắt tỉa: nếu khoảng cách tới hộp bao lớn hơn `best` hiện có thì bỏ qua ngay.
    if box_distance_squared(p, box) > best:
        return best

    # [EN] Leaf node: evaluate exact squared distance to each segment in the leaf.
    # [VI] Nút lá: tính khoảng cách bình phương chính xác tới từng đoạn thẳng trong lá.
    if segments:
        for a, b in segments:
            dist_sq = line_distance_squared(p, a, b)
            if dist_sq < best:
                best = dist_sq
        return best

    # [EN] Internal node: visit the closer child first.
    # [VI] Nút trong: ưu tiên duyệt nhánh con nào có hộp bao gần điểm `p` hơn trước.
    if box_distance_squared(p, left[0]) > box_distance_squared(p, right[0]):
        left, right = right, left

    best = tree_distance(p, left, best)
    return tree_distance(p, right, best)


@lru_cache(maxsize=64)
def curve_oracle(piece, error):
    """
    [EN] Cached ultra-fine BVH reference tree for a cubic Bézier `piece`.
         Flattens the cubic curve with a tiny step (0.15 mm) and flatness `error`,
         then indexes all resulting chords in a BVH tree for fast distance queries.
    [VI] Cây BVH tham chiếu độ mịn cao (có lưu bộ nhớ đệm) cho đường cong Bézier bậc 3.
         Chia nhỏ đường cong với bước siêu mịn (0.15 mm) và độ phẳng `error`, sau đó
         đưa toàn bộ các đoạn dây cung vào cây BVH để truy vấn khoảng cách cực nhanh.
    """
    points = (piece.data[0], *flatten_cubic(piece.data, 0.15, error))
    segments = tuple(zip(points, points[1:]))
    return distance_tree(segments)


def shape_ink_distance(xy, part, settings):
    """
    [EN] Return an upper bound on the Euclidean distance (mm) from point `xy`
         to the ideal mathematical curve identified by `part` (`"SH_i_j"`).
    [VI] Trả về cận trên khoảng cách Euclid (mm) từ điểm `xy` tới đường cong
         lý tưởng được định danh bởi `part` (`"SH_i_j"`).
    """
    _, i, j = part.split("_")
    scaled_strokes = scaled_pieces(
        settings.pattern, settings.size_mm, settings.center_x, settings.center_y
    )[2]
    piece = scaled_strokes[int(i)][int(j)]

    if piece.kind == "line":
        start_pt, end_pt = piece.data
        return math.sqrt(line_distance_squared(xy, start_pt, end_pt))

    if piece.kind == "ellipse":
        cx, cy, rx, ry = piece.data
        return ellipse_radial_error(xy, cx, cy, rx, ry)

    error = settings.tolerance_mm / 100
    # [EN] Fine reference polyline's Hausdorff bound is added conservatively.
    # [VI] Cộng thêm cận Hausdorff của đường gãy khúc tham chiếu siêu mịn để đảm bảo an toàn.
    oracle_tree = curve_oracle(piece, error)
    return math.sqrt(tree_distance(xy, oracle_tree)) + error
