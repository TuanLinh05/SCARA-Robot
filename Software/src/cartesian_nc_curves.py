"""
============================================================================
FILE: cartesian_nc_curves.py
============================================================================
[EN] Parametric Curve Flattening (Cubic Bézier & Elliptical Arcs)
     Subdivides smooth mathematical curves on the PC into short, bounded straight
     line chords (`max_step`, `max_error`) using recursive de Casteljau subdivision.
     This keeps the STM32 microcontroller's real-time ISR purely integer-based.

[VI] Xấp xỉ Đường cong Tham số (Đường cong Bézier Bậc 3 & Cung Ellipse)
     Chia nhỏ các đường cong toán học mượt trên máy tính thành chuỗi các đoạn thẳng
     ngắn có kiểm soát độ dài (`max_step`) và sai số dây cung (`max_error`) bằng
     thuật toán đệ quy de Casteljau. Nhờ đó vi điều khiển STM32 chỉ cần chạy nội
     suy đường thẳng nguyên (DDA) mà vẫn vẽ được đường cong mượt.

[DEBUG TIP / MẸO GỠ LỖI]:
- `flatten_cubic()` returns ONLY the subdivided end vertices `p[3]` of each piece
  (excluding the initial start point `control[0]`) so callers can chain multiple
  curves together without duplicating junction points!
- Hàm `flatten_cubic()` chỉ trả về danh sách các đỉnh cuối của từng đoạn chia nhỏ
  (không bao gồm điểm đầu `control[0]`) để khi ghép nối tiếp nhiều đường cong sẽ
  không bị trùng lặp điểm nối!
============================================================================
"""

import math


def distance_to_line(p, a, b):
    """
    [EN] Compute perpendicular distance from 2D point `p` to the infinite line passing
         through `a` and `b` (or Euclidean distance to `a` if `a == b`).
         Formula:
         Using the 2D cross product magnitude |(b - a) x (a - p)|, which equals twice
         the area of triangle PAB, divided by the base length |b - a|.

    [VI] Tính khoảng cách vuông góc từ điểm 2D `p` tới đường thẳng đi qua 2 điểm `a`, `b`
         (hoặc khoảng cách tới `a` nếu `a` và `b` trùng nhau).
         Công thức:
         Sử dụng độ lớn tích có hướng 2D |(b - a) x (a - p)| (bằng 2 lần diện tích
         tam giác PAB) chia cho độ dài cạnh đáy |b - a|.
    """
    # [EN] Step 1: Direction vector (dx, dy) of line segment AB and its length.
    # [VI] Bước 1: Vector chỉ phương (dx, dy) của đoạn thẳng AB và độ dài của nó.
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    length = math.hypot(dx, dy)

    # [EN] Step 2: If A and B are distinct, divide parallelogram area by base length.
    # [VI] Bước 2: Nếu A và B phân biệt, lấy diện tích hình bình hành chia cho cạnh đáy.
    if length:
        cross_area = abs(dx * (a[1] - p[1]) - (a[0] - p[0]) * dy)
        return cross_area / length

    # [EN] Degenerate case: A and B coincide, return direct point-to-point distance.
    # [VI] Trường hợp suy biến: A và B trùng nhau, trả về khoảng cách giữa hai điểm.
    return math.dist(p, a)


def flatten_cubic(control, max_step, max_error, cancelled=lambda: False):
    """
    [EN] Recursively subdivide a cubic Bézier curve `control = (P0, P1, P2, P3)`
         using de Casteljau's algorithm until:
         1. The control polygon length `||P0P1|| + ||P1P2|| + ||P2P3|| <= max_step`, AND
         2. Both interior control points `P1, P2` lie within `max_error` of chord `P0P3`.
         Returns a tuple of endpoint `(x, y)` vertices (excluding `P0`).

    [VI] Chia đôi đệ quy đường cong Bézier bậc 3 `control = (P0, P1, P2, P3)` bằng
         thuật toán de Casteljau cho tới khi:
         1. Tổng độ dài đa giác điều khiển `||P0P1|| + ||P1P2|| + ||P2P3|| <= max_step`, VÀ
         2. Khoảng cách từ 2 điểm điều khiển `P1, P2` tới dây cung `P0P3` `<= max_error`.
         Trả về tuple các điểm đỉnh `(x, y)` (không lặp lại điểm xuất phát `P0`).
    """
    output = []

    def midpoint(pt1, pt2):
        """
        [EN] Compute the 2D midpoint ((x1 + x2)/2, (y1 + y2)/2) of two points.
        [VI] Tính trung điểm 2D ((x1 + x2)/2, (y1 + y2)/2) của hai điểm.
        """
        mid_x = (pt1[0] + pt2[0]) / 2
        mid_y = (pt1[1] + pt2[1]) / 2
        return (mid_x, mid_y)

    def visit(p, depth):
        # [EN] Allow the user or UI thread to cancel a long curve tessellation.
        # [VI] Cho phép người dùng hoặc luồng giao diện hủy tác vụ chia đường cong.
        if cancelled():
            raise ValueError("Tác vụ đường cong đã hủy.")

        # [EN] Unpack the 4 cubic Bézier control points: start P0, handles P1 & P2, end P3.
        # [VI] Giải nén 4 điểm điều khiển Bézier bậc 3: điểm đầu P0, điểm uốn P1 & P2, điểm cuối P3.
        p0 = p[0]
        p1 = p[1]
        p2 = p[2]
        p3 = p[3]

        # [EN] Step 1: Compute control polygon perimeter ||P0P1|| + ||P1P2|| + ||P2P3||.
        #      By the convex hull property of Bézier curves, the true arc length is
        #      always less than or equal to this control polygon perimeter.
        # [VI] Bước 1: Tính tổng chiều dài 3 đoạn của đa giác điều khiển.
        #      Theo tính chất bao lồi của đường cong Bézier, chiều dài cung thực tế
        #      luôn nhỏ hơn hoặc bằng tổng chiều dài đa giác điều khiển này.
        len_01 = math.dist(p0, p1)
        len_12 = math.dist(p1, p2)
        len_23 = math.dist(p2, p3)
        polygon = len_01 + len_12 + len_23

        # [EN] Step 2: Compute flatness (maximum deviation of handles P1, P2 from chord P0->P3).
        # [VI] Bước 2: Tính độ lệch phẳng lớn nhất từ hai điểm điều khiển P1, P2 tới dây cung P0->P3.
        dev_p1 = distance_to_line(p1, p0, p3)
        dev_p2 = distance_to_line(p2, p0, p3)
        flat = max(dev_p1, dev_p2)

        # [EN] Base case: if the piece is both short enough and flat enough, accept chord P0->P3.
        # [VI] Điều kiện dừng: nếu đoạn cong đã đủ ngắn và đủ phẳng, ghi nhận đỉnh cuối P3.
        if polygon <= max_step and flat <= max_error:
            output.append(p3)
            return

        # [EN] Guard against infinite recursion or excessive memory usage.
        # [VI] Bảo vệ chống đệ quy vô tận hoặc vượt quá giới hạn số đoạn cho phép.
        if depth >= 18 or len(output) >= 4096:
            raise ValueError("Đường cong vượt ngân sách chia đoạn.")

        # [EN] Step 3: de Casteljau midpoint pyramid splitting the cubic curve at t = 0.5:
        #      Level 1 (edge midpoints):     a = mid(P0,P1), b = mid(P1,P2), c = mid(P2,P3)
        #      Level 2 (segment midpoints):  d = mid(a,b),   e = mid(b,c)
        #      Level 3 (curve split point):  f = mid(d,e) -> exact midpoint on curve B(0.5)!
        # [VI] Bước 3: Tháp trung điểm de Casteljau cắt đôi đường cong tại t = 0.5:
        #      Tầng 1 (trung điểm cạnh):     a = mid(P0,P1), b = mid(P1,P2), c = mid(P2,P3)
        #      Tầng 2 (trung điểm nối):      d = mid(a,b),   e = mid(b,c)
        #      Tầng 3 (điểm chia trên cung): f = mid(d,e) -> chính là điểm giữa B(0.5)!
        a = midpoint(p0, p1)
        b = midpoint(p1, p2)
        c = midpoint(p2, p3)
        d = midpoint(a, b)
        e = midpoint(b, c)
        f = midpoint(d, e)

        # [EN] Recursively flatten the left half (P0, a, d, f) then the right half (f, e, c, P3).
        # [VI] Đệ quy làm phẳng nửa trái (P0, a, d, f) rồi tiếp tục nửa phải (f, e, c, P3).
        visit((p0, a, d, f), depth + 1)
        visit((f, e, c, p3), depth + 1)

    visit(tuple(control), 0)
    return tuple(output)


def ellipse_arc(cx, cy, rx, ry, start, end, step, error):
    """
    [EN] Approximate an elliptical arc centered at `(cx, cy)` with semi-axes `(rx, ry)`
         from angle `start` to `end` (in radians) using cubic Bézier segments of <= 90 deg,
         then flatten each segment within `(step, error)`.
         Parametric ellipse equation:
             E(theta)  = (cx + rx * cos(theta), cy + ry * sin(theta))
             E'(theta) = (-rx * sin(theta),     ry * cos(theta))

    [VI] Xấp xỉ cung ellipse tâm `(cx, cy)` bán kính `(rx, ry)` từ góc `start` đến `end`
         (radian) bằng các mảnh Bézier bậc 3 góc <= 90 độ, sau đó làm phẳng theo `(step, error)`.
         Phương trình tham số của ellipse:
             E(theta)  = (cx + rx * cos(theta), cy + ry * sin(theta))
             E'(theta) = (-rx * sin(theta),     ry * cos(theta))
    """
    # [EN] Step 1: Split total sweep angle into segments of at most 90 degrees (pi / 2 rad).
    # [VI] Bước 1: Chia tổng góc quét thành các phần nhỏ tối đa 90 độ (pi / 2 rad).
    sweep_angle = abs(end - start)
    quarter_turn = math.pi / 2
    pieces = max(1, math.ceil(sweep_angle / quarter_turn))
    output = []

    # [EN] Step 2: Construct and flatten a cubic Bézier curve for each angular sub-arc [a, b].
    # [VI] Bước 2: Dựng và làm phẳng đường cong Bézier bậc 3 cho từng cung góc con [a, b].
    for i in range(pieces):
        a = start + (end - start) * i / pieces
        b = start + (end - start) * (i + 1) / pieces

        # [EN] Optimal cubic handle factor k = (4/3) * tan(delta_theta / 4) matches
        #      endpoints, tangents, and midpoint of the circular/elliptical arc.
        # [VI] Hệ số tiếp tuyến tối ưu k = (4/3) * tan(góc_chắn / 4) khớp chính xác
        #      hai đầu mút, tiếp tuyến và điểm giữa của cung tròn/ellipse.
        k = (4 / 3) * math.tan((b - a) / 4)

        # [EN] Endpoints P0 = E(a) and P3 = E(b) on the ellipse.
        # [VI] Hai điểm đầu mút P0 = E(a) và P3 = E(b) nằm trên đường ellipse.
        p0 = (cx + rx * math.cos(a), cy + ry * math.sin(a))
        p3 = (cx + rx * math.cos(b), cy + ry * math.sin(b))

        # [EN] Control points P1 = P0 + k * E'(a) and P2 = P3 - k * E'(b).
        # [VI] Hai điểm điều khiển tiếp tuyến P1 = P0 + k * E'(a) và P2 = P3 - k * E'(b).
        p1 = (p0[0] - k * rx * math.sin(a), p0[1] + k * ry * math.cos(a))
        p2 = (p3[0] + k * rx * math.sin(b), p3[1] - k * ry * math.cos(b))

        # [EN] Subdivide the cubic segment into straight chords and append vertices.
        # [VI] Chia nhỏ mảnh Bézier bậc 3 thành các dây cung thẳng và nối vào kết quả.
        subdivided_points = flatten_cubic((p0, p1, p2, p3), step, error)
        output.extend(subdivided_points)

    return tuple(output)


def ellipse_radial_error(xy, cx, cy, rx, ry):
    """
    [EN] Compute radial distance from point `xy` to the ellipse `(cx, cy, rx, ry)`
         along the ray from the center. Serves as an upper bound on shortest distance
         during ink preflight validation.
         Method:
         1. Offset point relative to center: (dx, dy) = (x - cx, y - cy).
         2. Normalize into unit-circle coordinates: (dx / rx, dy / ry), whose norm
            `radius` equals 1.0 when `xy` lies strictly on the ellipse.
         3. Scale (dx, dy) by `1 / radius` to find the intersection point on the
            ellipse along the same ray from center, then return the distance.

    [VI] Tính khoảng cách xuyên tâm từ điểm `xy` tới đường ellipse `(cx, cy, rx, ry)`
         dọc theo tia nối từ tâm. Dùng làm cận trên của khoảng cách ngắn nhất khi
         kiểm tra dung sai nét vẽ (preflight).
         Phương pháp:
         1. Tính độ lệch so với tâm: (dx, dy) = (x - cx, y - cy).
         2. Chuẩn hóa về hệ tọa độ đường tròn đơn vị: (dx / rx, dy / ry); độ dài
            `radius` bằng đúng 1.0 nếu điểm `xy` nằm trên ellipse.
         3. Chia (dx, dy) cho `radius` để tìm giao điểm trên đường ellipse cùng tia,
            sau đó tính khoảng cách Euclid giữa hai điểm.
    """
    dx = xy[0] - cx
    dy = xy[1] - cy
    radius = math.hypot(dx / rx, dy / ry)

    # [EN] If the query point is exactly at the center, distance to boundary is min(rx, ry).
    # [VI] Nếu điểm truy vấn trùng đúng tâm ellipse, khoảng cách tới biên gần nhất là min(rx, ry).
    if not radius:
        return min(rx, ry)

    return math.hypot(dx - dx / radius, dy - dy / radius)
