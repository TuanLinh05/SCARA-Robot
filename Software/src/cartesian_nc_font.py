"""
============================================================================
FILE: cartesian_nc_font.py
============================================================================
[EN] Single-Line Stroke Vector Font (A-Z, 0-9) & Parametric Pattern Generators
     Defines normalized 2D glyph strokes in a `[0..0.7] x [0..1.0]` character cell,
     provides exact `word_geometry` for built-in words ("LINH", "THOA"), and
     supports arbitrary custom text strings and parametric test patterns with
     optional rotation for the upgraded Drawing Studio.

[VI] Phông Chữ Nét Đơn Vector (A-Z, 0-9) & Bộ Sinh Mẫu Tham Số Tùy Biến
     Định nghĩa các nét chữ cái chuẩn hóa trong ô `[0..0.7] x [0..1.0]`, cung cấp
     hàm `word_geometry` cho các mẫu tích hợp ("LINH", "THOA"), đồng thời hỗ trợ
     sinh nét cho văn bản tùy ý và các mẫu kiểm thử tham số có góc xoay.
============================================================================
"""

import math


def _arc_points(cx, cy, rx, ry, a0_deg, a1_deg, steps=24):
    """
    [EN] Generate a tuple of `(x, y)` points along an elliptical arc centered at `(cx, cy)`
         with radii `(rx, ry)` from `a0_deg` to `a1_deg` (in degrees) using `steps` chords.
    [VI] Sinh chuỗi điểm `(x, y)` dọc theo cung elip tâm `(cx, cy)`, bán kính `(rx, ry)`
         từ góc `a0_deg` đến `a1_deg` (đơn vị độ) được chia đều thành `steps` đoạn.
    """
    # [EN] Step 1: Convert start and end angles from degrees to radians.
    # [VI] Bước 1: Đổi góc bắt đầu và góc kết thúc từ độ sang radian.
    a0 = math.radians(a0_deg)
    a1 = math.radians(a1_deg)

    # [EN] Step 2: Sample `steps + 1` points evenly along the angular sweep [a0, a1].
    # [VI] Bước 2: Lấy mẫu đều `steps + 1` điểm dọc theo khoảng góc quét [a0, a1].
    points = []
    for i in range(steps + 1):
        angle = a0 + (a1 - a0) * i / steps
        px = cx + rx * math.cos(angle)
        py = cy + ry * math.sin(angle)
        points.append((px, py))

    return tuple(points)


# [EN] Pre-compute the 48-segment closed oval stroke for glyph "O" (kept bit-for-bit
#      identical for regression compatibility with built-in word "THOA").
# [VI] Tính trước nét elip khép kín 48 đoạn cho chữ cái "O" (giữ nguyên từng bit
#      để tương thích kiểm thử hồi quy với mẫu chữ tích hợp "THOA").
_GLYPH_O_POINTS = []
for _i in range(49):
    _angle = math.pi / 2 - 2 * math.pi * _i / 48
    _ox = 0.35 + 0.35 * math.cos(_angle)
    _oy = 0.5 + 0.5 * math.sin(_angle)
    _GLYPH_O_POINTS.append((_ox, _oy))
_GLYPH_O_STROKE = tuple(_GLYPH_O_POINTS)


# [EN] Normalized glyph polylines: each character is a tuple of strokes, where
#      each stroke is a tuple of `(x, y)` coordinates in `[0..0.7] x [0..1.0]`.
#      Note: L, I, N, H, T, O, A are kept bit-for-bit identical for regression tests.
# [VI] Tọa độ nét chữ chuẩn hóa: mỗi ký tự gồm một hoặc nhiều nét rời (strokes),
#      mỗi nét là danh sách các điểm `(x, y)` trong khung `[0..0.7] x [0..1.0]`.
GLYPHS = {
    "A": (((0, 0), (0.35, 1), (0.7, 0)), ((0.14, 0.4), (0.56, 0.4))),
    "B": (
        ((0, 0), (0, 1)),
        ((0, 1), (0.45, 1), (0.65, 0.87), (0.65, 0.63), (0.45, 0.5), (0, 0.5)),
        ((0, 0.5), (0.48, 0.5), (0.7, 0.37), (0.7, 0.13), (0.48, 0), (0, 0)),
    ),
    "C": (_arc_points(0.38, 0.5, 0.36, 0.5, 45, 315, 28),),
    "D": (
        ((0, 0), (0, 1)),
        ((0, 1), (0.38, 1), (0.62, 0.82), (0.7, 0.5), (0.62, 0.18), (0.38, 0), (0, 0)),
    ),
    "E": (((0.7, 1), (0, 1), (0, 0), (0.7, 0)), ((0, 0.5), (0.55, 0.5))),
    "F": (((0, 0), (0, 1), (0.7, 1)), ((0, 0.5), (0.55, 0.5))),
    "G": (
        _arc_points(0.38, 0.5, 0.36, 0.5, 45, 330, 28)
        + ((0.69, 0.25), (0.69, 0.48), (0.42, 0.48)),
    ),
    "H": (((0, 0), (0, 1)), ((0, 0.5), (0.7, 0.5)), ((0.7, 0), (0.7, 1))),
    "I": (((0, 1), (0.7, 1)), ((0.35, 1), (0.35, 0)), ((0, 0), (0.7, 0))),
    "J": (((0.1, 1), (0.7, 1)), ((0.55, 1), (0.55, 0.25), (0.4, 0), (0.15, 0), (0, 0.2))),
    "K": (((0, 0), (0, 1)), ((0.7, 1), (0, 0.45), (0.7, 0))),
    "L": (((0, 1), (0, 0), (0.7, 0)),),
    "M": (((0, 0), (0, 1), (0.35, 0.35), (0.7, 1), (0.7, 0)),),
    "N": (((0, 0), (0, 1), (0.7, 0), (0.7, 1)),),
    "O": (_GLYPH_O_STROKE,),
    "P": (
        ((0, 0), (0, 1), (0.48, 1), (0.7, 0.85), (0.7, 0.62), (0.48, 0.48), (0, 0.48)),
    ),
    "Q": (
        _arc_points(0.35, 0.5, 0.35, 0.5, 90, -270, 32),
        ((0.42, 0.25), (0.72, -0.05)),
    ),
    "R": (
        ((0, 0), (0, 1), (0.48, 1), (0.7, 0.85), (0.7, 0.62), (0.48, 0.48), (0, 0.48)),
        ((0.38, 0.48), (0.7, 0)),
    ),
    "S": (
        _arc_points(0.35, 0.74, 0.32, 0.25, 30, 265, 18)
        + _arc_points(0.35, 0.26, 0.34, 0.25, 85, -150, 18),
    ),
    "T": (((0, 1), (0.7, 1)), ((0.35, 1), (0.35, 0))),
    "U": (
        ((0, 1), (0, 0.28))
        + _arc_points(0.35, 0.28, 0.35, 0.28, 180, 360, 16)
        + ((0.7, 0.28), (0.7, 1)),
    ),
    "V": (((0, 1), (0.35, 0), (0.7, 1)),),
    "W": (((0, 1), (0.18, 0), (0.35, 0.55), (0.52, 0), (0.7, 1)),),
    "X": (((0, 1), (0.7, 0)), ((0, 0), (0.7, 1))),
    "Y": (((0, 1), (0.35, 0.5), (0.7, 1)), ((0.35, 0.5), (0.35, 0))),
    "Z": (((0, 1), (0.7, 1), (0, 0), (0.7, 0)),),
    "0": (
        _arc_points(0.35, 0.5, 0.35, 0.5, 90, -270, 32),
        ((0.12, 0.18), (0.58, 0.82)),
    ),
    "1": (((0.15, 0.8), (0.35, 1), (0.35, 0)), ((0.12, 0), (0.58, 0))),
    "2": (
        _arc_points(0.35, 0.72, 0.32, 0.28, 160, -35, 18)
        + ((0, 0), (0.7, 0)),
    ),
    "3": (
        _arc_points(0.34, 0.74, 0.31, 0.25, 150, -90, 16),
        _arc_points(0.35, 0.26, 0.34, 0.26, 90, -150, 16),
    ),
    "4": (((0.52, 0), (0.52, 1), (0, 0.32), (0.7, 0.32)),),
    "5": (
        ((0.68, 1), (0.05, 1), (0.05, 0.54), (0.38, 0.58))
        + _arc_points(0.36, 0.28, 0.34, 0.28, 70, -150, 18),
    ),
    "6": (
        ((0.58, 0.92), (0.28, 1), (0.04, 0.72), (0.02, 0.28))
        + _arc_points(0.35, 0.28, 0.33, 0.27, 180, 540, 24),
    ),
    "7": (((0, 1), (0.7, 1), (0.25, 0)),),
    "8": (
        _arc_points(0.35, 0.75, 0.30, 0.24, 90, -270, 22),
        _arc_points(0.35, 0.26, 0.34, 0.26, 90, -270, 22),
    ),
    "9": (
        _arc_points(0.35, 0.72, 0.33, 0.27, 0, 360, 24)
        + ((0.68, 0.72), (0.64, 0.25), (0.38, 0), (0.12, 0.05)),
    ),
    "-": (((0.1, 0.5), (0.6, 0.5)),),
    "+": (((0.35, 0.2), (0.35, 0.8)), ((0.08, 0.5), (0.62, 0.5))),
    ".": (((0.32, 0.0), (0.38, 0.0), (0.38, 0.06), (0.32, 0.06), (0.32, 0.0)),),
    "/": (((0.05, 0), (0.65, 1)),),
}


def word_geometry(word, width, center_x, center_y):
    """
    [EN] Compute scaled dimensions `(width, height)` and 2D polylines for `word`
         ("LINH" or "THOA") centered at `(center_x, center_y)` with total width `width` mm.
         Layout math:
         - Each character is 0.7 units wide with a 0.3-unit inter-character gap
           (so character `index` starts at `x_offset = index * 1.0`).
         - Total normalized width of `N` characters is `units = (N - 1) * 1.0 + 0.7`.
         - `scale = width / units` is both the mm-per-unit factor and the character height!

    [VI] Tính kích thước `(width, height)` và danh sách các nét vẽ 2D cho từ `word`
         ("LINH" hoặc "THOA") đặt tâm tại `(center_x, center_y)` với tổng chiều rộng `width` mm.
         Công thức bố cục:
         - Mỗi ký tự rộng 0.7 đơn vị, cách nhau 0.3 đơn vị (ký tự thứ `index` bắt đầu tại `index * 1.0`).
         - Tổng chiều rộng chuẩn hóa của `N` ký tự là `units = (N - 1) * 1.0 + 0.7`.
         - `scale = width / units` vừa là hệ số phóng đại (mm/đơn vị) vừa là chiều cao chữ!
    """
    if word not in ("LINH", "THOA"):
        raise ValueError("Mẫu chữ hỗ trợ LINH hoặc THOA.")

    # [EN] Step 1: Compute unit span, scale factor (height in mm), and bottom-left corner.
    # [VI] Bước 1: Tính tổng số đơn vị ngang, hệ số tỷ lệ (chiều cao chữ mm) và góc dưới-trái.
    units = len(word) - 1 + 0.7
    scale = width / units
    left = center_x - width / 2
    bottom = center_y - scale / 2

    # [EN] Step 2: Translate and scale each character's normalized strokes into world XY (mm).
    # [VI] Bước 2: Dịch chuyển và phóng tỷ lệ từng nét chữ chuẩn hóa sang tọa độ thực XY (mm).
    strokes = []
    for index, letter in enumerate(word):
        for polyline in GLYPHS[letter]:
            scaled_points = []
            for x, y in polyline:
                px = left + (index + x) * scale
                py = bottom + y * scale
                scaled_points.append((px, py))
            strokes.append(tuple(scaled_points))

    return width, scale, tuple(strokes)


def _rotate_xy(x, y, rad):
    """
    [EN] Rotate 2D point `(x, y)` counterclockwise around the origin `(0, 0)` by `rad` radians:
             x' = x * cos(rad) - y * sin(rad)
             y' = x * sin(rad) + y * cos(rad)
    [VI] Xoay điểm 2D `(x, y)` ngược chiều kim đồng hồ quanh gốc `(0, 0)` một góc `rad` radian:
             x' = x * cos(rad) - y * sin(rad)
             y' = x * sin(rad) + y * cos(rad)
    """
    if abs(rad) < 1e-9:
        return (x, y)
    c = math.cos(rad)
    s = math.sin(rad)
    rotated_x = x * c - y * s
    rotated_y = x * s + y * c
    return (rotated_x, rotated_y)


def custom_text_strokes(text, width_mm=40.0, rotation_deg=0.0):
    """
    [EN] Compile an arbitrary text string (`A-Z`, `0-9`, spaces, `-`, `+`, `.`, `/`)
         into a tuple of 2D polyline strokes centered at `(0, 0)` with optional
         rotation `rotation_deg`, ready for `register_custom_pattern`.
    [VI] Biên dịch chuỗi văn bản tùy ý (`A-Z`, `0-9`, khoảng trắng, `-`, `+`, `.`, `/`)
         thành danh sách các nét vẽ 2D căn tâm tại `(0, 0)` kèm góc xoay `rotation_deg`,
         sẵn sàng đăng ký qua `register_custom_pattern`.
    """
    # [EN] Step 1: Normalize and validate the input text string.
    # [VI] Bước 1: Chuẩn hóa chữ in hoa và kiểm tra tính hợp lệ của chuỗi ký tự.
    cleaned = (text or "").strip().upper()
    if not cleaned:
        raise ValueError("Vui lòng nhập chuỗi chữ/số cần vẽ (A-Z, 0-9).")

    has_non_space = False
    for ch in cleaned:
        if ch != " ":
            has_non_space = True
            if ch not in GLYPHS:
                raise ValueError(
                    f"Ký tự '{ch}' chưa hỗ trợ. Chỉ hỗ trợ A-Z, 0-9, khoảng trắng và +-./"
                )
    if not has_non_space:
        raise ValueError("Chuỗi văn bản cần có ít nhất một chữ cái hoặc chữ số.")

    # [EN] Step 2: Compute bounding box centered at (0, 0) and rotation angle in radians.
    # [VI] Bước 2: Tính khung bao căn tâm tại (0, 0) và đổi góc xoay sang radian.
    units = max(0.7, len(cleaned) - 1 + 0.7)
    scale = max(5.0, float(width_mm)) / units
    left = -units * scale / 2.0
    bottom = -scale / 2.0
    rad = math.radians(float(rotation_deg))

    # [EN] Step 3: Transform every glyph stroke by scaling, centering, and rotating.
    # [VI] Bước 3: Biến đổi từng nét chữ bằng cách phóng tỷ lệ, căn tâm và xoay góc.
    strokes = []
    for index, ch in enumerate(cleaned):
        if ch == " ":
            continue
        for polyline in GLYPHS[ch]:
            pts = []
            for x, y in polyline:
                px = left + (index + x) * scale
                py = bottom + y * scale
                pts.append(_rotate_xy(px, py, rad))
            if len(pts) >= 2:
                strokes.append(tuple(pts))

    return tuple(strokes)


def parametric_generator_strokes(kind, param=6, width_mm=30.0, rotation_deg=0.0):
    """
    [EN] Generate custom parametric geometric strokes for advanced testing & calibration:
         - `"SPIRAL"`: Archimedes spiral with `param` turns.
         - `"GRID"`: Calibration crosshair & concentric squares (`param` rings).
         - `"ROSE"`: Mathematical rhodonea / multi-petal flower (`param` petals).
         - `"POLYGON"`: Regular N-sided polygon with inscribed star (`param` sides).
    [VI] Sinh các mẫu hình học tham số tùy biến phục vụ kiểm thử và hiệu chuẩn nâng cao:
         - `"SPIRAL"`: Đường xoắn ốc Archimedes với `param` vòng xoắn.
         - `"GRID"`: Lưới tâm ngắm & ô vuông đồng tâm hiệu chuẩn độ chính xác (`param` bậc).
         - `"ROSE"`: Đường cong hoa hồng toán học (`param` cánh).
         - `"POLYGON"`: Đa giác đều N cạnh kèm sao nội tiếp (`param` cạnh).
    """
    r_max = max(4.0, float(width_mm) / 2.0)
    rad = math.radians(float(rotation_deg))
    n = max(2, min(24, int(round(float(param)))))
    strokes = []

    if kind == "SPIRAL":
        # [EN] Archimedes spiral: radius grows linearly with angle from 8% to 100% of r_max.
        # [VI] Đường xoắn ốc Archimedes: bán kính tăng tuyến tính theo góc từ 8% đến 100% r_max.
        turns = max(1, min(10, n))
        steps = turns * 48
        pts = []
        for i in range(steps + 1):
            u = i / steps
            angle = turns * 2.0 * math.pi * u
            r = r_max * (0.08 + 0.92 * u)
            px = r * math.cos(angle)
            py = r * math.sin(angle)
            pts.append(_rotate_xy(px, py, rad))
        strokes.append(tuple(pts))

    elif kind == "GRID":
        # [EN] Concentric squares plus horizontal and vertical center crosshair lines.
        # [VI] Các hình vuông đồng tâm kết hợp với hai đường chữ thập ngang-dọc qua tâm.
        rings = max(1, min(5, n))
        for k in range(1, rings + 1):
            half = r_max * k / rings
            square_corners = (
                (-half, half),
                (half, half),
                (half, -half),
                (-half, -half),
                (-half, half),
            )
            rotated_square = []
            for x, y in square_corners:
                rotated_square.append(_rotate_xy(x, y, rad))
            strokes.append(tuple(rotated_square))
        strokes.append((_rotate_xy(-r_max, 0, rad), _rotate_xy(r_max, 0, rad)))
        strokes.append((_rotate_xy(0, -r_max, rad), _rotate_xy(0, r_max, rad)))

    elif kind == "ROSE":
        # [EN] Polar rose curve: r(theta) = r_max * (0.22 + 0.78 * |cos(petals * theta / 2)|).
        # [VI] Đường hoa hồng cực: r(theta) = r_max * (0.22 + 0.78 * |cos(petals * theta / 2)|).
        petals = max(3, min(16, n))
        steps = petals * 36
        pts = []
        for i in range(steps + 1):
            theta = 2.0 * math.pi * i / steps
            r = r_max * (0.22 + 0.78 * abs(math.cos(petals * theta / 2.0)))
            px = r * math.cos(theta)
            py = r * math.sin(theta)
            pts.append(_rotate_xy(px, py, rad))
        pts[-1] = pts[0]
        strokes.append(tuple(pts))

    else:  # POLYGON
        # [EN] Outer regular N-sided polygon + inner alternating N-pointed star.
        # [VI] Đa giác đều N cạnh bên ngoài kết hợp ngôi sao N cánh nội tiếp bên trong.
        sides = max(3, min(16, n))
        outer = []
        for i in range(sides):
            ang = math.pi / 2 + 2 * math.pi * i / sides
            px = r_max * math.cos(ang)
            py = r_max * math.sin(ang)
            outer.append(_rotate_xy(px, py, rad))
        outer.append(outer[0])
        strokes.append(tuple(outer))

        inner = []
        for i in range(sides * 2):
            r = r_max * (0.85 if i % 2 == 0 else 0.38)
            ang = math.pi / 2 + math.pi * i / sides
            px = r * math.cos(ang)
            py = r * math.sin(ang)
            inner.append(_rotate_xy(px, py, rad))
        inner.append(inner[0])
        strokes.append(tuple(inner))

    return tuple(strokes)
