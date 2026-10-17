"""
================================================================================
FILE: cartesian_nc_vector.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     CAD/CAM vector importer supporting external `.svg`, `.dxf`, and `.gcode`/`.nc`
     files, plus a custom pattern registry (`CUSTOM_PATTERNS`) so user-imported
     vector artwork goes through the exact same inverse-kinematics, DDA step
     simulation, and ink-tolerance preflight checks as built-in patterns.

[VI] MỤC ĐÍCH:
     Bộ nhập dữ liệu vector CAD/CAM hỗ trợ các định dạng `.svg`, `.dxf`, và
     `.gcode`/`.nc`, kết hợp với kho đăng ký mẫu tùy chỉnh (`CUSTOM_PATTERNS`)
     để các hình vẽ người dùng tải vào cũng được kiểm duyệt động học ngược,
     mô phỏng từng xung DDA và kiểm tra dung sai nét mực giống hệt mẫu có sẵn.

[DEBUG TIP / MẸO GỠ LỖI]:
     - SVG screen coordinates have `+Y` pointing DOWN, whereas robot world
       coordinates have `+Y` pointing UP. Notice the loop at the end of
       `parse_svg()` which flips `(x, -y)` so imported SVGs are not drawn
       upside-down!
     - Hệ tọa độ màn hình của SVG có trục `+Y` hướng XUỐNG, trong khi hệ tọa độ
       robot có trục `+Y` hướng LÊN. Hãy chú ý vòng lặp ở cuối hàm `parse_svg()`
       giúp lật `(x, -y)` để file SVG không bị vẽ ngược!
================================================================================
"""

from functools import lru_cache
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET
from cartesian_nc_curves import flatten_cubic, ellipse_arc
from cartesian_nc_geometry import segment_distance

# ==============================================================================
# 1. CUSTOM PATTERN REGISTRY & POLYLINE CLEANER / KHO MẪU TÙY CHỈNH & LÀM SẠCH NÉT
# ==============================================================================

# [EN] Runtime registry mapping custom pattern codes (e.g. `"CUSTOM"`) to
#      `(title, "Tùy chỉnh", default_width_mm, tuple_of_polylines)`.
# [VI] Từ điển lưu các mẫu tùy chỉnh trong phiên chạy: mã -> `(tên, "Tùy chỉnh", rộng_mm, các_nét)`.
CUSTOM_PATTERNS = {}


def _clean_polyline(pts):
    """
    [EN] Validate finiteness, convert to float tuples, remove consecutive
         duplicate points (`dist <= 1e-6`), and discard degenerate (<2 pt) strokes.
    [VI] Kiểm tra giá trị hữu hạn, ép kiểu float, loại bỏ các điểm trùng nhau liên tiếp
         (`dist <= 1e-6`) và bỏ qua các nét suy biến (ít hơn 2 điểm).
    """
    out = []
    for x, y in pts:
        # [EN] Reject NaN or Inf coordinates immediately.
        # [VI] Từ chối ngay lập tức nếu tọa độ chứa giá trị NaN hoặc vô cùng (Inf).
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError("Tọa độ vector chứa giá trị không hữu hạn.")
        pt = (float(x), float(y))

        # [EN] Only keep the point if it is distinct from the previous point.
        # [VI] Chỉ giữ lại điểm nếu nó cách điểm liền trước tối thiểu 1e-6 mm.
        if not out or math.dist(out[-1], pt) > 1e-6:
            out.append(pt)

    if len(out) >= 2:
        return tuple(out)
    return ()


# ==============================================================================
# 2. G-CODE PARSER (G0/G1/G2/G3) / BỘ ĐỌC DỮ LIỆU G-CODE
# ==============================================================================


def parse_gcode(text):
    """
    [EN] Extract 2D pen-down polylines from standard G0/G1/G2/G3 G-code text.
         Supports absolute (`G90`) and incremental (`G91`) modes, Z-height pen
         transitions (`Z <= 0.05` = pen down), and `M3`/`M4`/`M5` spindle/pen codes.
    [VI] Trích xuất các đường gãy khúc 2D khi hạ bút từ văn bản G-code G0/G1/G2/G3.
         Hỗ trợ tọa độ tuyệt đối (`G90`), tương đối (`G91`), nâng/hạ bút theo cao độ Z
         (`Z <= 0.05` là hạ bút) và mã lệnh `M3`/`M4`/`M5`.
    """
    strokes = []
    current = []
    x = 0.0
    y = 0.0
    z = 5.0
    relative = False
    pen_down = False
    token_re = re.compile(r"([A-Za-z])\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)")

    def flush():
        """[EN] Finalize current pen-down stroke / [VI] Kết thúc và lưu nét đang vẽ."""
        nonlocal current
        cleaned = _clean_polyline(current)
        if cleaned:
            strokes.append(cleaned)
        current = []

    for raw_line in text.splitlines():
        # [EN] Strip `;` and `(...)` comments from the G-code line.
        # [VI] Loại bỏ phần chú thích sau dấu `;` hoặc trong ngoặc đơn `(...)`.
        line = raw_line.split(";", 1)[0].split("(", 1)[0].strip()
        if not line:
            continue

        # [EN] Parse letter-number tokens (e.g. G1, X10.5, Y-3.2, F500).
        # [VI] Tách các cặp chữ-số trong dòng lệnh (ví dụ: G1, X10.5, Y-3.2, F500).
        words = []
        for match in token_re.finditer(line):
            letter = match.group(1).upper()
            number = float(match.group(2))
            words.append((letter, number))
        if not words:
            continue

        # [EN] Separate G-codes, M-codes, and coordinate/offset parameters (X, Y, Z, I, J).
        # [VI] Phân loại mã lệnh G, mã lệnh M và các tham số tọa độ (X, Y, Z, I, J).
        g_cmds = []
        m_cmds = []
        params = {}
        for k, v in words:
            if k == "G":
                g_cmds.append(int(round(v)))
            elif k == "M":
                m_cmds.append(int(round(v)))
            else:
                params[k] = v

        # [EN] Handle positioning mode (G90 = absolute, G91 = relative) and M3/M4/M5 pen state.
        # [VI] Xử lý chế độ tọa độ (G90 = tuyệt đối, G91 = tương đối) và lệnh bật/tắt bút M3/M4/M5.
        if 90 in g_cmds:
            relative = False
        if 91 in g_cmds:
            relative = True
        if 3 in m_cmds or 4 in m_cmds:
            if not pen_down:
                pen_down = True
                current = [(x, y)]
        if 5 in m_cmds:
            if pen_down:
                flush()
                pen_down = False

        # [EN] Compute target coordinates (nx, ny, nz) according to absolute/relative mode.
        # [VI] Tính tọa độ đích (nx, ny, nz) tùy theo chế độ tuyệt đối hay tương đối.
        if relative and "X" in params:
            nx = x + params["X"]
        else:
            nx = params.get("X", x)

        if relative and "Y" in params:
            ny = y + params["Y"]
        else:
            ny = params.get("Y", y)

        if relative and "Z" in params:
            nz = z + params["Z"]
        else:
            nz = params.get("Z", z)

        # [EN] Detect Z-axis pen-down / pen-up transitions around Z = 0.05 mm threshold.
        # [VI] Phát hiện chuyển trạng thái hạ bút / nhấc bút theo ngưỡng cao độ Z <= 0.05 mm.
        if "Z" in params:
            was_down = pen_down
            pen_down = nz <= 0.05
            if pen_down and not was_down:
                current = [(x, y)]
            elif not pen_down and was_down:
                flush()
        z = nz

        # [EN] Determine active motion mode: G0 (rapid), G1 (linear), G2 (CW arc), G3 (CCW arc).
        # [VI] Xác định lệnh chuyển động: G0 (chạy nhanh), G1 (đường thẳng), G2/G3 (cung tròn).
        motion = None
        for g in g_cmds:
            if g in (0, 1, 2, 3):
                motion = g
                break
        if motion is None and ("X" in params or "Y" in params):
            motion = 1 if pen_down else 0

        if motion == 0:
            # [EN] G0 rapid move: if XY moves without explicit Z, treat as pen-up travel.
            # [VI] G0 chạy không tải: nếu di chuyển XY mà không giữ Z thì ngắt nét đang vẽ.
            if "X" in params or "Y" in params:
                if pen_down and ("Z" not in params):
                    flush()
                    pen_down = False
                x, y = nx, ny
        elif motion == 1:
            # [EN] G1 linear interpolation: record vertex in current pen-down polyline.
            # [VI] G1 nội suy đường thẳng: thêm điểm đích vào nét vẽ hiện tại.
            if not pen_down and ("X" in params or "Y" in params) and "Z" not in params:
                pen_down = True
                current = [(x, y)]
            if pen_down and ("X" in params or "Y" in params):
                if not current:
                    current = [(x, y)]
                current.append((nx, ny))
            x, y = nx, ny
        elif motion in (2, 3):
            # [EN] G2/G3 circular arc interpolation with center offset (I, J) from current (x, y).
            # [VI] G2/G3 nội suy cung tròn với độ lệch tâm (I, J) tính từ điểm hiện tại (x, y).
            if not pen_down:
                pen_down = True
                current = [(x, y)]
            cx = x + params.get("I", 0.0)
            cy = y + params.get("J", 0.0)
            r = math.hypot(x - cx, y - cy)
            if r > 1e-6:
                a0 = math.atan2(y - cy, x - cx)
                a1 = math.atan2(ny - cy, nx - cx)
                if motion == 2:  # CW / Cùng chiều kim đồng hồ
                    if a1 >= a0 - 1e-9:
                        a1 -= 2 * math.pi
                else:  # CCW / Ngược chiều kim đồng hồ
                    if a1 <= a0 + 1e-9:
                        a1 += 2 * math.pi
                for pt in ellipse_arc(cx, cy, r, r, a0, a1, 0.5, 0.02):
                    current.append(pt)
            current.append((nx, ny))
            x, y = nx, ny

    flush()
    if not strokes:
        raise ValueError("Không tìm thấy nét vẽ hợp lệ trong dữ liệu G-code.")
    return tuple(strokes)


# ==============================================================================
# 3. ASCII DXF PARSER / BỘ ĐỌC DỮ LIỆU BẢN VẼ KỸ THUẬT DXF
# ==============================================================================


def parse_dxf(text):
    """
    [EN] Extract 2D polylines from ASCII DXF entities (`LINE`, `LWPOLYLINE`,
         `POLYLINE`, `CIRCLE`, `ARC`).
         DXF format stores data in consecutive 2-line `(group_code, value)` pairs:
         - Code 0: Entity type start (`LINE`, `CIRCLE`, `VERTEX`, `SEQEND`, ...)
         - Code 10, 20: Primary X, Y coordinates (start point or center)
         - Code 11, 21: Secondary X, Y coordinates (line endpoint)
         - Code 40: Circle/Arc radius
         - Code 50, 51: Arc start and end angles (in degrees, CCW)
         - Code 70: Polyline bitflags (bit 1 indicates a closed loop)

    [VI] Trích xuất các đường gãy khúc 2D từ các thực thể DXF dạng ASCII (`LINE`,
         `LWPOLYLINE`, `POLYLINE`, `CIRCLE`, `ARC`).
         Định dạng DXF lưu dữ liệu theo từng cặp 2 dòng `(mã_nhóm, giá_trị)`:
         - Mã 0: Bắt đầu một thực thể (`LINE`, `CIRCLE`, `VERTEX`, `SEQEND`, ...)
         - Mã 10, 20: Tọa độ X, Y chính (điểm đầu hoặc tâm)
         - Mã 11, 21: Tọa độ X, Y phụ (điểm cuối của đoạn thẳng)
         - Mã 40: Bán kính đường tròn hoặc cung tròn
         - Mã 50, 51: Góc bắt đầu và góc kết thúc của cung tròn (độ, ngược chiều kim đồng hồ)
         - Mã 70: Cờ thuộc tính đa tuyến (bit 1 biểu thị đường khép kín)
    """
    # [EN] Step 1: Group stripped lines into (int_code, string_value) pairs.
    # [VI] Bước 1: Gom các dòng văn bản thành danh sách các cặp (mã_số_nguyên, giá_trị_chuỗi).
    lines = []
    for raw_line in text.splitlines():
        lines.append(raw_line.strip())

    pairs = []
    for i in range(0, len(lines) - 1, 2):
        try:
            code = int(lines[i])
        except ValueError:
            continue
        pairs.append((code, lines[i + 1]))

    # [EN] Step 2: Scan entity records and convert each supported geometry into polylines.
    # [VI] Bước 2: Duyệt qua các bản ghi thực thể và chuyển đổi hình học thành các nét vẽ.
    strokes = []
    idx = 0
    n = len(pairs)
    while idx < n:
        code, val = pairs[idx]
        if code == 0 and val in ("LINE", "LWPOLYLINE", "CIRCLE", "ARC", "POLYLINE"):
            etype = val
            idx += 1

            # [EN] Legacy POLYLINE entity: sequence of VERTEX records terminated by SEQEND.
            # [VI] Thực thể POLYLINE kiểu cũ: chuỗi các bản ghi VERTEX kết thúc bằng SEQEND.
            if etype == "POLYLINE":
                pts = []
                closed = False
                while idx < n:
                    c, v = pairs[idx]
                    if c == 0 and v == "SEQEND":
                        idx += 1
                        break
                    if c == 70 and (int(v) & 1):
                        closed = True
                    if c == 0 and v == "VERTEX":
                        idx += 1
                        vx = 0.0
                        vy = 0.0
                        while idx < n and pairs[idx][0] != 0:
                            vc, vv = pairs[idx]
                            if vc == 10:
                                vx = float(vv)
                            elif vc == 20:
                                vy = float(vv)
                            idx += 1
                        pts.append((vx, vy))
                        continue
                    idx += 1
                if closed and pts:
                    pts.append(pts[0])
                cleaned = _clean_polyline(pts)
                if cleaned:
                    strokes.append(cleaned)
                continue

            # [EN] Collect all property (code, value) pairs belonging to this entity.
            # [VI] Thu thập tất cả các cặp thuộc tính (mã, giá trị) của thực thể hiện tại.
            props = []
            while idx < n and pairs[idx][0] != 0:
                props.append(pairs[idx])
                idx += 1

            if etype == "LINE":
                d = {}
                for c, v in props:
                    if c in (10, 20, 11, 21):
                        d[c] = float(v)
                start_pt = (d.get(10, 0.0), d.get(20, 0.0))
                end_pt = (d.get(11, 0.0), d.get(21, 0.0))
                cleaned = _clean_polyline([start_pt, end_pt])
                if cleaned:
                    strokes.append(cleaned)

            elif etype == "LWPOLYLINE":
                pts = []
                closed = False
                cur_x = None
                for c, v in props:
                    if c == 70 and (int(v) & 1):
                        closed = True
                    elif c == 10:
                        cur_x = float(v)
                    elif c == 20 and cur_x is not None:
                        pts.append((cur_x, float(v)))
                        cur_x = None
                if closed and pts:
                    pts.append(pts[0])
                cleaned = _clean_polyline(pts)
                if cleaned:
                    strokes.append(cleaned)

            elif etype == "CIRCLE":
                d = {}
                for c, v in props:
                    if c in (10, 20, 40):
                        d[c] = float(v)
                cx = d.get(10, 0.0)
                cy = d.get(20, 0.0)
                r = d.get(40, 0.0)
                if r > 1e-6:
                    arc_pts = ellipse_arc(
                        cx, cy, r, r, math.pi / 2, -3 * math.pi / 2, 0.5, 0.02
                    )
                    pts = [(cx, cy + r), *arc_pts]
                    pts[-1] = pts[0]
                    cleaned = _clean_polyline(pts)
                    if cleaned:
                        strokes.append(cleaned)

            elif etype == "ARC":
                d = {}
                for c, v in props:
                    if c in (10, 20, 40, 50, 51):
                        d[c] = float(v)
                cx = d.get(10, 0.0)
                cy = d.get(20, 0.0)
                r = d.get(40, 0.0)
                a0 = math.radians(d.get(50, 0.0))
                a1 = math.radians(d.get(51, 360.0))
                if a1 <= a0:
                    a1 += 2 * math.pi
                if r > 1e-6:
                    start = (cx + r * math.cos(a0), cy + r * math.sin(a0))
                    arc_pts = ellipse_arc(cx, cy, r, r, a0, a1, 0.5, 0.02)
                    pts = [start, *arc_pts]
                    cleaned = _clean_polyline(pts)
                    if cleaned:
                        strokes.append(cleaned)
        else:
            idx += 1

    if not strokes:
        raise ValueError("Không tìm thấy thực thể hình học hợp lệ trong dữ liệu DXF.")
    return tuple(strokes)


# ==============================================================================
# 4. SVG XML & PATH PARSER / BỘ ĐỌC DỮ LIỆU ĐỒ HỌA VECTOR SVG
# ==============================================================================


def _parse_svg_path_d(d_attr):
    """
    [EN] Parse SVG `<path d="...">` command strings (`M/m`, `L/l`, `H/h`, `V/v`,
         `C/c`, `Q/q`, `Z/z`), elevating quadratic Béziers (`Q`) to cubics (`C`).
         Uppercase commands use absolute coordinates; lowercase commands use
         relative offsets from the current pen position `(x, y)`.
    [VI] Giải mã chuỗi lệnh `<path d="...">` của SVG (`M/m`, `L/l`, `H/h`, `V/v`,
         `C/c`, `Q/q`, `Z/z`), nâng bậc đường cong Bézier bậc 2 (`Q`) lên bậc 3 (`C`).
         Lệnh chữ hoa dùng tọa độ tuyệt đối; lệnh chữ thường dùng độ lệch tương đối
         tính từ vị trí bút hiện tại `(x, y)`.
    """
    tokens = re.findall(
        r"[MmLlHhVvCcQqZz]|[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?", d_attr
    )
    strokes = []
    current = []
    x = 0.0
    y = 0.0
    sx = 0.0
    sy = 0.0
    last_ctrl = None
    cmd = None
    i = 0

    def flush():
        """[EN] Save completed subpath / [VI] Lưu đường con đã hoàn thành."""
        nonlocal current
        cleaned = _clean_polyline(current)
        if cleaned:
            strokes.append(cleaned)
        current = []

    while i < len(tokens):
        tok = tokens[i]
        if tok.isalpha():
            cmd = tok
            i += 1
            # [EN] Z/z closes the current subpath back to its initial point (sx, sy).
            # [VI] Lệnh Z/z khép kín đường con hiện tại về điểm xuất phát (sx, sy).
            if cmd in ("Z", "z"):
                if current:
                    current.append((sx, sy))
                x, y = sx, sy
                flush()
                last_ctrl = None
            continue

        if cmd is None:
            break

        if cmd in ("M", "m"):
            # [EN] Move-to: start a new subpath; subsequent coordinate pairs act as Line-to.
            # [VI] Di chuyển tới điểm mới: mở nét vẽ mới; các cặp số tiếp theo hiểu là vẽ thẳng.
            nx = float(tokens[i])
            ny = float(tokens[i + 1])
            i += 2
            x, y = (x + nx, y + ny) if cmd == "m" else (nx, ny)
            sx, sy = x, y
            flush()
            current = [(x, y)]
            cmd = "l" if cmd == "m" else "L"
            last_ctrl = None

        elif cmd in ("L", "l"):
            # [EN] Line-to: straight segment to (nx, ny).
            # [VI] Vẽ đoạn thẳng tới (nx, ny).
            nx = float(tokens[i])
            ny = float(tokens[i + 1])
            i += 2
            x, y = (x + nx, y + ny) if cmd == "l" else (nx, ny)
            if not current:
                current = [(sx, sy)]
            current.append((x, y))
            last_ctrl = None

        elif cmd in ("H", "h"):
            # [EN] Horizontal line-to: keep Y unchanged.
            # [VI] Vẽ đoạn thẳng ngang: giữ nguyên tung độ Y.
            nx = float(tokens[i])
            i += 1
            x = x + nx if cmd == "h" else nx
            if not current:
                current = [(sx, sy)]
            current.append((x, y))
            last_ctrl = None

        elif cmd in ("V", "v"):
            # [EN] Vertical line-to: keep X unchanged.
            # [VI] Vẽ đoạn thẳng đứng: giữ nguyên hoành độ X.
            ny = float(tokens[i])
            i += 1
            y = y + ny if cmd == "v" else ny
            if not current:
                current = [(sx, sy)]
            current.append((x, y))
            last_ctrl = None

        elif cmd in ("C", "c"):
            # [EN] Cubic Bézier curve with control points c1, c2 and endpoint end.
            # [VI] Đường cong Bézier bậc 3 với hai điểm điều khiển c1, c2 và điểm cuối end.
            vals = [float(tokens[i + k]) for k in range(6)]
            i += 6
            if cmd == "c":
                c1 = (x + vals[0], y + vals[1])
                c2 = (x + vals[2], y + vals[3])
                end = (x + vals[4], y + vals[5])
            else:
                c1 = (vals[0], vals[1])
                c2 = (vals[2], vals[3])
                end = (vals[4], vals[5])
            if not current:
                current = [(x, y)]
            current.extend(flatten_cubic(((x, y), c1, c2, end), 0.5, 0.02))
            x, y = end
            last_ctrl = c2

        elif cmd in ("Q", "q"):
            # [EN] Quadratic Bézier curve with control point `qc` and endpoint `end`.
            #      Degree-elevated to an exact cubic Bézier using:
            #          c1 = P0 + (2/3) * (qc - P0),   c2 = end + (2/3) * (qc - end)
            # [VI] Đường cong Bézier bậc 2 với điểm điều khiển `qc` và điểm cuối `end`.
            #      Nâng bậc chính xác lên Bézier bậc 3 theo công thức:
            #          c1 = P0 + (2/3) * (qc - P0),   c2 = end + (2/3) * (qc - end)
            vals = [float(tokens[i + k]) for k in range(4)]
            i += 4
            qc = (x + vals[0], y + vals[1]) if cmd == "q" else (vals[0], vals[1])
            end = (x + vals[2], y + vals[3]) if cmd == "q" else (vals[2], vals[3])
            c1 = (x + 2 / 3 * (qc[0] - x), y + 2 / 3 * (qc[1] - y))
            c2 = (end[0] + 2 / 3 * (qc[0] - end[0]), end[1] + 2 / 3 * (qc[1] - end[1]))
            if not current:
                current = [(x, y)]
            current.extend(flatten_cubic(((x, y), c1, c2, end), 0.5, 0.02))
            x, y = end
            last_ctrl = qc

        else:
            i += 1

    flush()
    return strokes


def parse_svg(text):
    """
    [EN] Extract 2D polylines from SVG XML (`path`, `line`, `polyline`, `polygon`,
         `rect`, `circle`, `ellipse`), flipping SVG +Y-down into Cartesian +Y-up.
    [VI] Trích xuất đường gãy khúc 2D từ XML của SVG (`path`, `line`, `polyline`,
         `polygon`, `rect`, `circle`, `ellipse`), đồng thời lật trục +Y hướng xuống
         của SVG thành trục +Y hướng lên của hệ Đề-các.
    """
    try:
        root = ET.fromstring(text)
    except ET.ParseError as e:
        raise ValueError("Dữ liệu SVG không đúng định dạng XML.") from e

    raw_strokes = []
    for elem in root.iter():
        # [EN] Strip XML namespace prefix `{http://www.w3.org/2000/svg}` if present.
        # [VI] Loại bỏ tiền tố không gian tên XML `{http://www.w3.org/2000/svg}` nếu có.
        tag = elem.tag.split("}")[-1]

        if tag == "path" and elem.get("d"):
            raw_strokes.extend(_parse_svg_path_d(elem.get("d", "")))

        elif tag == "line":
            x1 = float(elem.get("x1", 0))
            y1 = float(elem.get("y1", 0))
            x2 = float(elem.get("x2", 0))
            y2 = float(elem.get("y2", 0))
            cleaned = _clean_polyline([(x1, y1), (x2, y2)])
            if cleaned:
                raw_strokes.append(cleaned)

        elif tag in ("polyline", "polygon") and elem.get("points"):
            num_strings = re.findall(
                r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?",
                elem.get("points", ""),
            )
            nums = []
            for v in num_strings:
                nums.append(float(v))

            pts = []
            for k in range(0, len(nums) - 1, 2):
                pts.append((nums[k], nums[k + 1]))

            if tag == "polygon" and pts:
                pts.append(pts[0])
            cleaned = _clean_polyline(pts)
            if cleaned:
                raw_strokes.append(cleaned)

        elif tag == "rect":
            rx = float(elem.get("x", 0))
            ry = float(elem.get("y", 0))
            rw = float(elem.get("width", 0))
            rh = float(elem.get("height", 0))
            if rw > 0 and rh > 0:
                rect_corners = [
                    (rx, ry),
                    (rx + rw, ry),
                    (rx + rw, ry + rh),
                    (rx, ry + rh),
                    (rx, ry),
                ]
                cleaned = _clean_polyline(rect_corners)
                if cleaned:
                    raw_strokes.append(cleaned)

        elif tag in ("circle", "ellipse"):
            cx = float(elem.get("cx", 0))
            cy = float(elem.get("cy", 0))
            rx = float(elem.get("r", elem.get("rx", 0)))
            ry = float(elem.get("r", elem.get("ry", 0)))
            if rx > 0 and ry > 0:
                arc_pts = ellipse_arc(
                    cx, cy, rx, ry, math.pi / 2, -3 * math.pi / 2, 0.5, 0.02
                )
                pts = [(cx, cy + ry), *arc_pts]
                pts[-1] = pts[0]
                cleaned = _clean_polyline(pts)
                if cleaned:
                    raw_strokes.append(cleaned)

    if not raw_strokes:
        raise ValueError("Không tìm thấy nét vẽ hợp lệ trong dữ liệu SVG.")

    # [EN] Flip Y coordinate `(x, -y)` to convert screen space (+Y down) to robot space (+Y up).
    # [VI] Đảo dấu tung độ `(x, -y)` để chuyển từ tọa độ màn hình (+Y xuống) sang tọa độ robot (+Y lên).
    flipped_strokes = []
    for stroke in raw_strokes:
        flipped_pts = []
        for x, y in stroke:
            flipped_pts.append((x, -y))
        flipped_strokes.append(tuple(flipped_pts))
    return tuple(flipped_strokes)


# ==============================================================================
# 5. UNIFIED VECTOR LOADER & CUSTOM SCALING / HÀM NẠP CHUNG & TỶ LỆ MẪU TÙY CHỈNH
# ==============================================================================


def load_vector_strokes(source, kind=None):
    """
    [EN] Load vector strokes from a file path or raw text string (`.svg`, `.dxf`,
         `.gcode`/`.nc`/`.ngc`/`.tap`), auto-detecting the format if `kind` is omitted.
    [VI] Nạp các nét vẽ vector từ đường dẫn file hoặc chuỗi văn bản (`.svg`, `.dxf`,
         `.gcode`/`.nc`/`.ngc`/`.tap`), tự động nhận diện định dạng nếu không truyền `kind`.
    """
    text = source
    inferred = (kind or "").lower().lstrip(".")
    if isinstance(source, Path) or (
        isinstance(source, str)
        and "\n" not in source
        and "<" not in source
        and Path(source).suffix.lower() in (".svg", ".dxf", ".gcode", ".nc", ".ngc", ".tap")
    ):
        path = Path(source)
        text = path.read_text(encoding="utf-8")
        if not inferred:
            inferred = path.suffix.lower().lstrip(".")

    if not inferred:
        stripped = text.lstrip()
        if stripped.startswith("<"):
            inferred = "svg"
        elif "ENTITIES" in text or "LWPOLYLINE" in text:
            inferred = "dxf"
        else:
            inferred = "gcode"

    if inferred == "svg":
        return parse_svg(text)
    if inferred == "dxf":
        return parse_dxf(text)
    if inferred in ("gcode", "nc", "ngc", "tap"):
        return parse_gcode(text)
    raise ValueError(f"Định dạng vector không hỗ trợ: {inferred}")


def register_custom_pattern(code, title, polylines, default_width=30.0):
    """
    [EN] Register a custom vector pattern by `code` in `CUSTOM_PATTERNS` so
         `BKSettings` can compile, preview, and draw it.
    [VI] Đăng ký một mẫu vector tùy chỉnh theo mã `code` vào `CUSTOM_PATTERNS`
         để `BKSettings` có thể biên dịch, xem trước và vẽ mẫu đó.
    """
    key = code.strip().upper()

    # [EN] Step 1: Clean and filter out any empty/degenerate strokes.
    # [VI] Bước 1: Làm sạch và loại bỏ các nét rỗng hoặc suy biến.
    cleaned_list = []
    for p in polylines:
        cleaned_stroke = _clean_polyline(p)
        if cleaned_stroke:
            cleaned_list.append(cleaned_stroke)
    cleaned = tuple(cleaned_list)
    if not cleaned:
        raise ValueError("Mẫu tùy chỉnh cần ít nhất một nét hợp lệ.")

    # [EN] Step 2: Verify that the pattern has a non-zero bounding box span.
    # [VI] Bước 2: Kiểm tra đảm bảo khung bao của mẫu có kích thước lớn hơn 0.
    xs = []
    ys = []
    for stroke in cleaned:
        for pt in stroke:
            xs.append(pt[0])
            ys.append(pt[1])

    span_x = max(xs) - min(xs)
    span_y = max(ys) - min(ys)
    if span_x <= 1e-6 and span_y <= 1e-6:
        raise ValueError("Mẫu tùy chỉnh có kích thước bằng 0.")

    # [EN] Step 3: Save in registry and invalidate cached scaled geometry.
    # [VI] Bước 3: Lưu vào từ điển mẫu tùy chỉnh và xóa bộ nhớ đệm tỷ lệ cũ.
    CUSTOM_PATTERNS[key] = (title, "Tùy chỉnh", float(default_width), cleaned)
    _scaled_custom.cache_clear()
    return key


@lru_cache(maxsize=32)
def _scaled_custom(pattern, width, cx, cy):
    """
    [EN] Scale a registered custom pattern to horizontal `width` (mm) at `(cx, cy)`.
    [VI] Phóng/thu mẫu tùy chỉnh đã đăng ký theo chiều rộng `width` (mm) tại tâm `(cx, cy)`.
    """
    if pattern not in CUSTOM_PATTERNS:
        raise ValueError("Mẫu tùy chỉnh chưa được nạp.")
    _title, _group, _def_w, polylines = CUSTOM_PATTERNS[pattern]

    # [EN] Step 1: Find raw bounding box (x0, y0, x1, y1) and center (ox, oy).
    # [VI] Bước 1: Tìm khung bao gốc (x0, y0, x1, y1) và tâm gốc (ox, oy).
    xs = []
    ys = []
    for stroke in polylines:
        for pt in stroke:
            xs.append(pt[0])
            ys.append(pt[1])

    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    base_w = max(x1 - x0, 1e-6)
    base_h = max(y1 - y0, 1e-6)
    scale = width / base_w
    ox = (x0 + x1) / 2
    oy = (y0 + y1) / 2

    # [EN] Step 2: Scale and translate every vertex into target workspace coordinates.
    # [VI] Bước 2: Phóng tỷ lệ và dịch chuyển từng đỉnh sang hệ tọa độ vùng làm việc.
    scaled_strokes = []
    for stroke in polylines:
        scaled_pts = []
        for x, y in stroke:
            px = cx + (x - ox) * scale
            py = cy + (y - oy) * scale
            scaled_pts.append((px, py))
        scaled_strokes.append(tuple(scaled_pts))
    scaled = tuple(scaled_strokes)

    # [EN] Step 3: Build lookup dictionary mapping "VEC_i_j" to ideal segment (a, b).
    # [VI] Bước 3: Tạo từ điển ánh xạ mã cạnh "VEC_i_j" sang đoạn thẳng lý tưởng (a, b).
    edges = {}
    for i, line in enumerate(scaled):
        for j, (a, b) in enumerate(zip(line, line[1:])):
            edges[f"VEC_{i}_{j}"] = (a, b)

    return (width, base_h * scale, scaled, edges)


def custom_dimensions(settings):
    """
    [EN] Return `(width_mm, height_mm)` for a custom pattern.
    [VI] Trả về `(chiều_rộng_mm, chiều_cao_mm)` của mẫu tùy chỉnh.
    """
    width_mm, height_mm, _scaled, _edges = _scaled_custom(
        settings.pattern, settings.size_mm, settings.center_x, settings.center_y
    )
    return (width_mm, height_mm)


def custom_strokes(settings, vertex_cls):
    """
    [EN] Subdivide scaled custom polylines into `vertex_cls(xy, "VEC_i_j")` strokes
         with maximum segment length `settings.step_mm`.
    [VI] Chia nhỏ các nét của mẫu tùy chỉnh thành các đỉnh `vertex_cls(xy, "VEC_i_j")`
         với độ dài đoạn tối đa `settings.step_mm`.
    """
    lines = _scaled_custom(
        settings.pattern, settings.size_mm, settings.center_x, settings.center_y
    )[2]
    strokes = []
    for i, line in enumerate(lines):
        vertices = [vertex_cls(line[0], f"VEC_{i}_0")]
        for j, (a, b) in enumerate(zip(line, line[1:])):
            x0, y0 = a
            x1, y1 = b
            n = max(1, math.ceil(math.dist(a, b) / settings.step_mm))
            part_label = f"VEC_{i}_{j}"
            for k in range(1, n + 1):
                px = x0 + (x1 - x0) * k / n
                py = y0 + (y1 - y0) * k / n
                vertices.append(vertex_cls((px, py), part_label))
        strokes.append(tuple(vertices))
    return tuple(strokes)


def custom_ink_distance(xy, part, settings):
    """
    [EN] Return the exact Euclidean distance from `xy` to custom edge `part` (`"VEC_i_j"`).
    [VI] Trả về khoảng cách Euclid chính xác từ điểm `xy` tới cạnh `part` (`"VEC_i_j"`).
    """
    edges = _scaled_custom(
        settings.pattern, settings.size_mm, settings.center_x, settings.center_y
    )[3]
    start_pt, end_pt = edges[part]
    return segment_distance(xy, start_pt, end_pt)
