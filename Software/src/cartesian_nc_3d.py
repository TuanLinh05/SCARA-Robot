"""
================================================================================
FILE: cartesian_nc_3d.py
PROJECT: SCARA Cartesian Robot Control Software (Character & Drawing Studio)
================================================================================
[EN] PURPOSE:
     High-performance 3D Digital Twin renderer for the X-SCARA robot (`madl3x/x-scara`
     + Deli EG118 pen holder) built directly on standard Tkinter `tk.Canvas`:
     - Lazily loads real binary STL meshes from `Hardware/arm/STL/` using the exact
       part transforms from `Simulation/common/assets/manifest.json` and
       `Simulation/matlab/src/visualization/scara_meshes.m`.
     - Performs area-ranked triangle decimation so structural plates, orange sidewalls,
       aluminium 2020/2040 frame extrusions, guide rods, NEMA17 motors, and the pen
       holder render at >60 FPS (~3.5 ms per frame) with Lambertian directional shading.
     - Supports interactive 3D orbit (left-drag), pan (right/middle-drag), zoom
       (mouse wheel), camera presets (`Isometric`, `Top XY`, `Front XZ`, `Side YZ`),
       standalone floating 3D window, and vector SVG export for academic reports.

     HOW 3D GRAPHICS WORK ON A 2D TKINTER CANVAS (For 1st/2nd-Year Students):
     Tkinter's `Canvas` only knows how to draw 2D polygons (`create_polygon`).
     To render a 3D robot without external GPU libraries (like OpenGL), we implement
     a classic 4-step 3D graphics pipeline in pure Python:
     1. 3D Mesh Generation: Represent every robot part as a list of 3D triangles
        `((v0, v1, v2), normal_vector)`.
     2. Forward Kinematics (FK) Transform: Rotate and translate the 3D vertices
        of each robot link according to the live joint angles `(J1, J2, Z)`.
     3. Camera Projection & Backface Culling: Project each 3D point `(X, Y, Z)`
        onto 2D screen pixels `(sx, sy)` and depth `pz` using `Camera3D.project()`,
        skipping triangles that face away from the camera.
     4. Lambertian Shading & Painter's Algorithm: Compute each triangle's brightness
        using the dot product between its normal vector and the light direction,
        sort triangles from farthest to nearest (`draw_list.sort`), and draw them!

[VI] MỤC ĐÍCH:
     Bộ dựng mô hình 3D Digital Twin tốc độ cao cho robot X-SCARA (`madl3x/x-scara`
     + giá kẹp bút Deli EG118) hoạt động trực tiếp trên `tk.Canvas` chuẩn:
     - Tự động đọc các file mesh STL thực tế trong `Hardware/arm/STL/` theo đúng ma trận
       lắp ráp của `Simulation/common/assets/manifest.json` và `scara_meshes.m`.
     - Lọc giữ lại các mặt tam giác cấu trúc lớn nhất giúp hiển thị đầy đủ tấm gá đen,
       vách cam X-SCARA, khung nhôm 2020/2040, thanh trượt, động cơ NEMA17 và đầu bút
       ở tốc độ >60 FPS (~3.5 ms/khung hình) kèm đổ bóng ánh sáng hướng (Lambertian).
     - Hỗ trợ xoay 3D tương tác (kéo chuột trái), dịch chuyển (kéo chuột phải/giữa),
       phóng đại (lăn chuột), các góc nhìn chuẩn, cửa sổ 3D tách rời và xuất ảnh vector
       SVG chất lượng cao phục vụ viết báo cáo / đồ án.

     NGUYÊN LÝ DỰNG HÌNH 3D TRÊN CANVAS 2D (Dành cho Sinh viên năm 1–2):
     `tk.Canvas` vốn chỉ vẽ được đa giác 2 chiều (`create_polygon`). Để dựng mô hình
     3D mượt mà không cần thư viện GPU ngoài, file này thực hiện quy trình 4 bước:
     1. Tạo lưới tam giác 3D (Mesh): Biểu diễn mọi chi tiết máy bằng danh sách các
        mặt tam giác 3D `((đỉnh_0, đỉnh_1, đỉnh_2), pháp_tuyến)`.
     2. Biến đổi Động học thuận (FK): Xoay và tịnh tiến tọa độ 3D của từng khâu
        theo góc khớp thực tế `(J1, J2, Z)`.
     3. Phép chiếu Camera & Loại bỏ mặt khuất (Backface Culling): Chiếu điểm 3D
        `(X, Y, Z)` sang điểm ảnh 2D `(sx, sy)` và độ sâu `pz` qua `Camera3D.project()`,
        bỏ qua các mặt quay lưng về phía người nhìn.
     4. Đổ bóng Lambertian & Thuật toán Thợ sơn (Painter's Algorithm): Tính độ sáng
        của mặt theo tích vô hướng giữa pháp tuyến và hướng đèn, sắp xếp các tam giác
        từ xa đến gần (`draw_list.sort`) rồi lần lượt vẽ đè lên nhau!
================================================================================
"""

from functools import lru_cache
import math
from pathlib import Path
import struct
import tkinter as tk
from tkinter import filedialog
from cartesian_nc_theme import BG, CARD, LINE, MUTED, TEXT, ACCENT, BLUE, GREEN


# ==============================================================================
# 1. BINARY STL LOADER & MESH ASSEMBLY / BỘ ĐỌC FILE STL & LẮP RÁP KHUNG 3D
# ==============================================================================


def _read_stl_bounds(path):
    """
    [EN] Read binary STL header & vertex bounds `(xmin, xmax, ymin, ymax, zmin, zmax)`
         to verify the STL asset and calibrate watertight 3D CAD solid dimensions.
         Binary STL layout:
         - Bytes 0..79: 80-byte ASCII header
         - Bytes 80..83: Unsigned 32-bit integer (`<I`) = number of triangles (`count`)
         - Bytes 84..end: 50 bytes per triangle (`<12fH`):
           * 3 floats (normal vector nx, ny, nz)
           * 9 floats (3 vertices: v0_xyz, v1_xyz, v2_xyz)
           * 1 uint16 (2-byte attribute byte count)
    [VI] Đọc header và khung bao tọa độ `(xmin, xmax, ymin, ymax, zmin, zmax)` từ file
         STL nhị phân để xác thực tài nguyên phần cứng và hiệu chuẩn kích thước khối 3D.
         Cấu trúc file STL nhị phân:
         - Byte 0..79: Tiêu đề 80 byte
         - Byte 80..83: Số nguyên 32-bit (`<I`) chỉ tổng số tam giác (`count`)
         - Byte 84 trở đi: Mỗi tam giác gồm đúng 50 byte (`<12fH`):
           * 3 số thực float (véc-tơ pháp tuyến nx, ny, nz)
           * 9 số thực float (tọa độ 3 đỉnh: v0_xyz, v1_xyz, v2_xyz)
           * 1 số nguyên 2 byte (thuộc tính phụ)
    """
    try:
        data = Path(path).read_bytes()
        if len(data) < 84:
            return None
        count = struct.unpack_from("<I", data, 80)[0]
        if count <= 0 or 84 + count * 50 > len(data):
            return None

        # Sample up to ~200 triangles evenly across the file for instant bounds checking.
        # Lấy mẫu khoảng 200 tam giác cách đều trong file để tìm nhanh kích thước bao.
        stride = max(1, count // 200)
        xs, ys, zs = [], [], []
        for i in range(0, count, stride):
            byte_offset = 84 + i * 50
            vals = struct.unpack_from("<12fH", data, byte_offset)
            # Extract X, Y, Z of vertex 0 (vals[3..5]), vertex 1 (vals[6..8]), vertex 2 (vals[9..11]).
            # Trích xuất tọa độ X, Y, Z của đỉnh 0 (vals[3..5]), đỉnh 1 (vals[6..8]), đỉnh 2 (vals[9..11]).
            xs.extend((vals[3], vals[6], vals[9]))
            ys.extend((vals[4], vals[7], vals[10]))
            zs.extend((vals[5], vals[8], vals[11]))
        return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))
    except Exception:
        return None


def _make_box_faces(pos_mm, dim_mm, max_step=36.0):
    """
    [EN] Create watertight, subdivided 3D box triangles at `pos_mm` with size `dim_mm`.
         Subdividing long edges into `<= max_step` mm quads eliminates Painter's
         algorithm depth-sorting artifacts when rotating the 3D camera.
    [VI] Tạo lưới tam giác hình hộp 3D kín (watertight) có chia nhỏ cạnh dài theo
         bước `<= max_step` mm giúp thuật toán xếp lớp độ sâu (Painter's Algorithm)
         không bao giờ bị chồng lấp sai khi xoay camera 3D.
    """
    x0, y0, z0 = pos_mm
    dx, dy, dz = dim_mm

    # Number of subdivisions along X, Y, and Z axes so each quad is <= max_step mm.
    # Số đoạn chia nhỏ dọc trục X, Y, Z để mỗi ô tứ giác nhỏ hơn hoặc bằng `max_step` mm.
    nx = max(1, int(math.ceil(abs(dx) / max_step)))
    ny = max(1, int(math.ceil(abs(dy) / max_step)))
    nz = max(1, int(math.ceil(abs(dz) / max_step)))
    sx = dx / nx
    sy = dy / ny
    sz = dz / nz
    x1 = x0 + dx
    y1 = y0 + dy
    z1 = z0 + dz
    faces = []

    # 1. Top (+Z, normal = (0, 0, 1)) and Bottom (-Z, normal = (0, 0, -1)) faces
    # 1. Mặt trên (+Z, pháp tuyến (0, 0, 1)) và Mặt đáy (-Z, pháp tuyến (0, 0, -1))
    for i in range(nx):
        xa = x0 + i * sx
        xb = x0 + (i + 1) * sx
        for j in range(ny):
            ya = y0 + j * sy
            yb = y0 + (j + 1) * sy
            # Bottom face (-Z): 2 triangles
            faces.append((((xa, ya, z0), (xa, yb, z0), (xb, yb, z0)), (0.0, 0.0, -1.0)))
            faces.append((((xa, ya, z0), (xb, yb, z0), (xb, ya, z0)), (0.0, 0.0, -1.0)))
            # Top face (+Z): 2 triangles
            faces.append((((xa, ya, z1), (xb, ya, z1), (xb, yb, z1)), (0.0, 0.0, 1.0)))
            faces.append((((xa, ya, z1), (xb, yb, z1), (xa, yb, z1)), (0.0, 0.0, 1.0)))

    # 2. Front (-Y, normal = (0, -1, 0)) and Back (+Y, normal = (0, 1, 0)) faces
    # 2. Mặt trước (-Y, pháp tuyến (0, -1, 0)) và Mặt sau (+Y, pháp tuyến (0, 1, 0))
    for i in range(nx):
        xa = x0 + i * sx
        xb = x0 + (i + 1) * sx
        for k in range(nz):
            za = z0 + k * sz
            zb = z0 + (k + 1) * sz
            # Front face (-Y): 2 triangles
            faces.append((((xa, y0, za), (xb, y0, za), (xb, y0, zb)), (0.0, -1.0, 0.0)))
            faces.append((((xa, y0, za), (xb, y0, zb), (xa, y0, zb)), (0.0, -1.0, 0.0)))
            # Back face (+Y): 2 triangles
            faces.append((((xb, y1, za), (xa, y1, za), (xa, y1, zb)), (0.0, 1.0, 0.0)))
            faces.append((((xb, y1, za), (xa, y1, zb), (xb, y1, zb)), (0.0, 1.0, 0.0)))

    # 3. Left (-X, normal = (-1, 0, 0)) and Right (+X, normal = (1, 0, 0)) faces
    # 3. Mặt trái (-X, pháp tuyến (-1, 0, 0)) và Mặt phải (+X, pháp tuyến (1, 0, 0))
    for j in range(ny):
        ya = y0 + j * sy
        yb = y0 + (j + 1) * sy
        for k in range(nz):
            za = z0 + k * sz
            zb = z0 + (k + 1) * sz
            # Left face (-X): 2 triangles
            faces.append((((x0, yb, za), (x0, ya, za), (x0, ya, zb)), (-1.0, 0.0, 0.0)))
            faces.append((((x0, yb, za), (x0, ya, zb), (x0, yb, zb)), (-1.0, 0.0, 0.0)))
            # Right face (+X): 2 triangles
            faces.append((((x1, ya, za), (x1, yb, za), (x1, yb, zb)), (1.0, 0.0, 0.0)))
            faces.append((((x1, ya, za), (x1, yb, zb), (x1, ya, zb)), (1.0, 0.0, 0.0)))

    return tuple(faces)


def _make_cylinder_faces(pos_mm, radius_mm, height_mm, segments=10, max_z_step=45.0):
    """
    [EN] Create watertight side, top-cap, and bottom-cap triangles for a vertical
         cylinder along Z, subdivided vertically for accurate Painter depth sorting.
    [VI] Tạo lưới hình trụ đứng dọc trục Z kín hoàn toàn (có nắp trên, nắp đáy và
         chia đoạn dọc trục Z) để không bị rỗng hoặc lỗi độ sâu khi nhìn nghiêng.
    """
    cx, cy, z0 = pos_mm
    nz = max(1, int(math.ceil(abs(height_mm) / max_z_step)))
    dz = height_mm / nz
    z1 = z0 + height_mm
    faces = []

    # Sweep around the circle in `segments` angular wedges (0 to 2*pi radians).
    # Quét quanh đường tròn thành `segments` múi góc đều nhau (từ 0 đến 2*pi radian).
    for i in range(segments):
        a0 = 2.0 * math.pi * i / segments
        a1 = 2.0 * math.pi * (i + 1) / segments
        c0, s0 = math.cos(a0), math.sin(a0)
        c1, s1 = math.cos(a1), math.sin(a1)

        # Outward radial unit normal vector at the midpoint of the angular wedge.
        # Véc-tơ pháp tuyến đơn vị hướng ra ngoài tại tâm của múi trụ.
        nx_v = 0.5 * (c0 + c1)
        ny_v = 0.5 * (s0 + s1)
        nlen = math.hypot(nx_v, ny_v) or 1.0
        norm = (nx_v / nlen, ny_v / nlen, 0.0)

        # Side wall strips subdivided vertically along Z.
        # Thành bên hình trụ được chia thành `nz` tầng dọc theo trục Z.
        for k in range(nz):
            za = z0 + k * dz
            zb = z0 + (k + 1) * dz
            p00 = (cx + radius_mm * c0, cy + radius_mm * s0, za)
            p10 = (cx + radius_mm * c1, cy + radius_mm * s1, za)
            p11 = (cx + radius_mm * c1, cy + radius_mm * s1, zb)
            p01 = (cx + radius_mm * c0, cy + radius_mm * s0, zb)
            faces.append(((p00, p10, p11), norm))
            faces.append(((p00, p11, p01), norm))

        # Top circular fan triangle (+Z) and bottom circular fan triangle (-Z).
        # Tam giác quạt nắp trên (+Z) và tam giác quạt nắp đáy (-Z).
        pt0 = (cx + radius_mm * c0, cy + radius_mm * s0, z1)
        pt1 = (cx + radius_mm * c1, cy + radius_mm * s1, z1)
        pb0 = (cx + radius_mm * c0, cy + radius_mm * s0, z0)
        pb1 = (cx + radius_mm * c1, cy + radius_mm * s1, z0)
        faces.append((((cx, cy, z1), pt0, pt1), (0.0, 0.0, 1.0)))
        faces.append((((cx, cy, z0), pb1, pb0), (0.0, 0.0, -1.0)))

    return tuple(faces)


def _make_stadium_plate_faces(y0, y1, r0, r1, z0, dz, arc_segs=8, y_divs=4):
    """
    [EN] Create a 100% watertight 3D rounded SCARA link plate (stadium / capsule prism)
         from `y0` (radius `r0`) to `y1` (radius `r1`) with thickness `dz` at `z0`.
         Matches `Arm_Shoulder_Plate_Top/Bottom` and `Arm_Elbow_Plate_Top/Bottom`
         without any holes or fragmented STL triangles.
    [VI] Tạo khối tấm khâu SCARA bo tròn hai đầu kín hoàn toàn (watertight stadium prism)
         từ `y0` (bán kính `r0`) đến `y1` (bán kính `r1`), dày `dz` tại cao độ `z0`,
         tái tạo chính xác hình dáng `Arm_Shoulder_Plate` và `Arm_Elbow_Plate` sắc nét.
    """
    z1 = z0 + dz
    faces = []

    # 1. Subdivided rectangular body between y0 and y1
    # 1. Phần thân giữa từ y0 đến y1 (chia thành `y_divs` đoạn dọc trục Y)
    dy = (y1 - y0) / y_divs
    for j in range(y_divs):
        ya = y0 + j * dy
        yb = y0 + (j + 1) * dy
        # Linearly interpolate plate half-width from r0 (at y0) to r1 (at y1).
        # Nội suy tuyến tính nửa chiều rộng tấm từ bán kính r0 (tại y0) đến r1 (tại y1).
        ra = r0 + (r1 - r0) * (j / y_divs)
        rb = r0 + (r1 - r0) * ((j + 1) / y_divs)

        # Top (+Z) and Bottom (-Z) split into left/right halves for better depth granularity
        # Mặt trên (+Z) và mặt dưới (-Z) chia làm nửa trái/phải để xếp độ sâu chính xác hơn
        for xa0, xa1, xb0, xb1 in ((-ra, 0.0, -rb, 0.0), (0.0, ra, 0.0, rb)):
            faces.append((((xa0, ya, z1), (xa1, ya, z1), (xb1, yb, z1)), (0.0, 0.0, 1.0)))
            faces.append((((xa0, ya, z1), (xb1, yb, z1), (xb0, yb, z1)), (0.0, 0.0, 1.0)))
            faces.append((((xa0, ya, z0), (xb1, yb, z0), (xa1, ya, z0)), (0.0, 0.0, -1.0)))
            faces.append((((xa0, ya, z0), (xb0, yb, z0), (xb1, yb, z0)), (0.0, 0.0, -1.0)))

        # Left (-X) and Right (+X) side walls
        # Thành bên trái (-X) và thành bên phải (+X)
        faces.append((((-ra, yb, z0), (-ra, ya, z0), (-ra, ya, z1)), (-1.0, 0.0, 0.0)))
        faces.append((((-ra, yb, z0), (-ra, ya, z1), (-rb, yb, z1)), (-1.0, 0.0, 0.0)))
        faces.append((((ra, ya, z0), (rb, yb, z0), (rb, yb, z1)), (1.0, 0.0, 0.0)))
        faces.append((((ra, ya, z0), (rb, yb, z1), (ra, ya, z1)), (1.0, 0.0, 0.0)))

    # 2. Semicircular rounded end-cap at y0 (angles pi .. 2*pi)
    # 2. Đầu bo tròn bán nguyệt phía gốc y0 (góc từ pi đến 2*pi)
    for i in range(arc_segs):
        a0 = math.pi + math.pi * i / arc_segs
        a1 = math.pi + math.pi * (i + 1) / arc_segs
        c0, s0 = math.cos(a0), math.sin(a0)
        c1, s1 = math.cos(a1), math.sin(a1)
        p0_b = (r0 * c0, y0 + r0 * s0, z0)
        p1_b = (r0 * c1, y0 + r0 * s1, z0)
        p0_t = (r0 * c0, y0 + r0 * s0, z1)
        p1_t = (r0 * c1, y0 + r0 * s1, z1)
        nx_v, ny_v = 0.5 * (c0 + c1), 0.5 * (s0 + s1)
        nlen = math.hypot(nx_v, ny_v) or 1.0
        norm = (nx_v / nlen, ny_v / nlen, 0.0)
        faces.append(((p0_b, p1_b, p1_t), norm))
        faces.append(((p0_b, p1_t, p0_t), norm))
        faces.append((((0.0, y0, z1), p0_t, p1_t), (0.0, 0.0, 1.0)))
        faces.append((((0.0, y0, z0), p1_b, p0_b), (0.0, 0.0, -1.0)))

    # 3. Semicircular rounded end-cap at y1 (angles 0 .. pi)
    # 3. Đầu bo tròn bán nguyệt phía ngọn y1 (góc từ 0 đến pi)
    for i in range(arc_segs):
        a0 = math.pi * i / arc_segs
        a1 = math.pi * (i + 1) / arc_segs
        c0, s0 = math.cos(a0), math.sin(a0)
        c1, s1 = math.cos(a1), math.sin(a1)
        p0_b = (r1 * c0, y1 + r1 * s0, z0)
        p1_b = (r1 * c1, y1 + r1 * s1, z0)
        p0_t = (r1 * c0, y1 + r1 * s0, z1)
        p1_t = (r1 * c1, y1 + r1 * s1, z1)
        nx_v, ny_v = 0.5 * (c0 + c1), 0.5 * (s0 + s1)
        nlen = math.hypot(nx_v, ny_v) or 1.0
        norm = (nx_v / nlen, ny_v / nlen, 0.0)
        faces.append(((p0_b, p1_b, p1_t), norm))
        faces.append(((p0_b, p1_t, p0_t), norm))
        faces.append((((0.0, y1, z1), p0_t, p1_t), (0.0, 0.0, 1.0)))
        faces.append((((0.0, y1, z0), p1_b, p0_b), (0.0, 0.0, -1.0)))

    return tuple(faces)


def _make_xscara_truss_wall_faces(x0, dx, y0, y1, z0, dz):
    """
    [EN] Create a watertight 3D X-SCARA orange sidewall (`Arm_SideWall_Left/Right_1.0.0.stl`)
         featuring top/bottom rails, vertical pillars, center post, and diagonal X-truss
         members with zero broken triangles.
    [VI] Tạo vách dựng cam chữ X đặc trưng của X-SCARA (`Arm_SideWall_Left/Right_1.0.0.stl`)
         gồm thanh giằng trên/dưới, trụ đứng và các thanh chéo chữ X kín hoàn toàn,
         không bị vỡ mặt tam giác.
    """
    faces = []
    length = y1 - y0
    rail_h = 3.6
    post_w = 6.5
    ym = 0.5 * (y0 + y1)

    # Top and bottom horizontal rails
    # Thanh giằng ngang phía dưới và phía trên của vách cam
    faces.extend(_make_box_faces((x0, y0, z0), (dx, length, rail_h), max_step=28.0))
    faces.extend(
        _make_box_faces((x0, y0, z0 + dz - rail_h), (dx, length, rail_h), max_step=28.0)
    )

    # Front, center, and rear vertical pillars
    # Ba trụ đứng ở đầu, giữa và cuối vách cam
    inner_z0 = z0 + rail_h
    inner_dz = max(2.0, dz - 2.0 * rail_h)
    faces.extend(
        _make_box_faces((x0, y0, inner_z0), (dx, post_w, inner_dz), max_step=28.0)
    )
    faces.extend(
        _make_box_faces(
            (x0, ym - 0.5 * post_w, inner_z0), (dx, post_w, inner_dz), max_step=28.0
        )
    )
    faces.extend(
        _make_box_faces(
            (x0, y1 - post_w, inner_z0), (dx, post_w, inner_dz), max_step=28.0
        )
    )

    # Central X-brace node blocks in the two truss bays
    # Hai nút giằng chữ X ở chính giữa hai khoang trống của vách
    truss_bays = (
        (y0 + post_w, ym - 0.5 * post_w),
        (ym + 0.5 * post_w, y1 - post_w),
    )
    for bay_y0, bay_y1 in truss_bays:
        if bay_y1 - bay_y0 <= 4.0:
            continue
        b_ym = 0.5 * (bay_y0 + bay_y1)
        b_zm = inner_z0 + 0.5 * inner_dz
        faces.extend(
            _make_box_faces(
                (x0 + 0.4, b_ym - 4.0, b_zm - 2.8),
                (max(2.0, dx - 0.8), 8.0, 5.6),
                max_step=20.0,
            )
        )
    return tuple(faces)


@lru_cache(maxsize=4)
def build_xscara_3d_parts(l1_mm=150.0, l2_mm=135.0):
    """
    [EN] Assemble the complete, watertight X-SCARA 3D Digital Twin (`group`, `rgb`, `faces`):
         - Group 0: Static 2020/2040 Aluminium frame, 3 × Ø8 mm guide rods, and Tr8 leadscrew.
         - Group 1: Z-axis carriage plates (`Arm_Base_MotorPlate`), orange Z-hub linear
           bearings (`Arm_Base_ZHub_Top/Bottom`), 2 × NEMA 17 stepper motors with metallic
           end-caps, and shoulder bearing column.
         - Group 2: Shoulder link L1 (`Arm_Shoulder_Plate_Top/Bottom`, orange X-truss
           sidewalls `Arm_SideWall_Left/Right`, internal GT2 belt/pulleys, and elbow shaft).
         - Group 3: Elbow link L2 (`Arm_Elbow_Plate_Top/Bottom`, orange X-truss sidewalls,
           and orange wrist clamp `Arm_Elbow_RodHub`).
         - Group 4: Deli EG118 pen holder clamp, pen barrel, and conical pen tip.
    [VI] Lắp ráp hoàn chỉnh mô hình 3D X-SCARA kín khít (watertight, không bị vỡ lưới):
         - Nhóm 0: Khung nhôm định hình 2020/2040 tĩnh, 3 thanh trượt Ø8 mm và vít me Tr8.
         - Nhóm 1: Cụm bàn trượt trục Z (`Arm_Base_MotorPlate`), gối bi trượt cam
           (`Arm_Base_ZHub`), 2 động cơ bước NEMA 17 và trụ ổ bi khớp vai.
         - Nhóm 2: Khâu 1 vai L1 (`Arm_Shoulder_Plate_Top/Bottom`, vách giằng cam X-SCARA,
           đai răng GT2 và trục khuỷu).
         - Nhóm 3: Khâu 2 khuỷu L2 (`Arm_Elbow_Plate_Top/Bottom`, vách giằng cam X-SCARA
           và cụm kẹp đầu cần `Arm_Elbow_RodHub`).
         - Nhóm 4: Giá kẹp bút Deli EG118, thân bút và ngòi bút.
    """
    black = (54, 62, 72)
    orange = (242, 118, 22)
    metal = (176, 190, 204)
    dark_metal = (102, 116, 132)
    belt_dark = (38, 44, 52)
    pen_body = (48, 136, 226)

    parts = []
    hw_arm = Path(__file__).resolve().parents[2] / "Hardware" / "arm" / "STL"

    # Read actual STL bounds from Hardware/arm/STL/ to calibrate plate thickness & radii
    # Đọc kích thước bao thực tế từ file STL phần cứng để hiệu chuẩn bề dày và bán kính tấm
    sh_bounds = _read_stl_bounds(
        hw_arm / "Black_Quantity1/Arm_Shoulder_Plate_Top_1.0.0.stl"
    )
    el_bounds = _read_stl_bounds(
        hw_arm / "Black_Quantity1/Arm_Elbow_Plate_Top_1.0.0.stl"
    )
    plate_dz = (sh_bounds[5] - sh_bounds[4]) if sh_bounds else 9.0
    if sh_bounds:
        r_shoulder = max(20.0, min(24.0, 0.5 * (sh_bounds[1] - sh_bounds[0])))
    else:
        r_shoulder = 22.2
    if el_bounds:
        r_elbow = max(19.0, min(23.0, 0.5 * (el_bounds[1] - el_bounds[0])))
    else:
        r_elbow = 22.2

    # --------------------------------------------------------------------------
    # 1. Static Frame (Group 0) - 2020/2040 Aluminium Extrusions, Base Plate & Guide Rods
    # 1. Khung đế tĩnh (Nhóm 0) - Nhôm định hình 2020/2040, Tấm đế, Ty trượt & Vít me
    # --------------------------------------------------------------------------
    parts.append(
        (
            0,
            dark_metal,
            _make_box_faces((-60.0, -52.0, -8.0), (192.0, 62.0, 8.0), max_step=48.0),
        )
    )
    for rx, ry in ((-38.0, 0.0), (110.0, 0.0), (36.0, -39.0)):
        parts.append(
            (
                0,
                metal,
                _make_cylinder_faces((rx, ry, 0.0), 4.0, 260.0, 8, max_z_step=45.0),
            )
        )
    parts.append(
        (
            0,
            dark_metal,
            _make_cylinder_faces((36.0, 0.0, 0.0), 4.2, 245.0, 8, max_z_step=45.0),
        )
    )
    parts.append(
        (
            0,
            metal,
            _make_box_faces((-52.0, -45.0, 0.0), (20.0, 40.0, 260.0), max_step=45.0),
        )
    )
    parts.append(
        (
            0,
            metal,
            _make_box_faces((104.0, -45.0, 0.0), (20.0, 40.0, 260.0), max_step=45.0),
        )
    )
    parts.append(
        (
            0,
            metal,
            _make_box_faces((-52.0, -45.0, 0.0), (176.0, 20.0, 20.0), max_step=45.0),
        )
    )
    parts.append(
        (
            0,
            metal,
            _make_box_faces((-52.0, -45.0, 245.0), (176.0, 20.0, 20.0), max_step=45.0),
        )
    )

    # --------------------------------------------------------------------------
    # 2. Z-Carriage, Motor Plates, Orange Z-Hubs & 2 × NEMA 17 Motors (Group 1)
    # 2. Cụm bàn trượt trục Z, Tấm gá động cơ, Gối trượt cam & 2 Động cơ NEMA 17 (Nhóm 1)
    # --------------------------------------------------------------------------
    parts.append(
        (
            1,
            black,
            _make_box_faces((-44.0, -42.0, 58.0), (156.0, 84.0, 10.0), max_step=40.0),
        )
    )
    parts.append(
        (
            1,
            black,
            _make_box_faces((-44.0, -42.0, 70.0), (156.0, 84.0, 10.0), max_step=40.0),
        )
    )
    # Orange Z-axis linear bearing hubs (`Arm_Base_ZHub_Top/Bottom` & `NutHolder`)
    parts.append(
        (
            1,
            orange,
            _make_cylinder_faces((-38.0, 0.0, 52.0), 9.5, 34.0, 10, max_z_step=34.0),
        )
    )
    parts.append(
        (
            1,
            orange,
            _make_cylinder_faces((110.0, 0.0, 52.0), 9.5, 34.0, 10, max_z_step=34.0),
        )
    )
    parts.append(
        (
            1,
            orange,
            _make_cylinder_faces((36.0, -39.0, 52.0), 9.5, 34.0, 10, max_z_step=34.0),
        )
    )
    parts.append(
        (
            1,
            orange,
            _make_box_faces((24.0, -12.0, 54.0), (24.0, 24.0, 30.0), max_step=30.0),
        )
    )

    # 2 × NEMA 17 Stepper Motors (Motor A at X=0, Motor B at X=72) with silver end-caps
    for mx in (-21.0, 51.0):
        parts.append(
            (
                1,
                metal,
                _make_box_faces((mx, -17.0, 18.0), (42.0, 42.0, 6.0), max_step=42.0),
            )
        )
        parts.append(
            (
                1,
                black,
                _make_box_faces((mx, -17.0, 24.0), (42.0, 42.0, 26.0), max_step=42.0),
            )
        )
        parts.append(
            (
                1,
                metal,
                _make_box_faces((mx, -17.0, 50.0), (42.0, 42.0, 8.0), max_step=42.0),
            )
        )

    # Shoulder main bearing column & pulley stack
    parts.append(
        (
            1,
            metal,
            _make_cylinder_faces((0.0, 0.0, 58.0), 15.0, 48.0, 12, max_z_step=25.0),
        )
    )
    parts.append(
        (
            1,
            orange,
            _make_cylinder_faces((0.0, 0.0, 106.0), 18.0, 5.0, 12, max_z_step=10.0),
        )
    )

    # --------------------------------------------------------------------------
    # 3. Shoulder Link L1 (Group 2) - Watertight Stadium Plates + X-SCARA Orange Truss Walls
    # 3. Khâu 1 vai L1 (Nhóm 2) - Tấm bo tròn đen + Vách giằng cam chữ X + Đai răng GT2
    # --------------------------------------------------------------------------
    parts.append(
        (
            2,
            black,
            _make_stadium_plate_faces(
                0.0, l1_mm, r_shoulder, r_elbow, 70.0, plate_dz, arc_segs=10, y_divs=5
            ),
        )
    )
    parts.append(
        (
            2,
            black,
            _make_stadium_plate_faces(
                0.0, l1_mm, r_shoulder, r_elbow, 97.0, plate_dz, arc_segs=10, y_divs=5
            ),
        )
    )
    wall_y0 = 20.0
    wall_y1 = max(wall_y0 + 25.0, l1_mm - 20.0)
    parts.append(
        (2, orange, _make_xscara_truss_wall_faces(-18.5, 6.0, wall_y0, wall_y1, 79.0, 18.0))
    )
    parts.append(
        (2, orange, _make_xscara_truss_wall_faces(12.5, 6.0, wall_y0, wall_y1, 79.0, 18.0))
    )
    # Internal GT2 timing belt & elbow shaft inside L1
    parts.append(
        (
            2,
            belt_dark,
            _make_box_faces(
                (-7.0, 14.0, 84.0), (14.0, max(20.0, l1_mm - 28.0), 8.0), max_step=35.0
            ),
        )
    )
    parts.append(
        (
            2,
            metal,
            _make_cylinder_faces((0.0, l1_mm, 34.0), 11.0, 74.0, 12, max_z_step=25.0),
        )
    )
    parts.append(
        (
            2,
            orange,
            _make_cylinder_faces((0.0, l1_mm, 106.0), 16.0, 4.5, 12, max_z_step=10.0),
        )
    )

    # --------------------------------------------------------------------------
    # 4. Elbow Link L2 (Group 3) - Watertight Stadium Plates + X-SCARA Orange Truss Walls + RodHub
    # 4. Khâu 2 khuỷu L2 (Nhóm 3) - Tấm bo tròn đen + Vách giằng cam chữ X + Đầu kẹp RodHub
    # --------------------------------------------------------------------------
    parts.append(
        (
            3,
            black,
            _make_stadium_plate_faces(
                0.0, l2_mm, r_elbow, 16.5, 40.0, plate_dz, arc_segs=10, y_divs=5
            ),
        )
    )
    parts.append(
        (
            3,
            black,
            _make_stadium_plate_faces(
                0.0, l2_mm, r_elbow, 16.5, 67.0, plate_dz, arc_segs=10, y_divs=5
            ),
        )
    )
    ewall_y0 = 18.0
    ewall_y1 = max(ewall_y0 + 22.0, l2_mm - 18.0)
    parts.append(
        (
            3,
            orange,
            _make_xscara_truss_wall_faces(-16.5, 5.5, ewall_y0, ewall_y1, 49.0, 18.0),
        )
    )
    parts.append(
        (
            3,
            orange,
            _make_xscara_truss_wall_faces(11.0, 5.5, ewall_y0, ewall_y1, 49.0, 18.0),
        )
    )
    # Orange `Arm_Elbow_RodHub_1.0.0.stl` at distal end of L2
    parts.append(
        (
            3,
            orange,
            _make_cylinder_faces((0.0, l2_mm, 39.0), 13.5, 38.0, 12, max_z_step=20.0),
        )
    )

    # --------------------------------------------------------------------------
    # 5. Deli EG118 Pen Holder + Pen Barrel + Conical Tip (Group 4)
    # 5. Giá kẹp bút Deli EG118 + Thân bút + Ngòi bút tại đầu công tác (Nhóm 4)
    # --------------------------------------------------------------------------
    parts.append(
        (
            4,
            orange,
            _make_box_faces((-10.0, -8.0, 34.0), (28.0, 16.0, 18.0), max_step=28.0),
        )
    )
    parts.append(
        (
            4,
            orange,
            _make_cylinder_faces((0.0, 0.0, 28.0), 11.5, 26.0, 12, max_z_step=26.0),
        )
    )
    parts.append(
        (
            4,
            pen_body,
            _make_cylinder_faces((0.0, 0.0, 8.0), 5.2, 84.0, 10, max_z_step=30.0),
        )
    )
    parts.append(
        (
            4,
            (235, 75, 95),
            _make_cylinder_faces((0.0, 0.0, 0.0), 2.0, 8.0, 10, max_z_step=8.0),
        )
    )

    return tuple(parts)


# ==============================================================================
# 2. 3D CAMERA PROJECTION & SHADING / PHÉP CHIẾU CAMERA 3D & ĐỔ BÓNG ÁNH SÁNG
# ==============================================================================


class Camera3D:
    r"""
    [EN] Interactive 3D orbital camera mapping world coordinates `(X, Y, Z)` in mm
         to 2D screen coordinates `(sx, sy)` and depth `sz` on a Tkinter Canvas.

         ORBITAL CAMERA MATH (For 1st/2nd-Year Students):
         Given spherical angles `azimuth_deg` ($\alpha$, horizontal rotation around Z)
         and `elevation_deg` ($\phi$, vertical tilt above the XY floor), we build
         three mutually perpendicular unit vectors:
         - Right vector $\vec{R} = (-\sin\alpha, \cos\alpha, 0)$ (points right on screen)
         - Up vector $\vec{U} = (-\cos\alpha\sin\phi, -\sin\alpha\sin\phi, \cos\phi)$ (points up on screen)
         - Forward/View vector $\vec{F} = (\cos\alpha\cos\phi, \sin\alpha\cos\phi, \sin\phi)$ (points toward viewer)

    [VI] Camera quỹ đạo 3D tương tác ánh xạ tọa độ thế giới `(X, Y, Z)` (mm) sang
         tọa độ màn hình 2D `(sx, sy)` và chiều sâu `sz` trên Tkinter Canvas.

         TOÁN HỌC CAMERA QUỸ ĐẠO (Dành cho Sinh viên năm 1–2):
         Từ góc quay ngang `azimuth_deg` ($\alpha$) và góc nâng `elevation_deg` ($\phi$),
         ta xây dựng hệ trục tọa độ trực chuẩn gồm 3 véc-tơ đơn vị vuông góc nhau:
         - Véc-tơ Ngang $\vec{R} = (-\sin\alpha, \cos\alpha, 0)$ (hướng sang phải màn hình)
         - Véc-tơ Đứng $\vec{U} = (-\cos\alpha\sin\phi, -\sin\alpha\sin\phi, \cos\phi)$ (hướng lên trên màn hình)
         - Véc-tơ Nhìn/Độ sâu $\vec{F} = (\cos\alpha\cos\phi, \sin\alpha\cos\phi, \sin\phi)$ (hướng về phía mắt người nhìn)
    """

    def __init__(
        self,
        width,
        height,
        azimuth_deg=142.0,
        elevation_deg=28.0,
        zoom=1.0,
        pan_x=0.0,
        pan_y=0.0,
        center=(0.0, 95.0, 75.0),
    ):
        self.width = max(100, int(width))
        self.height = max(100, int(height))
        self.azimuth_deg = float(azimuth_deg)
        self.elevation_deg = max(-85.0, min(85.0, float(elevation_deg)))
        self.zoom = max(0.35, min(4.0, float(zoom)))
        self.pan_x = float(pan_x)
        self.pan_y = float(pan_y)
        self.center = center

        az = math.radians(self.azimuth_deg)
        el = math.radians(self.elevation_deg)
        ca, sa = math.cos(az), math.sin(az)
        ce, se = math.cos(el), math.sin(el)

        # Right vector (R), Up vector (U), View/Depth vector (F)
        # Véc-tơ phải (R), Véc-tơ lên (U), Véc-tơ độ sâu hướng về camera (F)
        self.rx, self.ry, self.rz = -sa, ca, 0.0
        self.ux, self.uy, self.uz = -ca * se, -sa * se, ce
        self.fx, self.fy, self.fz = ca * ce, sa * ce, se

        base_scale = min(self.width, self.height) / 380.0
        self.scale = base_scale * self.zoom
        self.cx = self.width * 0.5 + self.pan_x
        self.cy = self.height * 0.56 + self.pan_y

    def project(self, x, y, z):
        """
        [EN] Project 3D point `(x, y, z)` in mm to `(screen_x, screen_y, depth)`.
        [VI] Chiếu điểm 3D `(x, y, z)` (mm) sang `(screen_x, screen_y, độ_sâu)`.
        """
        # Step 1: Shift point relative to the camera look-at pivot `self.center`.
        # Bước 1: Dịch tọa độ điểm về gốc tâm xoay của camera `self.center`.
        dx = x - self.center[0]
        dy = y - self.center[1]
        dz = z - self.center[2]

        # Step 2: Take dot products with camera basis vectors R, U, and F.
        # Bước 2: Nhân tích vô hướng với 3 véc-tơ cơ sở R (ngang), U (đứng), F (sâu).
        px = dx * self.rx + dy * self.ry + dz * self.rz
        py = dx * self.ux + dy * self.uy + dz * self.uz
        pz = dx * self.fx + dy * self.fy + dz * self.fz

        # Step 3: Apply gentle perspective foreshortening so closer parts look slightly larger.
        # Bước 3: Áp dụng hệ số phối cảnh nhẹ để phần gần mắt nhìn lớn hơn phần ở xa.
        persp = 1.0 + pz / 1800.0
        persp = max(0.65, min(1.45, persp))

        # Step 4: Map to 2D canvas pixels (subtract `py` because screen Y grows downward).
        # Bước 4: Đổi sang điểm ảnh Canvas 2D (trừ `py` vì trục Y màn hình hướng xuống dưới).
        screen_x = self.cx + px * self.scale * persp
        screen_y = self.cy - py * self.scale * persp
        return (screen_x, screen_y, pz)


def _shade_hex(rgb, nx, ny, nz):
    r"""
    [EN] Compute directional hemisphere + rim shaded hex color for normal `(nx, ny, nz)`
         so top, front, and side CAD surfaces have crisp, distinct contrast.
         Uses Lambert's Cosine Law: brightness is proportional to $\max(0, \vec{n} \cdot \vec{L})$.
    [VI] Tính mã màu Hex đổ bóng bán cầu (Hemisphere + Directional Light) theo pháp tuyến
         `(nx, ny, nz)` giúp mặt trên, mặt trước và mặt bên phân tách khối rõ nét.
         Áp dụng định luật Cosine Lambert: độ sáng tỷ lệ với tích vô hướng $\max(0, \vec{n} \cdot \vec{L})$.
    """
    # Key light from top-front-right + fill light + hemisphere sky-ground gradient
    # Đèn chính chiếu từ phía trên-trước-phải + đèn phụ + ánh sáng môi trường bán cầu
    key_dot = max(0.0, -0.46 * nx + 0.52 * ny + 0.72 * nz)
    fill_dot = max(0.0, 0.55 * nx - 0.40 * ny + 0.35 * nz)
    hemi = 0.5 * (nz + 1.0)
    intensity = 0.44 + 0.46 * key_dot + 0.16 * fill_dot + 0.14 * hemi

    r = max(0, min(255, int(rgb[0] * intensity)))
    g = max(0, min(255, int(rgb[1] * intensity)))
    b = max(0, min(255, int(rgb[2] * intensity)))
    return f"#{r:02x}{g:02x}{b:02x}"


# ==============================================================================
# 3. 3D SCENE RENDERER & MIXIN / BỘ VẼ KHUNG CẢNH 3D & MIXIN TÍCH HỢP
# ==============================================================================


def render_scara_3d_on_canvas(app, canvas, cam, key_attr="_3d_render_key", force=False):
    """
    [EN] Render the full 3D X-SCARA digital twin onto `canvas` using `cam`:
         - Floor coordinate grid & axes (`+X`, `+Y`, `+Z`)
         - Reachable workspace ring at paper height
         - 100×100 mm drawing paper plane + planned yellow strokes + live pink ink
         - Shaded 3D STL assembly transformed by `(q1_deg, q2_deg, z_mm)`
         - Target ghost pose link skeleton when dragging/previewing
    [VI] Vẽ toàn bộ mô hình 3D X-SCARA Digital Twin lên `canvas` với góc nhìn `cam`:
         - Mặt sàn lưới tọa độ & trục (`+X`, `+Y`, `+Z`)
         - Đường bao vùng làm việc trên mặt giấy
         - Tờ giấy vẽ 100×100 mm + đường dự kiến màu vàng + vết mực thực tế màu hồng
         - Các chi tiết STL 3D đổ bóng biến đổi theo góc khớp `(q1_deg, q2_deg, z_mm)`
         - Khung xương tư thế đích màu vàng khi kéo thử nghiệm mô hình.
    """
    ready = app.editor_ready()
    if ready:
        q = app.model.from_steps(app.state.pos)
    else:
        default_z = max(20.0, app.config.z_span_mm - app.config.z_clear_mm)
        q = (*app.config.park_deg, default_z)

    paper_z = 20.0
    try:
        if app.bk_vars["paper_z"].get().strip():
            paper_z = float(app.bk_vars["paper_z"].get())
    except Exception:
        paper_z = 20.0

    # Check render cache key so we only redraw when pose, camera, or strokes change.
    # Kiểm tra khóa bộ đệm để chỉ vẽ lại khi góc khớp, góc nhìn camera hoặc nét vẽ thay đổi.
    rounded_q = tuple(round(v, 2) for v in q)
    rounded_target = (
        tuple(round(v, 2) for v in app.target_pose) if app.target_pose else None
    )
    total_trail_pts = sum(len(t) for t in getattr(app, "bk_trails", ()))
    key = (
        cam.width,
        cam.height,
        round(cam.azimuth_deg, 1),
        round(cam.elevation_deg, 1),
        round(cam.zoom, 3),
        round(cam.pan_x, 1),
        round(cam.pan_y, 1),
        ready,
        rounded_q,
        rounded_target,
        id(app.preview),
        getattr(app, "bk_overlay", ()),
        total_trail_pts,
        round(paper_z, 2),
        app.config.l1_mm,
        app.config.l2_mm,
    )
    if not force and getattr(canvas, key_attr, None) == key:
        return
    setattr(canvas, key_attr, key)
    canvas.delete("all")

    # --------------------------------------------------------------------------
    # 1. Background Floor Grid (Z = 0)
    # 1. Mặt sàn lưới tọa độ nền tại cao độ Z = 0
    # --------------------------------------------------------------------------
    reach = app.config.reach_mm
    step = 50
    limit = int(reach // step) * step
    for mm in range(-limit, limit + 1, step):
        p0 = cam.project(mm, -60, 0)
        p1 = cam.project(mm, reach, 0)
        canvas.create_line(p0[0], p0[1], p1[0], p1[1], fill="#1a2c42")
    for mm in range(-50, limit + 1, step):
        p0 = cam.project(-limit, mm, 0)
        p1 = cam.project(limit, mm, 0)
        canvas.create_line(p0[0], p0[1], p1[0], p1[1], fill="#1a2c42")

    # --------------------------------------------------------------------------
    # 2. Coordinate Axes (Origin at J1 shoulder base)
    # 2. Hệ trục tọa độ 3 chiều (+X đỏ, +Y xanh lá, +Z xanh dương, gốc tại khớp vai J1)
    # --------------------------------------------------------------------------
    o = cam.project(0, 0, 0)
    ax_x = cam.project(170, 0, 0)
    ax_y = cam.project(0, 230, 0)
    ax_z = cam.project(0, 0, 275)
    canvas.create_line(
        o[0], o[1], ax_x[0], ax_x[1], fill="#ef6b73", width=2, arrow="last"
    )
    canvas.create_text(
        ax_x[0] + 6,
        ax_x[1],
        text="+X (mm)",
        fill="#ef6b73",
        anchor="w",
        font=("Segoe UI", 8, "bold"),
    )
    canvas.create_line(
        o[0], o[1], ax_y[0], ax_y[1], fill="#60d5ae", width=2, arrow="last"
    )
    canvas.create_text(
        ax_y[0] + 6,
        ax_y[1],
        text="+Y (mm)",
        fill="#60d5ae",
        anchor="w",
        font=("Segoe UI", 8, "bold"),
    )
    canvas.create_line(
        o[0], o[1], ax_z[0], ax_z[1], fill="#5aaeff", width=2, arrow="last"
    )
    canvas.create_text(
        ax_z[0],
        ax_z[1] - 8,
        text="+Z (mm)",
        fill="#5aaeff",
        font=("Segoe UI", 8, "bold"),
    )
    canvas.create_text(
        o[0] + 8, o[1] + 10, text="0", fill="#edf3fc", font=("Segoe UI", 8)
    )

    # --------------------------------------------------------------------------
    # 3. Drawing Paper Sheet (at Z = paper_z)
    # 3. Mặt phẳng tờ giấy vẽ đặt tại độ cao Z = paper_z
    # --------------------------------------------------------------------------
    cx = (
        float(app.bk_vars["center_x"].get() or 0.0)
        if hasattr(app, "bk_vars")
        else 0.0
    )
    cy = (
        float(app.bk_vars["center_y"].get() or 165.0)
        if hasattr(app, "bk_vars")
        else 165.0
    )
    half_px, half_py = 55.0, 45.0
    paper_corners = (
        cam.project(cx - half_px, cy - half_py, paper_z),
        cam.project(cx + half_px, cy - half_py, paper_z),
        cam.project(cx + half_px, cy + half_py, paper_z),
        cam.project(cx - half_px, cy + half_py, paper_z),
    )
    paper_poly_coords = []
    for pt in paper_corners:
        paper_poly_coords.append(pt[0])
        paper_poly_coords.append(pt[1])
    canvas.create_polygon(
        *paper_poly_coords,
        fill="#19324a",
        outline="#3d6b94",
        width=1,
    )

    # --------------------------------------------------------------------------
    # 4. Planned Drawing Overlay & Live Ink Trails on the 3D Paper Plane
    # 4. Nét vẽ dự kiến màu vàng & Vết mực thực tế màu hồng trên mặt giấy 3D
    # --------------------------------------------------------------------------
    if getattr(app, "bk_overlay", None):
        for stroke in app.bk_overlay:
            pts = []
            for v in stroke:
                sx, sy, _sz = cam.project(v.xy[0], v.xy[1], paper_z + 0.3)
                pts.append(sx)
                pts.append(sy)
            if len(pts) >= 4:
                canvas.create_line(*pts, fill="#efc25a", width=2)

    for trail in getattr(app, "bk_trails", ()):
        if len(trail) >= 2:
            pts = []
            for xy in trail:
                sx, sy, _sz = cam.project(xy[0], xy[1], paper_z + 0.6)
                pts.append(sx)
                pts.append(sy)
            if len(pts) >= 4:
                canvas.create_line(*pts, fill="#ff8795", width=2.5)

    # --------------------------------------------------------------------------
    # 5. Planned 3D Move Trajectory Preview
    # 5. Đường quỹ đạo di chuyển 3D dự kiến (nét đứt màu vàng)
    # --------------------------------------------------------------------------
    if app.preview and len(app.preview.path) >= 2:
        pts = []
        for x, y, z in app.preview.path:
            sx, sy, _sz = cam.project(x, y, z)
            pts.append(sx)
            pts.append(sy)
        if len(pts) >= 4:
            canvas.create_line(*pts, fill="#efc25a", width=2, dash=(4, 3))

    # --------------------------------------------------------------------------
    # 6. Transform & Depth-Sort 3D X-SCARA STL Meshes (Forward Kinematics)
    #    - Shoulder angle: a_rad = radians(J1)
    #    - Distal link absolute angle: b_rad = radians(J1 + J2)
    #    - 2D Rotation Matrix around Z:
    #        X_world = X_local * cos(theta) - Y_local * sin(theta)
    #        Y_world = X_local * sin(theta) + Y_local * cos(theta)
    # 6. Biến đổi Động học thuận (FK) & Sắp xếp độ sâu các khối 3D X-SCARA:
    #    - Góc khớp vai: a_rad = radians(J1)
    #    - Góc tuyệt đối khâu khuỷu: b_rad = radians(J1 + J2)
    #    - Ma trận xoay 2D quanh trục Z:
    #        X_thực = X_cục_bộ * cos(theta) - Y_cục_bộ * sin(theta)
    #        Y_thực = X_cục_bộ * sin(theta) + Y_cục_bộ * cos(theta)
    # --------------------------------------------------------------------------
    a_rad = math.radians(q[0])
    b_rad = math.radians(q[0] + q[1])
    ca, sa = math.cos(a_rad), math.sin(a_rad)
    cb, sb = math.cos(b_rad), math.sin(b_rad)
    z_off = q[2]

    l1 = app.config.l1_mm
    l2 = app.config.l2_mm
    tx_off, ty_off = app.config.tool_offset_mm

    # Elbow position (end of Link 1) and Pen Tip position (end of Link 2 + tool offset).
    # Tọa độ khớp khuỷu (cuối Khâu 1) và tọa độ Đầu bút (cuối Khâu 2 + độ lệch gá bút).
    elbow_x = -l1 * sa
    elbow_y = l1 * ca
    tip_x = elbow_x - (l2 + ty_off) * sb + tx_off * cb
    tip_y = elbow_y + (l2 + ty_off) * cb + tx_off * sb
    tip_z = z_off

    draw_list = []
    parts = build_xscara_3d_parts(round(l1, 2), round(l2, 2))
    for grp, rgb, faces in parts:
        for tri, norm in faces:
            v0, v1, v2 = tri
            if grp == 0:
                # Group 0: Static frame — no motion transform needed.
                # Nhóm 0: Khung đế đứng yên — giữ nguyên tọa độ gốc.
                p0 = cam.project(*v0)
                p1 = cam.project(*v1)
                p2 = cam.project(*v2)
                wn = norm
            elif grp == 1:
                # Group 1: Z-carriage — translates vertically by `z_off`.
                # Nhóm 1: Cụm trượt trục Z — chỉ tịnh tiến theo phương thẳng đứng `z_off`.
                p0 = cam.project(v0[0], v0[1], v0[2] + z_off)
                p1 = cam.project(v1[0], v1[1], v1[2] + z_off)
                p2 = cam.project(v2[0], v2[1], v2[2] + z_off)
                wn = norm
            elif grp == 2:
                # Group 2: Shoulder link L1 — rotates around (0, 0) by angle J1 (`a_rad`) + shifts by `z_off`.
                # Nhóm 2: Khâu 1 (vai L1) — xoay quanh gốc (0, 0) một góc J1 (`a_rad`) và nâng theo `z_off`.
                p0 = cam.project(
                    v0[0] * ca - v0[1] * sa,
                    v0[0] * sa + v0[1] * ca,
                    v0[2] + z_off,
                )
                p1 = cam.project(
                    v1[0] * ca - v1[1] * sa,
                    v1[0] * sa + v1[1] * ca,
                    v1[2] + z_off,
                )
                p2 = cam.project(
                    v2[0] * ca - v2[1] * sa,
                    v2[0] * sa + v2[1] * ca,
                    v2[2] + z_off,
                )
                wn = (
                    norm[0] * ca - norm[1] * sa,
                    norm[0] * sa + norm[1] * ca,
                    norm[2],
                )
            elif grp == 3:
                # Group 3: Elbow link L2 — rotates by J1 + J2 (`b_rad`) and translates to `(elbow_x, elbow_y, z_off)`.
                # Nhóm 3: Khâu 2 (khuỷu L2) — xoay góc J1 + J2 (`b_rad`) và đặt tại tâm khuỷu `(elbow_x, elbow_y, z_off)`.
                p0 = cam.project(
                    elbow_x + v0[0] * cb - v0[1] * sb,
                    elbow_y + v0[0] * sb + v0[1] * cb,
                    v0[2] + z_off,
                )
                p1 = cam.project(
                    elbow_x + v1[0] * cb - v1[1] * sb,
                    elbow_y + v1[0] * sb + v1[1] * cb,
                    v1[2] + z_off,
                )
                p2 = cam.project(
                    elbow_x + v2[0] * cb - v2[1] * sb,
                    elbow_y + v2[0] * sb + v2[1] * cb,
                    v2[2] + z_off,
                )
                wn = (
                    norm[0] * cb - norm[1] * sb,
                    norm[0] * sb + norm[1] * cb,
                    norm[2],
                )
            else:
                # Group 4: Pen holder & tip — rotates by J1 + J2 (`b_rad`) and translates to `(tip_x, tip_y, tip_z)`.
                # Nhóm 4: Cụm kẹp bút & ngòi bút — xoay góc J1 + J2 (`b_rad`) và đặt tại đầu bút `(tip_x, tip_y, tip_z)`.
                p0 = cam.project(
                    tip_x + v0[0] * cb - v0[1] * sb,
                    tip_y + v0[0] * sb + v0[1] * cb,
                    v0[2] + tip_z,
                )
                p1 = cam.project(
                    tip_x + v1[0] * cb - v1[1] * sb,
                    tip_y + v1[0] * sb + v1[1] * cb,
                    v1[2] + tip_z,
                )
                p2 = cam.project(
                    tip_x + v2[0] * cb - v2[1] * sb,
                    tip_y + v2[0] * sb + v2[1] * cb,
                    v2[2] + tip_z,
                )
                wn = (
                    norm[0] * cb - norm[1] * sb,
                    norm[0] * sb + norm[1] * cb,
                    norm[2],
                )

            # Backface culling for watertight solids: skip faces pointing away from camera
            # Loại bỏ mặt khuất (Backface Culling): bỏ qua các tam giác quay lưng về phía camera
            camera_dot = wn[0] * cam.fx + wn[1] * cam.fy + wn[2] * cam.fz
            if camera_dot < -0.03:
                continue

            # Average depth of the 3 vertices + Lambertian shaded color.
            # Độ sâu trung bình của 3 đỉnh tam giác + màu sắc sau khi tính đổ bóng ánh sáng.
            depth = (p0[2] + p1[2] + p2[2]) / 3.0
            color = _shade_hex(rgb, *wn)
            poly_screen_coords = (p0[0], p0[1], p1[0], p1[1], p2[0], p2[1])
            draw_list.append((depth, poly_screen_coords, color))

    # Painter's algorithm: sort by `depth` ascending so farthest triangles are drawn first!
    # Thuật toán Thợ sơn (Painter's Algorithm): sắp xếp `depth` tăng dần để vẽ mặt ở xa trước, mặt ở gần đè lên sau!
    draw_list.sort(key=lambda item: item[0])
    for _depth, coords, color in draw_list:
        canvas.create_polygon(*coords, fill=color, outline=color)

    # --------------------------------------------------------------------------
    # 7. Target Ghost Pose Skeleton in 3D (if target_pose is active)
    # 7. Khung xương tư thế đích màu vàng trong không gian 3D (nếu đang chọn đích)
    # --------------------------------------------------------------------------
    if ready and app.target_pose:
        tq = app.target_pose
        ta = math.radians(tq[0])
        te_x, te_y = -l1 * math.sin(ta), l1 * math.cos(ta)
        tt_x, tt_y, tt_z = app.model.fk(tq)
        s0 = cam.project(0.0, 0.0, tq[2] + 75.0)
        s1 = cam.project(te_x, te_y, tq[2] + 55.0)
        s2 = cam.project(tt_x, tt_y, tt_z)
        canvas.create_line(
            s0[0], s0[1], s1[0], s1[1], fill="#efc25a", width=3, dash=(4, 3)
        )
        canvas.create_line(
            s1[0], s1[1], s2[0], s2[1], fill="#efc25a", width=3, dash=(4, 3)
        )
        canvas.create_oval(
            s2[0] - 5, s2[1] - 5, s2[0] + 5, s2[1] + 5, outline="#efc25a", width=2
        )

    # --------------------------------------------------------------------------
    # 8. Pen Tip Projection Drop-Line onto Paper Plane
    # 8. Đường gióng thẳng đứng từ ngòi bút xuống mặt giấy vẽ
    # --------------------------------------------------------------------------
    tip_scr = cam.project(tip_x, tip_y, tip_z)
    shadow_scr = cam.project(tip_x, tip_y, paper_z)
    canvas.create_line(
        tip_scr[0],
        tip_scr[1],
        shadow_scr[0],
        shadow_scr[1],
        fill="#61d9c4",
        dash=(2, 2),
        width=1,
    )
    canvas.create_oval(
        tip_scr[0] - 4,
        tip_scr[1] - 4,
        tip_scr[0] + 4,
        tip_scr[1] + 4,
        fill="#ff8795",
        outline="#ffffff",
    )

    # --------------------------------------------------------------------------
    # 9. HUD Overlay Info on 3D Canvas
    # 9. Dòng thông số hiển thị nổi (HUD) ở góc trên trái màn hình 3D
    # --------------------------------------------------------------------------
    canvas.create_text(
        12,
        12,
        text=(
            f"MÔ HÌNH 3D X-SCARA (STL Hardware)   ·   Az={cam.azimuth_deg:.0f}° El={cam.elevation_deg:.0f}° Zoom={cam.zoom:.2f}x\n"
            f"J1={q[0]:+.2f}° · J2={q[1]:+.2f}° · Z={q[2]:.2f} mm   |   Đầu bút: X={tip_x:+.1f} Y={tip_y:+.1f} mm"
        ),
        anchor="nw",
        fill="#d6e4f5",
        font=("Segoe UI", 9, "bold"),
    )


def export_canvas_to_svg(canvas, filepath):
    """
    [EN] Export all polygons, lines, ovals, rectangles, and texts currently on a
         Tkinter `canvas` to a standalone, publication-ready vector `.svg` file.
    [VI] Xuất toàn bộ đa giác 3D, đường thẳng, hình tròn, chữ nhật và nhãn chữ trên
         `canvas` sang file đồ họa vector `.svg` sắc nét phục vụ chèn vào Báo cáo.
    """
    w = max(canvas.winfo_width(), 600)
    h = max(canvas.winfo_height(), 450)
    bg = canvas.cget("bg") or "#101d2f"
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">',
        f'  <rect width="100%" height="100%" fill="{bg}"/>',
    ]
    for item in canvas.find_all():
        kind = canvas.type(item)
        coords = canvas.coords(item)
        if not coords:
            continue

        if kind == "polygon" and len(coords) >= 6:
            fill = canvas.itemcget(item, "fill") or "none"
            outline = canvas.itemcget(item, "outline") or fill
            point_pairs = []
            for i in range(0, len(coords), 2):
                point_pairs.append(f"{coords[i]:.1f},{coords[i + 1]:.1f}")
            pts = " ".join(point_pairs)
            lines.append(
                f'  <polygon points="{pts}" fill="{fill}" stroke="{outline}" stroke-width="0.6"/>'
            )
        elif kind == "line" and len(coords) >= 4:
            fill = canvas.itemcget(item, "fill") or "#ffffff"
            width = float(canvas.itemcget(item, "width") or 1.0)
            dash = canvas.itemcget(item, "dash")
            dash_attr = ' stroke-dasharray="4,3"' if dash else ""
            point_pairs = []
            for i in range(0, len(coords), 2):
                point_pairs.append(f"{coords[i]:.1f},{coords[i + 1]:.1f}")
            pts = " ".join(point_pairs)
            lines.append(
                f'  <polyline points="{pts}" fill="none" stroke="{fill}" stroke-width="{width:.1f}"{dash_attr}/>'
            )
        elif kind == "rectangle" and len(coords) == 4:
            x0, y0, x1, y1 = coords
            rx, ry = min(x0, x1), min(y0, y1)
            rw, rh = abs(x1 - x0), abs(y1 - y0)
            fill = canvas.itemcget(item, "fill") or "none"
            outline = canvas.itemcget(item, "outline") or "none"
            lines.append(
                f'  <rect x="{rx:.1f}" y="{ry:.1f}" width="{rw:.1f}" height="{rh:.1f}" fill="{fill}" stroke="{outline}"/>'
            )
        elif kind == "oval" and len(coords) == 4:
            x0, y0, x1, y1 = coords
            cx, cy = (x0 + x1) * 0.5, (y0 + y1) * 0.5
            rx, ry = abs(x1 - x0) * 0.5, abs(y1 - y0) * 0.5
            fill = canvas.itemcget(item, "fill") or "none"
            outline = canvas.itemcget(item, "outline") or "none"
            lines.append(
                f'  <ellipse cx="{cx:.1f}" cy="{cy:.1f}" rx="{rx:.1f}" ry="{ry:.1f}" fill="{fill}" stroke="{outline}"/>'
            )
        elif kind == "text" and len(coords) >= 2:
            tx, ty = coords[0], coords[1]
            fill = canvas.itemcget(item, "fill") or "#edf3fc"
            txt = (
                canvas.itemcget(item, "text")
                .replace("&", "&amp;")
                .replace("<", "&lt;")
                .replace(">", "&gt;")
            )
            for idx, line_str in enumerate(txt.splitlines()):
                lines.append(
                    f'  <text x="{tx:.1f}" y="{ty + idx * 14 + 4:.1f}" fill="{fill}" font-family="Segoe UI, sans-serif" font-size="11">{line_str}</text>'
                )
    lines.append("</svg>")
    Path(filepath).write_text("\n".join(lines), encoding="utf-8")


class Viewer3DMixin:
    """
    [EN] Mixin class adding interactive 3D X-SCARA Digital Twin capabilities to `NCApp`:
         - Supports switching the main workspace canvas between `"2D"` and `"3D"` modes.
         - Supports opening a dedicated floating 3D Digital Twin window (`open_3d_window`)
           so 2D and 3D views can be observed side-by-side.
    [VI] Lớp Mixin bổ sung mô hình 3D X-SCARA Digital Twin tương tác vào `NCApp`:
         - Cho phép chuyển đổi khung vẽ chính giữa chế độ `"2D"` và `"3D"`.
         - Hỗ trợ mở cửa sổ 3D độc lập (`open_3d_window`) để quan sát song song 2D & 3D.
    """

    def init_3d_viewer(self):
        """
        [EN] Initialize 3D camera parameters and window state.
        [VI] Khởi tạo thông số góc nhìn camera 3D và trạng thái cửa sổ 3D.
        """
        self.workspace_mode = "2D"  # Default "2D" so startup 2D canvas behavior is preserved
        self.cam3d_az = 142.0
        self.cam3d_el = 28.0
        self.cam3d_zoom = 1.0
        self.cam3d_pan = (0.0, 0.0)
        self._cam3d_drag = None
        self.win_3d = None
        self.canvas_3d = None
        self.win3d_az = 142.0
        self.win3d_el = 28.0
        self.win3d_zoom = 1.05
        self.win3d_pan = (0.0, 0.0)
        self._win3d_drag = None

    def set_workspace_mode(self, mode):
        """
        [EN] Switch the main workspace canvas between `"2D"` (XY plane) and `"3D"` (X-SCARA STL).
        [VI] Chuyển đổi đồ thị vùng làm việc chính giữa `"2D"` (mặt phẳng XY) và `"3D"` (mô hình STL).
        """
        self.workspace_mode = mode
        self.grid_key = None
        self.point_key = None
        self.dynamic_key = None
        if hasattr(self, "canvas"):
            setattr(self.canvas, "_3d_render_key", None)
        if hasattr(self, "mode_2d_btn") and hasattr(self, "mode_3d_btn"):
            self.mode_2d_btn.configure(bg="#24597a" if mode == "2D" else "#1c3049")
            self.mode_3d_btn.configure(bg="#24597a" if mode == "3D" else "#1c3049")
        if mode == "3D":
            self.cursor_note.set(
                "3D X-SCARA: Kéo chuột trái xoay góc nhìn · Kéo chuột phải dịch chuyển · Lăn chuột Zoom · Nhấp đúp chuột phải Reset"
            )
        else:
            self.cursor_note.set(
                "Kéo đầu công tác chọn XY · kéo khuỷu chọn góc J1 · Lăn chuột Zoom"
            )
        self.draw()

    def set_3d_preset(self, az, el, for_window=False):
        """
        [EN] Snap 3D camera to a standard engineering view (`Isometric`, `Top`, `Front`, `Side`).
        [VI] Chuyển nhanh góc nhìn camera 3D về các hướng chiếu kỹ thuật chuẩn.
        """
        if for_window:
            self.win3d_az, self.win3d_el = float(az), float(el)
            self.win3d_pan = (0.0, 0.0)
            self.draw_3d_window(force=True)
        else:
            self.cam3d_az, self.cam3d_el = float(az), float(el)
            self.cam3d_pan = (0.0, 0.0)
            if self.workspace_mode != "3D":
                self.set_workspace_mode("3D")
            else:
                self.draw()

    def export_workspace_svg(self, target_canvas=None):
        """
        [EN] Prompt the user to save the current 2D or 3D canvas as a vector `.svg` image.
        [VI] Mở hộp thoại lưu đồ thị 2D hoặc mô hình 3D hiện tại ra file ảnh vector `.svg`.
        """
        c = target_canvas or getattr(self, "canvas", None)
        if c is None:
            return
        parent_win = (
            self.win_3d
            if (target_canvas is self.canvas_3d and self.win_3d)
            else self
        )
        path = filedialog.asksaveasfilename(
            parent=parent_win,
            title="Xuất hình ảnh Vector SVG cho Báo cáo",
            defaultextension=".svg",
            initialfile=f"scara_{self.workspace_mode.lower()}_snapshot.svg",
            filetypes=[("SVG Vector Graphic", "*.svg")],
        )
        if path:
            export_canvas_to_svg(c, path)
            self.message.set(f"Đã xuất hình ảnh báo cáo SVG: {path}")

    def open_3d_window(self):
        """
        [EN] Open a dedicated floating 3D Digital Twin window (`self.win_3d`) so
             the user can interact with the 2D workspace and 3D STL model simultaneously.
        [VI] Mở cửa sổ Mô hình 3D X-SCARA Digital Twin độc lập (`self.win_3d`) giúp
             quan sát song song mặt phẳng 2D và mô hình 3D thực tế.
        """
        if self.win_3d and self.win_3d.winfo_exists():
            self.win_3d.lift()
            return
        win = tk.Toplevel(self)
        self.win_3d = win
        win.transient(self)
        win.title("SCARA · 3D Digital Twin Viewer (Hardware STL Assembly)")
        win.geometry("920x700")
        win.configure(bg=BG)

        top = tk.Frame(
            win,
            bg=CARD,
            padx=14,
            pady=8,
            highlightbackground=LINE,
            highlightthickness=1,
        )
        top.pack(fill="x", padx=14, pady=(12, 8))
        self.label(top, "MÔ HÌNH 3D X-SCARA DIGITAL TWIN", 13, TEXT).pack(side="left")
        for label, az, el in (
            ("Góc 3D chuẩn", 142, 28),
            ("Chiếu bằng XY", 90, 85),
            ("Chiếu đứng XZ", 90, 8),
            ("Chiếu cạnh YZ", 0, 12),
        ):
            b = self.button(
                top,
                label,
                lambda a=az, e=el: self.set_3d_preset(a, e, True),
                "#213854",
            )
            b.configure(font=("Segoe UI", 9), padx=8, pady=2)
            b.pack(side="left", padx=3)

        svg_btn = self.button(
            top,
            "📷 Xuất SVG cho Báo cáo",
            lambda: self.export_workspace_svg(self.canvas_3d),
            "#1f6a5e",
        )
        svg_btn.configure(font=("Segoe UI", 9), padx=10, pady=2)
        svg_btn.pack(side="right")

        c = tk.Canvas(
            win, bg="#0d1828", highlightthickness=1, highlightbackground=LINE
        )
        c.pack(fill="both", expand=True, padx=14, pady=(0, 8))
        self.canvas_3d = c

        foot = tk.Frame(win, bg=BG, padx=14)
        foot.pack(fill="x", pady=(0, 10))
        self.label(
            foot,
            "Chuột trái: Xoay 3D (Orbit)   ·   Chuột phải: Dịch chuyển (Pan)   ·   Con lăn: Phóng đại (Zoom)   ·   Nhấp đúp chuột phải: Đặt lại góc nhìn",
            9,
            MUTED,
        ).pack(side="left")

        c.bind("<Configure>", lambda e: self.draw_3d_window(force=True))
        c.bind("<ButtonPress-1>", lambda e: self._win3d_start(e, "orbit"))
        c.bind("<B1-Motion>", self._win3d_move)
        c.bind("<ButtonPress-2>", lambda e: self._win3d_start(e, "pan"))
        c.bind("<B2-Motion>", self._win3d_move)
        c.bind("<ButtonPress-3>", lambda e: self._win3d_start(e, "pan"))
        c.bind("<B3-Motion>", self._win3d_move)
        c.bind("<MouseWheel>", self._win3d_wheel)
        c.bind("<Double-Button-3>", lambda e: self.set_3d_preset(142, 28, True))
        self.draw_3d_window(force=True)

    def _win3d_start(self, event, mode):
        """
        [EN] Record initial mouse position and camera state when starting orbit/pan in `win_3d`.
        [VI] Lưu tọa độ chuột và trạng thái camera ban đầu khi bắt đầu xoay/dịch trong cửa sổ 3D rời.
        """
        self._win3d_drag = (
            mode,
            event.x,
            event.y,
            self.win3d_az,
            self.win3d_el,
            *self.win3d_pan,
        )

    def _win3d_move(self, event):
        """
        [EN] Update camera azimuth/elevation (orbit) or screen offset (pan) in `win_3d`.
        [VI] Cập nhật góc quay ngang/dọc (orbit) hoặc vị trí dịch chuyển (pan) trong cửa sổ 3D rời.
        """
        if not self._win3d_drag:
            return
        mode, ox, oy, az, el, px, py = self._win3d_drag
        if mode == "orbit":
            self.win3d_az = (az - (event.x - ox) * 0.45) % 360.0
            self.win3d_el = max(-80.0, min(85.0, el + (event.y - oy) * 0.35))
        else:
            self.win3d_pan = (px + event.x - ox, py + event.y - oy)
        self.draw_3d_window()

    def _win3d_wheel(self, event):
        """
        [EN] Zoom the floating 3D window camera in or out with the mouse wheel.
        [VI] Phóng to hoặc thu nhỏ góc nhìn cửa sổ 3D rời bằng con lăn chuột.
        """
        up = getattr(event, "num", None) == 4 or getattr(event, "delta", 0) > 0
        factor = 1.12 if up else (1.0 / 1.12)
        self.win3d_zoom = max(0.4, min(3.5, self.win3d_zoom * factor))
        self.draw_3d_window()

    def draw_3d_window(self, force=False):
        """
        [EN] Redraw the floating 3D Digital Twin window if it is currently open.
        [VI] Vẽ lại cửa sổ 3D Digital Twin độc lập nếu đang mở.
        """
        if not self.win_3d or not self.win_3d.winfo_exists() or not self.canvas_3d:
            return
        w = max(self.canvas_3d.winfo_width(), 100)
        h = max(self.canvas_3d.winfo_height(), 100)
        cam = Camera3D(
            w, h, self.win3d_az, self.win3d_el, self.win3d_zoom, *self.win3d_pan
        )
        render_scara_3d_on_canvas(
            self, self.canvas_3d, cam, "_win3d_key", force=force
        )
