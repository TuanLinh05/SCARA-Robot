"""Shared segment-distance queries for text and vector ink preflight.

[EN] This module provides fundamental computational geometry utilities to measure
the shortest Euclidean distance between a query point P and a finite line segment AB.
During CAD/CAM preflight, the drawing compiler simulates every stepper motor pulse
(via Forward Kinematics) and uses these functions to verify that the physical pen tip
never deviates from the ideal vector stroke by more than `tolerance_mm`.

[VI] Mô-đun này cung cấp các hàm hình học tính toán cơ bản để đo khoảng cách
Euclid ngắn nhất từ một điểm truy vấn P đến đoạn thẳng hữu hạn AB.
Trong quá trình kiểm tra trước khi vẽ (preflight), trình biên dịch mô phỏng từng xung
động cơ bước (qua Động học Thuận - FK) và dùng các hàm này để đảm bảo đầu bút thực tế
không bao giờ lệch khỏi nét vẽ lý tưởng quá ngưỡng dung sai `tolerance_mm`.
"""

import math


def closest_segment_point(point, start, end):
    """Find the point on the finite line segment [start, end] closest to `point`.

    [EN] Mathematical formulation (Vector Projection):
    Let A = `start`, B = `end`, and P = `point`.
    1. Segment direction vector:  v = B - A
    2. Squared segment length:    L^2 = v . v = ||B - A||^2
    3. Unclamped projection parameter `t` (dot product normalized by L^2):
           t = ((P - A) . v) / L^2
       - If t < 0: the orthogonal projection falls before A (closest is A).
       - If t > 1: the orthogonal projection falls past B (closest is B).
       - If 0 <= t <= 1: the projection lies directly on the segment AB.
    4. Clamp `t` to [0, 1] -> `fraction = max(0, min(1, t))`
    5. Closest point C = A + fraction * v

    [VI] Công thức toán học (Hình chiếu Vector):
    Gọi A = `start`, B = `end`, và P = `point`.
    1. Vector chỉ phương của đoạn thẳng: v = B - A
    2. Bình phương độ dài đoạn thẳng:    L^2 = v . v = ||B - A||^2
    3. Tham số hình chiếu chưa giới hạn `t` (tích vô hướng chuẩn hóa theo L^2):
           t = ((P - A) . v) / L^2
       - Nếu t < 0: hình chiếu vuông góc nằm trước điểm đầu A (điểm gần nhất là A).
       - Nếu t > 1: hình chiếu vuông góc vượt quá điểm cuối B (điểm gần nhất là B).
       - Nếu 0 <= t <= 1: hình chiếu nằm ngay trên đoạn thẳng AB.
    4. Giới hạn (clamp) `t` vào đoạn [0, 1] -> `fraction = max(0, min(1, t))`
    5. Điểm gần nhất C = A + fraction * v

    [DEBUG TIP] If `start == end` (a zero-length degenerate segment), `length_squared`
    is 0, so `projection` safely defaults to 0 and returns `start`.
    """
    # [EN] Unpack 2D coordinates of query point P = (px, py), start A = (ax, ay), and end B = (bx, by).
    # [VI] Tách tọa độ 2D của điểm truy vấn P = (px, py), điểm đầu A = (ax, ay) và điểm cuối B = (bx, by).
    px, py = point
    ax, ay = start
    bx, by = end

    # [EN] Step 1: Compute segment direction vector (dx, dy) = B - A.
    # [VI] Bước 1: Tính vector chỉ phương của đoạn thẳng (dx, dy) = B - A.
    dx = bx - ax
    dy = by - ay

    # [EN] Step 2: Compute squared Euclidean length L^2 = dx^2 + dy^2.
    # [VI] Bước 2: Tính bình phương độ dài đoạn thẳng L^2 = dx^2 + dy^2.
    length_squared = dx * dx + dy * dy

    # [EN] Step 3: Project vector (P - A) onto (dx, dy) using dot product,
    #      guarding against division by zero when start and end are identical.
    # [VI] Bước 3: Chiếu vector (P - A) lên (dx, dy) bằng tích vô hướng,
    #      kiểm tra tránh lỗi chia cho 0 khi điểm đầu và điểm cuối trùng nhau.
    if length_squared:
        dot_product = (px - ax) * dx + (py - ay) * dy
        projection = dot_product / length_squared
    else:
        projection = 0

    # [EN] Step 4: Clamp the normalized parameter into the finite segment range [0, 1].
    # [VI] Bước 4: Giới hạn hệ số chiếu vào trong phạm vi đoạn thẳng hữu hạn [0, 1].
    if projection <= 0:
        fraction = 0
    elif projection >= 1:
        fraction = 1
    else:
        fraction = projection

    # [EN] Step 5: Linearly interpolate the closest point on the segment: C = A + fraction * (B - A).
    # [VI] Bước 5: Nội suy tuyến tính tọa độ điểm gần nhất trên đoạn thẳng: C = A + fraction * (B - A).
    closest_x = ax + fraction * dx
    closest_y = ay + fraction * dy
    return (closest_x, closest_y)


def segment_distance(point, start, end):
    """Return the Euclidean distance (in mm) from `point` to segment [start, end].

    [EN] First finds the closest point C on the segment [start, end], then computes
    the straight-line distance ||point - C||_2 using `math.dist`.
    Used directly during text and custom vector preflight error checks.

    [VI] Tìm điểm gần nhất C trên đoạn thẳng [start, end], sau đó tính khoảng cách
    Euclid ||point - C||_2 (đơn vị mm) bằng `math.dist`.
    Được gọi trực tiếp khi kiểm tra sai số quỹ đạo cho chữ viết và vector tùy chỉnh.
    """
    # [EN] Find the projection point on the segment and measure distance to `point`.
    # [VI] Tìm điểm hình chiếu gần nhất trên đoạn thẳng và đo khoảng cách tới `point`.
    closest = closest_segment_point(point, start, end)
    return math.dist(point, closest)


def segment_distance_squared(point, start, end):
    """Return the squared Euclidean distance from `point` to segment [start, end].

    [EN] Avoids the `math.sqrt()` call so bounding-volume hierarchy (BVH) tree searches
    in `cartesian_nc_shapes.py` can compare distances rapidly and only take the square
    root once at the very end.

    [VI] Trả về bình phương khoảng cách từ `point` đến đoạn thẳng [start, end].
    Giúp tránh phép tính căn bậc hai `math.sqrt()` lặp lại nhiều lần khi duyệt cây
    phân cấp hộp bao (BVH) trong `cartesian_nc_shapes.py`, tăng tốc độ kiểm tra.
    """
    # [EN] Step 1: Locate the closest point C = (cx, cy) on the segment [start, end].
    # [VI] Bước 1: Tìm điểm gần nhất C = (cx, cy) trên đoạn thẳng [start, end].
    cx, cy = closest_segment_point(point, start, end)

    # [EN] Step 2: Compute squared Euclidean distance (px - cx)^2 + (py - cy)^2.
    # [VI] Bước 2: Tính bình phương khoảng cách (px - cx)^2 + (py - cy)^2.
    err_x = point[0] - cx
    err_y = point[1] - cy
    return err_x * err_x + err_y * err_y
