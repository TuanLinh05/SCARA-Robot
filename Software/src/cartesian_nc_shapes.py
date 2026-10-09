"""Vector templates, exact bounds and bounded ink-distance queries on the PC."""

from dataclasses import dataclass
from functools import lru_cache
import math
from cartesian_nc_curves import flatten_cubic, ellipse_arc, ellipse_radial_error
from cartesian_nc_geometry import segment_distance_squared as line_distance_squared

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
SHAPES = frozenset(k for k, v in PATTERNS.items() if v[1] == "Hình vẽ")


@dataclass(frozen=True)
class Piece:
    kind: str
    data: tuple


def cubic_value(p, t):
    v = 1 - t
    return tuple(
        v**3 * a + 3 * v * v * t * b + 3 * v * t * t * c + t**3 * d
        for a, b, c, d in zip(*p)
    )


def piece_bounds(piece):
    if piece.kind == "ellipse":
        cx, cy, rx, ry = piece.data
        return (cx - rx, cy - ry, cx + rx, cy + ry)
    points = list(
        piece.data if piece.kind == "line" else (piece.data[0], piece.data[3])
    )
    if piece.kind == "cubic":
        for values in zip(*piece.data):
            a, b, c, d = values
            qa = 3 * (-a + 3 * b - 3 * c + d)
            qb = 2 * (3 * a - 6 * b + 3 * c)
            qc = 3 * (b - a)
            if abs(qa) < 1e-12:
                roots = () if abs(qb) < 1e-12 else (-qc / qb,)
            else:
                disc = qb * qb - 4 * qa * qc
                roots = (
                    ()
                    if disc < 0
                    else (
                        (-qb + math.sqrt(disc)) / (2 * qa),
                        (-qb - math.sqrt(disc)) / (2 * qa),
                    )
                )
            points.extend(cubic_value(piece.data, t) for t in roots if 0 < t < 1)
    return (
        min(p[0] for p in points),
        min(p[1] for p in points),
        max(p[0] for p in points),
        max(p[1] for p in points),
    )


def polygon(points):
    return (tuple(Piece("line", (a, b)) for a, b in zip(points, points[1:])),)


@lru_cache(maxsize=8)
def shape_template(pattern):
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
        points = tuple(
            (
                r * math.cos(math.pi / 2 - i * math.pi / 5),
                r * math.sin(math.pi / 2 - i * math.pi / 5),
            )
            for i in range(10)
            for r in ((0.5 if i % 2 == 0 else 0.205),)
        )
        return polygon((*points, points[0]))
    if pattern == "HEART":
        return (
            (
                Piece("cubic", ((0, -0.5), (-0.12, -0.32), (-0.5, -0.12), (-0.5, 0.1))),
                Piece("cubic", ((-0.5, 0.1), (-0.5, 0.62), (-0.07, 0.58), (0, 0.25))),
                Piece("cubic", ((0, 0.25), (0.07, 0.58), (0.5, 0.62), (0.5, 0.1))),
                Piece("cubic", ((0.5, 0.1), (0.5, -0.12), (0.12, -0.32), (0, -0.5))),
            ),
        )
    if pattern == "FLOWER":
        strokes = []
        for i in range(6):
            angle = math.pi / 2 + i * math.pi / 3

            def rotate(p):
                return (
                    p[0] * math.cos(angle) - p[1] * math.sin(angle),
                    p[0] * math.sin(angle) + p[1] * math.cos(angle),
                )

            strokes.append(
                tuple(
                    Piece("cubic", tuple(map(rotate, p)))
                    for p in (
                        ((0.1, 0), (0.2, 0.18), (0.5, 0.14), (0.5, 0)),
                        ((0.5, 0), (0.5, -0.14), (0.2, -0.18), (0.1, 0)),
                    )
                )
            )
        strokes.append((Piece("ellipse", (0, 0, 0.07, 0.07)),))
        return tuple(strokes)
    raise ValueError("Mẫu hình không hợp lệ.")


@lru_cache(maxsize=32)
def scaled_pieces(pattern, width, cx, cy):
    strokes = shape_template(pattern)
    boxes = [piece_bounds(p) for line in strokes for p in line]
    x0 = min(b[0] for b in boxes)
    x1 = max(b[2] for b in boxes)
    y0 = min(b[1] for b in boxes)
    y1 = max(b[3] for b in boxes)
    scale = width / (x1 - x0)
    ox = (x0 + x1) / 2
    oy = (y0 + y1) / 2

    def point(p):
        return (cx + (p[0] - ox) * scale, cy + (p[1] - oy) * scale)

    def convert(piece):
        if piece.kind == "ellipse":
            x, y, rx, ry = piece.data
            return Piece("ellipse", (*point((x, y)), rx * scale, ry * scale))
        return Piece(piece.kind, tuple(map(point, piece.data)))

    return (
        width,
        (y1 - y0) * scale,
        tuple(tuple(convert(p) for p in line) for line in strokes),
    )


def shape_dimensions(settings):
    return scaled_pieces(
        settings.pattern, settings.size_mm, settings.center_x, settings.center_y
    )[:2]


def piece_vertices(piece, step, error):
    if piece.kind == "ellipse":
        cx, cy, rx, ry = piece.data
        start = (cx, cy + ry)
        return start, ellipse_arc(
            cx, cy, rx, ry, math.pi / 2, -3 * math.pi / 2, step, error
        )
    if piece.kind == "cubic":
        return piece.data[0], flatten_cubic(piece.data, step, error)
    a, b = piece.data
    n = max(1, math.ceil(math.dist(a, b) / step))
    return a, tuple(
        tuple(x + (y - x) * i / n for x, y in zip(a, b)) for i in range(1, n + 1)
    )


def shape_vertices(settings):
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
            vertices.extend((p, part) for p in ends)
        # All supplied motifs are closed outlines; avoid a floating-point seam.
        vertices[-1] = (vertices[0][0], vertices[-1][1])
        result.append(tuple(vertices))
    return tuple(result)


def box_distance_squared(p, box):
    return (
        max(box[0] - p[0], 0, p[0] - box[2]) ** 2
        + max(box[1] - p[1], 0, p[1] - box[3]) ** 2
    )


def distance_tree(segments):
    box = (
        min(min(a[0], b[0]) for a, b in segments),
        min(min(a[1], b[1]) for a, b in segments),
        max(max(a[0], b[0]) for a, b in segments),
        max(max(a[1], b[1]) for a, b in segments),
    )
    if len(segments) <= 6:
        return box, segments, None, None
    mid = len(segments) // 2
    return box, (), distance_tree(segments[:mid]), distance_tree(segments[mid:])


def tree_distance(p, node, best=math.inf):
    box, segments, left, right = node
    if box_distance_squared(p, box) > best:
        return best
    if segments:
        return min(best, *(line_distance_squared(p, a, b) for a, b in segments))
    if box_distance_squared(p, left[0]) > box_distance_squared(p, right[0]):
        left, right = right, left
    return tree_distance(p, right, tree_distance(p, left, best))


@lru_cache(maxsize=64)
def curve_oracle(piece, error):
    points = (piece.data[0], *flatten_cubic(piece.data, 0.15, error))
    return distance_tree(tuple(zip(points, points[1:])))


def shape_ink_distance(xy, part, settings):
    _, i, j = part.split("_")
    piece = scaled_pieces(
        settings.pattern, settings.size_mm, settings.center_x, settings.center_y
    )[2][int(i)][int(j)]
    if piece.kind == "line":
        return math.sqrt(line_distance_squared(xy, *piece.data))
    if piece.kind == "ellipse":
        return ellipse_radial_error(xy, *piece.data)
    error = settings.tolerance_mm / 100
    # Fine reference polyline's Hausdorff bound is added conservatively.
    return math.sqrt(tree_distance(xy, curve_oracle(piece, error))) + error
