"""Simple connected stroke lettering for the requested names LINH and THOA."""

import math

GLYPHS = {
    "L": (((0, 1), (0, 0), (0.7, 0)),),
    "I": (((0, 1), (0.7, 1)), ((0.35, 1), (0.35, 0)), ((0, 0), (0.7, 0))),
    "N": (((0, 0), (0, 1), (0.7, 0), (0.7, 1)),),
    "H": (((0, 0), (0, 1)), ((0, 0.5), (0.7, 0.5)), ((0.7, 0), (0.7, 1))),
    "T": (((0, 1), (0.7, 1)), ((0.35, 1), (0.35, 0))),
    "O": (
        (
            (0.15, 1),
            (0.55, 1),
            (0.7, 0.8),
            (0.7, 0.2),
            (0.55, 0),
            (0.15, 0),
            (0, 0.2),
            (0, 0.8),
            (0.15, 1),
        ),
    ),
    "A": (((0, 0), (0.35, 1), (0.7, 0)), ((0.14, 0.4), (0.56, 0.4))),
}
# An ellipse replaces the visibly eight-sided O. The drawing compiler still
# validates every chord and pulse against the configured ink tolerance.
GLYPHS["O"] = (
    tuple(
        (
            0.35 + 0.35 * math.cos(math.pi / 2 - 2 * math.pi * i / 48),
            0.5 + 0.5 * math.sin(math.pi / 2 - 2 * math.pi * i / 48),
        )
        for i in range(49)
    ),
)


def word_geometry(word, width, center_x, center_y):
    if word not in ("LINH", "THOA"):
        raise ValueError("Mẫu chữ hỗ trợ LINH hoặc THOA.")
    units = len(word) - 1 + 0.7
    scale = width / units
    left = center_x - width / 2
    bottom = center_y - scale / 2
    strokes = []
    for index, letter in enumerate(word):
        for polyline in GLYPHS[letter]:
            strokes.append(
                tuple(
                    (left + (index + x) * scale, bottom + y * scale)
                    for x, y in polyline
                )
            )
    return width, scale, tuple(strokes)
