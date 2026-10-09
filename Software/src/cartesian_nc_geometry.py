"""Shared segment-distance queries for text and vector ink preflight."""

import math


def closest_segment_point(point, start, end):
    delta = tuple(b - a for a, b in zip(start, end))
    length_squared = sum(value * value for value in delta)
    projection = (
        sum((p - a) * value for p, a, value in zip(point, start, delta))
        / length_squared
        if length_squared
        else 0
    )
    fraction = max(0, min(1, projection))
    return tuple(a + fraction * value for a, value in zip(start, delta))


def segment_distance(point, start, end):
    return math.dist(point, closest_segment_point(point, start, end))


def segment_distance_squared(point, start, end):
    return sum(
        (p - closest) ** 2
        for p, closest in zip(point, closest_segment_point(point, start, end))
    )
