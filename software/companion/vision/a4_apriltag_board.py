"""A4 calibration sheet and four-AprilTag board geometry."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np


A4_WIDTH_M = 0.210
A4_HEIGHT_M = 0.297
DEFAULT_TAG_SIZE_M = 0.050
DEFAULT_TAG_MARGIN_M = 0.015
CHECKERBOARD_INNER_CORNERS = (9, 6)
CHECKERBOARD_SQUARE_SIZE_M = 0.020


@dataclass(frozen=True)
class TagSpec:
    tag_id: int
    name: str
    center_x_m: float
    center_y_m: float


def default_tag_specs(
    *,
    tag_size_m: float = DEFAULT_TAG_SIZE_M,
    margin_m: float = DEFAULT_TAG_MARGIN_M,
) -> tuple[TagSpec, ...]:
    """Return four tag centers in A4-board coordinates.

    Board coordinates use the page center as origin, +x to the right, +y down.
    """

    x = (A4_WIDTH_M / 2.0) - margin_m - (tag_size_m / 2.0)
    y = (A4_HEIGHT_M / 2.0) - margin_m - (tag_size_m / 2.0)
    return (
        TagSpec(tag_id=0, name="top_left", center_x_m=-x, center_y_m=-y),
        TagSpec(tag_id=1, name="top_right", center_x_m=x, center_y_m=-y),
        TagSpec(tag_id=2, name="bottom_right", center_x_m=x, center_y_m=y),
        TagSpec(tag_id=3, name="bottom_left", center_x_m=-x, center_y_m=y),
    )


def tag_center_distance_m(tag: TagSpec) -> float:
    return math.hypot(tag.center_x_m, tag.center_y_m)


def tag_corner_object_points(tag: TagSpec, *, tag_size_m: float = DEFAULT_TAG_SIZE_M) -> np.ndarray:
    """Return top-left, top-right, bottom-right, bottom-left 3D points."""

    half = tag_size_m / 2.0
    return np.array(
        [
            [tag.center_x_m - half, tag.center_y_m - half, 0.0],
            [tag.center_x_m + half, tag.center_y_m - half, 0.0],
            [tag.center_x_m + half, tag.center_y_m + half, 0.0],
            [tag.center_x_m - half, tag.center_y_m + half, 0.0],
        ],
        dtype="float32",
    )


def board_object_points_for_tags(
    tag_ids: Iterable[int],
    *,
    tag_size_m: float = DEFAULT_TAG_SIZE_M,
) -> np.ndarray:
    specs = {tag.tag_id: tag for tag in default_tag_specs(tag_size_m=tag_size_m)}
    points = []
    for tag_id in tag_ids:
        points.extend(tag_corner_object_points(specs[tag_id], tag_size_m=tag_size_m))
    return np.array(points, dtype="float32")


def board_metadata() -> dict[str, object]:
    tags = default_tag_specs()
    return {
        "paper": {"name": "A4", "width_m": A4_WIDTH_M, "height_m": A4_HEIGHT_M},
        "checkerboard": {
            "inner_corners": CHECKERBOARD_INNER_CORNERS,
            "square_size_m": CHECKERBOARD_SQUARE_SIZE_M,
        },
        "apriltag": {
            "family": "tag36h11",
            "tag_size_m": DEFAULT_TAG_SIZE_M,
            "margin_m": DEFAULT_TAG_MARGIN_M,
            "tags": [
                {
                    "id": tag.tag_id,
                    "name": tag.name,
                    "center_x_m": tag.center_x_m,
                    "center_y_m": tag.center_y_m,
                    "distance_to_board_center_m": tag_center_distance_m(tag),
                }
                for tag in tags
            ],
        },
    }
