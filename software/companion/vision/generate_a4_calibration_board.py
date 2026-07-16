"""Generate the A4 checkerboard and four-AprilTag printable PDF."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

import cv2
from PIL import Image, ImageDraw

from software.companion.vision.a4_apriltag_board import (
    A4_HEIGHT_M,
    A4_WIDTH_M,
    CHECKERBOARD_INNER_CORNERS,
    CHECKERBOARD_SQUARE_SIZE_M,
    DEFAULT_TAG_SIZE_M,
    board_metadata,
    default_tag_specs,
)


MM_PER_INCH = 25.4
DEFAULT_DPI = 300
DEFAULT_OUTPUT = Path("output/pdf/a4_calibration_and_apriltag_board.pdf")


def generate_pdf(output_path: str | Path = DEFAULT_OUTPUT, *, dpi: int = DEFAULT_DPI) -> dict[str, object]:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)

    checkerboard_page = _new_a4_page(dpi)
    tag_page = _new_a4_page(dpi)
    _draw_checkerboard(checkerboard_page, dpi)
    _draw_apriltag_page(tag_page, dpi)

    checkerboard_page.save(output, "PDF", resolution=dpi, save_all=True, append_images=[tag_page])

    preview_paths = _write_previews(output, checkerboard_page, tag_page)
    metadata = board_metadata()
    metadata["output_pdf"] = str(output)
    metadata["dpi"] = dpi
    metadata["preview_pngs"] = [str(path) for path in preview_paths]
    metadata_path = output.with_suffix(".metadata.json")
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    return metadata


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate the A4 checkerboard + four-AprilTag PDF.")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT), help="Output PDF path.")
    parser.add_argument("--dpi", type=int, default=DEFAULT_DPI, help="Raster PDF DPI.")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    metadata = generate_pdf(args.output, dpi=args.dpi)
    print(metadata["output_pdf"])
    for preview in metadata["preview_pngs"]:
        print(preview)
    return 0


def _new_a4_page(dpi: int) -> Image.Image:
    width_px = round((A4_WIDTH_M * 1000.0 / MM_PER_INCH) * dpi)
    height_px = round((A4_HEIGHT_M * 1000.0 / MM_PER_INCH) * dpi)
    return Image.new("RGB", (width_px, height_px), "white")


def _m_to_px(value_m: float, dpi: int) -> int:
    return round((value_m * 1000.0 / MM_PER_INCH) * dpi)


def _board_to_px(x_m: float, y_m: float, page: Image.Image, dpi: int) -> tuple[int, int]:
    return (
        round((page.width / 2.0) + _m_to_px(x_m, dpi)),
        round((page.height / 2.0) + _m_to_px(y_m, dpi)),
    )


def _draw_checkerboard(page: Image.Image, dpi: int) -> None:
    draw = ImageDraw.Draw(page)
    inner_x, inner_y = CHECKERBOARD_INNER_CORNERS
    squares_x = inner_x + 1
    squares_y = inner_y + 1
    square_px = _m_to_px(CHECKERBOARD_SQUARE_SIZE_M, dpi)
    board_w = squares_x * square_px
    board_h = squares_y * square_px
    x0 = (page.width - board_w) // 2
    y0 = (page.height - board_h) // 2

    for row in range(squares_y):
        for col in range(squares_x):
            if (row + col) % 2 == 0:
                x = x0 + (col * square_px)
                y = y0 + (row * square_px)
                draw.rectangle([x, y, x + square_px, y + square_px], fill="black")

    border = max(2, round(dpi / 100))
    draw.rectangle([x0, y0, x0 + board_w, y0 + board_h], outline="black", width=border)


def _draw_apriltag_page(page: Image.Image, dpi: int) -> None:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
    tag_px = _m_to_px(DEFAULT_TAG_SIZE_M, dpi)
    for tag in default_tag_specs():
        marker = cv2.aruco.generateImageMarker(dictionary, tag.tag_id, tag_px)
        marker_image = Image.fromarray(marker).convert("RGB")
        center_x, center_y = _board_to_px(tag.center_x_m, tag.center_y_m, page, dpi)
        x0 = center_x - (tag_px // 2)
        y0 = center_y - (tag_px // 2)
        page.paste(marker_image, (x0, y0))


def _write_previews(output: Path, checkerboard_page: Image.Image, tag_page: Image.Image) -> tuple[Path, Path]:
    checkerboard_preview = output.with_name(f"{output.stem}_page1_checkerboard.png")
    tag_preview = output.with_name(f"{output.stem}_page2_apriltags.png")
    checkerboard_page.save(checkerboard_preview)
    tag_page.save(tag_preview)
    return checkerboard_preview, tag_preview


if __name__ == "__main__":
    raise SystemExit(main())
