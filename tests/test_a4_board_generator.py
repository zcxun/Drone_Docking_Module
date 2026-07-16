import json
import tempfile
import unittest
from pathlib import Path

import cv2
from pypdf import PdfReader

from software.companion.vision.a4_apriltag_board import (
    A4_HEIGHT_M,
    A4_WIDTH_M,
    DEFAULT_TAG_SIZE_M,
    board_metadata,
    default_tag_specs,
    tag_center_distance_m,
)
from software.companion.vision.generate_a4_calibration_board import generate_pdf


class A4BoardGeneratorTest(unittest.TestCase):
    def test_board_geometry_defaults(self):
        tags = default_tag_specs()

        self.assertEqual([tag.tag_id for tag in tags], [0, 1, 2, 3])
        self.assertAlmostEqual(A4_WIDTH_M, 0.210)
        self.assertAlmostEqual(A4_HEIGHT_M, 0.297)
        self.assertAlmostEqual(DEFAULT_TAG_SIZE_M, 0.050)
        for tag in tags:
            self.assertAlmostEqual(tag_center_distance_m(tag), tag_center_distance_m(tags[0]))

    def test_metadata_describes_four_tags(self):
        metadata = board_metadata()
        tags = metadata["apriltag"]["tags"]

        self.assertEqual(len(tags), 4)
        self.assertEqual([tag["id"] for tag in tags], [0, 1, 2, 3])

    def test_generated_pdf_and_preview_are_valid(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "board.pdf"
            generate_pdf(output, dpi=100)

            reader = PdfReader(str(output))
            self.assertEqual(len(reader.pages), 2)
            width_pt = float(reader.pages[0].mediabox.width)
            height_pt = float(reader.pages[0].mediabox.height)
            self.assertAlmostEqual(width_pt, 595.28, delta=2.0)
            self.assertAlmostEqual(height_pt, 841.89, delta=2.0)

            metadata = json.loads(output.with_suffix(".metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["dpi"], 100)

            tag_preview = output.with_name("board_page2_apriltags.png")
            image = cv2.imread(str(tag_preview), cv2.IMREAD_GRAYSCALE)
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
            detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
            _corners, ids, _rejected = detector.detectMarkers(image)

            self.assertIsNotNone(ids)
            self.assertEqual(sorted(int(tag_id[0]) for tag_id in ids), [0, 1, 2, 3])


if __name__ == "__main__":
    unittest.main()
