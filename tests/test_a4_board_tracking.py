import tempfile
import unittest
from pathlib import Path

import cv2

from software.companion.vision.a4_board_tracking import A4BoardTracker, CalibrationSession
from software.companion.vision.generate_a4_calibration_board import generate_pdf


class A4BoardTrackingTest(unittest.TestCase):
    def test_tracker_detects_four_printed_tags_without_calibration(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "board.pdf"
            generate_pdf(output, dpi=100)
            frame = cv2.imread(str(output.with_name("board_page2_apriltags.png")), cv2.IMREAD_COLOR)

            result = A4BoardTracker().track(frame)

            self.assertEqual(result["status"], "CALIBRATION_REQUIRED")
            self.assertEqual(result["visible_tag_ids"], [0, 1, 2, 3])
            self.assertFalse(result["height_valid"])
            self.assertIsNotNone(result["board_center_px"])

    def test_calibration_session_requires_minimum_samples(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            session = CalibrationSession(output_path=Path(temp_dir) / "calibration.json")

            result = session.solve()

            self.assertFalse(result["ok"])
            self.assertEqual(result["status"], "NOT_ENOUGH_SAMPLES")
            self.assertEqual(result["sample_count"], 0)


if __name__ == "__main__":
    unittest.main()
