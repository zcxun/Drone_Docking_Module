import math
import unittest

from software.companion.vision.apriltag_phone_pose import VisionObservation, VisionStatus
from software.companion.vision.vision_to_sensor_snapshot import observation_to_sensor_snapshot


class AprilTagOutputContractTest(unittest.TestCase):
    def test_calibrated_observation_maps_to_sensor_snapshot(self):
        observation = VisionObservation(
            timestamp_s=12.5,
            target_visible=True,
            tag_id=0,
            target_offset_x_m=0.02,
            target_offset_y_m=-0.01,
            yaw_error_deg=3.0,
            range_m=0.55,
            range_valid=True,
            pose_valid=True,
            status=VisionStatus.OK,
            calibrated=True,
        )

        snapshot = observation_to_sensor_snapshot(observation, auto_enabled=True)

        self.assertEqual(snapshot.time_s, 12.5)
        self.assertTrue(snapshot.auto_enabled)
        self.assertTrue(snapshot.target_visible)
        self.assertEqual(snapshot.target_offset_x_m, 0.02)
        self.assertEqual(snapshot.target_offset_y_m, -0.01)
        self.assertEqual(snapshot.yaw_error_deg, 3.0)
        self.assertEqual(snapshot.range_m, 0.55)
        self.assertTrue(snapshot.range_valid)

    def test_uncalibrated_estimate_is_not_valid_for_state_machine_by_default(self):
        observation = VisionObservation(
            timestamp_s=2.0,
            target_visible=True,
            tag_id=0,
            target_offset_x_m=0.04,
            target_offset_y_m=0.02,
            yaw_error_deg=6.0,
            range_m=0.50,
            range_valid=False,
            pose_valid=False,
            status=VisionStatus.UNCALIBRATED_ESTIMATE,
            calibrated=False,
        )

        snapshot = observation_to_sensor_snapshot(observation, auto_enabled=True)

        self.assertFalse(snapshot.target_visible)
        self.assertIsNone(snapshot.target_offset_x_m)
        self.assertIsNone(snapshot.target_offset_y_m)
        self.assertIsNone(snapshot.yaw_error_deg)
        self.assertIsNone(snapshot.range_m)
        self.assertFalse(snapshot.range_valid)

    def test_uncalibrated_estimate_can_be_allowed_for_tabletop_dry_run(self):
        observation = VisionObservation(
            timestamp_s=2.0,
            target_visible=True,
            tag_id=0,
            target_offset_x_m=0.04,
            target_offset_y_m=0.02,
            yaw_error_deg=6.0,
            range_m=0.50,
            range_valid=False,
            pose_valid=False,
            status=VisionStatus.UNCALIBRATED_ESTIMATE,
            calibrated=False,
        )

        snapshot = observation_to_sensor_snapshot(
            observation,
            auto_enabled=True,
            allow_uncalibrated_estimate=True,
        )

        self.assertTrue(snapshot.target_visible)
        self.assertEqual(snapshot.target_offset_x_m, 0.04)
        self.assertEqual(snapshot.target_offset_y_m, 0.02)
        self.assertEqual(snapshot.yaw_error_deg, 6.0)
        self.assertEqual(snapshot.range_m, 0.50)
        self.assertTrue(snapshot.range_valid)

    def test_non_finite_values_are_rejected(self):
        observation = VisionObservation(
            timestamp_s=3.0,
            target_visible=True,
            tag_id=0,
            target_offset_x_m=math.nan,
            target_offset_y_m=0.0,
            yaw_error_deg=0.0,
            range_m=0.50,
            range_valid=True,
            pose_valid=True,
            status=VisionStatus.OK,
            calibrated=True,
        )

        snapshot = observation_to_sensor_snapshot(observation, auto_enabled=True)

        self.assertFalse(snapshot.target_visible)
        self.assertIsNone(snapshot.target_offset_x_m)
        self.assertFalse(snapshot.range_valid)


if __name__ == "__main__":
    unittest.main()
