import unittest

from software.companion.docking_state_machine import ControlCommand, DockingState, SensorSnapshot
from software.companion.vision.apriltag_phone_pose import VisionObservation, VisionStatus
from software.companion.vision.guidance_adapter import (
    GuidanceConfig,
    build_guidance_payload,
    camera_offsets_to_body,
)


class GuidanceAdapterTest(unittest.TestCase):
    def test_default_camera_orientation_maps_image_top_to_forward(self):
        offset = camera_offsets_to_body(0.05, -0.12, GuidanceConfig())

        self.assertAlmostEqual(offset.forward_m, 0.12)
        self.assertAlmostEqual(offset.right_m, 0.05)

    def test_camera_orientation_can_be_rotated(self):
        offset = camera_offsets_to_body(0.08, 0.02, GuidanceConfig(camera_forward="right"))

        self.assertAlmostEqual(offset.forward_m, 0.08)
        self.assertAlmostEqual(offset.right_m, 0.02)

    def test_payload_uses_human_readable_forward_right_guidance(self):
        payload = build_guidance_payload(
            _observation(x=0.04, y=-0.12, yaw=3.0, calibrated=False),
            _snapshot(x=0.04, y=-0.12, yaw=3.0, range_m=0.68),
            _command(vx=-0.024, vy=0.06, yaw_rate=-1.5),
        )

        self.assertIn("往前 12 cm", payload.horizontal_instruction)
        self.assertIn("往右 4 cm", payload.horizontal_instruction)
        self.assertIn("建議下降 18 cm", payload.height_instruction)
        self.assertIn("順時針轉 3.0°", payload.yaw_instruction)
        self.assertFalse(payload.control_allowed)
        self.assertIn("尚未校正", payload.control_note)

    def test_attitude_warning_is_primary_when_tilt_is_unsafe(self):
        payload = build_guidance_payload(
            _observation(calibrated=True),
            _snapshot(roll=12.0, pitch=2.0),
            _command(),
        )

        self.assertEqual(payload.primary_instruction, "傾斜過大，先扶正")
        self.assertIn("roll 12.0°", payload.attitude_instruction)
        self.assertFalse(payload.control_allowed)

    def test_command_preview_matches_control_command_values(self):
        payload = build_guidance_payload(
            _observation(calibrated=True),
            _snapshot(),
            _command(vx=0.06, vy=-0.04, vz=-0.08, yaw_rate=2.5),
        )

        self.assertIn("往前慢移 0.06 m/s", payload.command_preview)
        self.assertIn("往左慢移 0.04 m/s", payload.command_preview)
        self.assertIn("下降 0.08 m/s", payload.command_preview)
        self.assertIn("逆時針修正 2.5 deg/s", payload.command_preview)

    def test_missing_tag_reports_search_instruction(self):
        observation = VisionObservation.empty(1.0, status=VisionStatus.TAG_NOT_FOUND)
        snapshot = SensorSnapshot(time_s=1.0, target_visible=False)
        payload = build_guidance_payload(observation, snapshot, _command())

        self.assertEqual(payload.primary_instruction, "找不到定位標記")
        self.assertIn("找不到定位標記", payload.control_note)


def _observation(x=0.0, y=0.0, yaw=0.0, range_m=0.50, calibrated=True):
    return VisionObservation(
        timestamp_s=1.0,
        target_visible=True,
        tag_id=0,
        target_offset_x_m=x,
        target_offset_y_m=y,
        yaw_error_deg=yaw,
        range_m=range_m,
        range_valid=True,
        pose_valid=calibrated,
        status=VisionStatus.OK if calibrated else VisionStatus.UNCALIBRATED_ESTIMATE,
        calibrated=calibrated,
    )


def _snapshot(x=0.0, y=0.0, yaw=0.0, range_m=0.50, roll=0.0, pitch=0.0):
    return SensorSnapshot(
        time_s=1.0,
        target_visible=True,
        target_offset_x_m=x,
        target_offset_y_m=y,
        yaw_error_deg=yaw,
        range_m=range_m,
        range_valid=True,
        roll_deg=roll,
        pitch_deg=pitch,
    )


def _command(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
    return ControlCommand(
        state=DockingState.HORIZONTAL_ALIGN,
        mode="GUIDED",
        horizontal_velocity_mps=(vx, vy),
        vertical_velocity_mps=vz,
        yaw_rate_dps=yaw_rate,
    )


if __name__ == "__main__":
    unittest.main()

