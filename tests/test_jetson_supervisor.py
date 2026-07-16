import unittest

from software.companion.docking_state_machine import AbortReason, DockingState
from software.companion.jetson.filtering import ObservationFilterConfig
from software.companion.jetson.range_gate import RangeGateConfig
from software.companion.jetson.sensor_node import JetsonSensorPacket, RangefinderSample
from software.companion.jetson.supervisor_node import JetsonDockingSupervisor, JetsonSupervisorConfig
from software.companion.vision.apriltag_phone_pose import VisionObservation, VisionStatus


class JetsonSupervisorTest(unittest.TestCase):
    def test_tof_range_replaces_apriltag_range_for_snapshot(self):
        supervisor = JetsonDockingSupervisor()
        supervisor.update(_empty_observation(0.0), JetsonSensorPacket(auto_enabled=True))

        output = supervisor.update(
            _valid_observation(0.1, x=0.04, y=0.0, yaw=8.0, vision_range=0.50),
            _packet(0.1, tof_range=0.44),
        )

        self.assertEqual(output.command.state, DockingState.HORIZONTAL_ALIGN)
        self.assertTrue(output.snapshot.range_valid)
        self.assertAlmostEqual(output.snapshot.range_m, 0.44)

    def test_range_mismatch_does_not_enter_alignment(self):
        supervisor = JetsonDockingSupervisor()
        supervisor.update(_empty_observation(0.0), JetsonSensorPacket(auto_enabled=True))

        output = supervisor.update(
            _valid_observation(0.1, x=0.01, y=0.0, yaw=2.0, vision_range=0.50),
            _packet(0.1, tof_range=0.70),
        )

        self.assertEqual(output.observation.status, VisionStatus.RANGE_MISMATCH)
        self.assertFalse(output.snapshot.target_visible)
        self.assertFalse(output.snapshot.range_valid)
        self.assertEqual(output.command.state, DockingState.TARGET_SEARCH)

    def test_invalid_tof_aborts_during_yaw_alignment(self):
        supervisor = JetsonDockingSupervisor()
        supervisor.update(_empty_observation(0.0), JetsonSensorPacket(auto_enabled=True))
        supervisor.update(_valid_observation(0.1, x=0.01, y=0.0, yaw=8.0), _packet(0.1))
        yaw_output = supervisor.update(_valid_observation(0.2, x=0.01, y=0.0, yaw=8.0), _packet(0.2))
        self.assertEqual(yaw_output.command.state, DockingState.YAW_ALIGN)

        output = supervisor.update(
            _valid_observation(0.3, x=0.01, y=0.0, yaw=2.0),
            _packet(0.3, tof_range=None, tof_valid=False),
        )

        self.assertEqual(output.command.state, DockingState.ABORT)
        self.assertEqual(output.command.abort_reason, AbortReason.RANGE_INVALID)

    def test_stale_high_latency_observation_is_rejected(self):
        supervisor = JetsonDockingSupervisor()
        supervisor.update(_empty_observation(0.0), JetsonSensorPacket(auto_enabled=True))

        output = supervisor.update(
            _valid_observation(0.1, latency_ms=350.0),
            _packet(0.1),
        )

        self.assertEqual(output.observation.status, VisionStatus.STALE_FRAME)
        self.assertFalse(output.snapshot.target_visible)
        self.assertEqual(output.command.state, DockingState.TARGET_SEARCH)

    def test_filter_rejects_large_pose_jump(self):
        config = JetsonSupervisorConfig(
            observation_filter=ObservationFilterConfig(max_offset_jump_m=0.05),
            range_gate=RangeGateConfig(max_range_disagreement_m=0.20),
        )
        supervisor = JetsonDockingSupervisor(config)
        supervisor.update(_empty_observation(0.0), JetsonSensorPacket(auto_enabled=True))
        supervisor.update(_valid_observation(0.1, x=0.01, y=0.0), _packet(0.1))

        output = supervisor.update(
            _valid_observation(0.2, x=0.20, y=0.0),
            _packet(0.2),
        )

        self.assertEqual(output.observation.status, VisionStatus.FILTER_JUMP)
        self.assertFalse(output.snapshot.target_visible)
        self.assertFalse(output.snapshot.range_valid)


def _valid_observation(
    time_s,
    *,
    x=0.01,
    y=0.0,
    yaw=2.0,
    vision_range=0.50,
    latency_ms=20.0,
):
    return VisionObservation(
        timestamp_s=time_s,
        target_visible=True,
        tag_id=0,
        target_offset_x_m=x,
        target_offset_y_m=y,
        yaw_error_deg=yaw,
        range_m=vision_range,
        range_valid=True,
        pose_valid=True,
        latency_ms=latency_ms,
        status=VisionStatus.OK,
        calibrated=True,
    )


def _empty_observation(time_s):
    return VisionObservation.empty(time_s)


def _packet(time_s, *, tof_range=0.50, tof_valid=True):
    return JetsonSensorPacket(
        auto_enabled=True,
        rangefinder=RangefinderSample(time_s=time_s, range_m=tof_range, valid=tof_valid),
    )


if __name__ == "__main__":
    unittest.main()
