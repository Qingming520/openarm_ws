"""Receive-path regression tests with queued CAN frames, without hardware."""
import time
import unittest
from unittest.mock import Mock, patch

import can

from lerobot.motors.damiao import DamiaoMotorsBus
from lerobot.motors.damiao.tables import MotorType
from test_native_backend import make_robot


class FeedbackTests(unittest.TestCase):
    def setUp(self):
        self.bus = object.__new__(DamiaoMotorsBus)
        self.bus.canbus = Mock()
        self.bus.use_can_fd = True
        self.bus._recv_id_to_motor = {0x11: 'joint_1', 0x18: 'gripper'}
        self.bus._motor_types = {'joint_1': MotorType.DM4310, 'gripper': MotorType.DM4310}
        self.bus._last_known_states = {}
        self.bus._state_received_at = {}

    def frame(self, identifier, **kwargs):
        return can.Message(arbitration_id=identifier, data=bytes([1, 128, 0, 128, 0, 0, 30, 30]),
                           is_extended_id=False, **kwargs)

    def test_gripper_wait_preserves_arm_feedback(self):
        arm, grip = self.frame(0x11), self.frame(0x18)
        self.bus.canbus.recv.side_effect = [arm, grip]
        self.assertIs(self.bus._recv_motor_response(0x18, timeout=.01), grip)
        self.assertIn('joint_1', self.bus._last_known_states)
        self.assertLess(time.monotonic() - self.bus._state_received_at['joint_1'], .1)

    def test_arm_batch_preserves_gripper_feedback(self):
        grip, arm = self.frame(0x18), self.frame(0x11)
        self.bus.canbus.recv.side_effect = [grip, arm]
        result = self.bus._recv_all_responses([0x11], timeout=.01)
        self.assertIs(result[0x11], arm)
        self.assertIn('gripper', self.bus._last_known_states)

    def test_unknown_and_error_frames_not_cached(self):
        self.bus._cache_unsolicited_feedback(self.frame(0x55))
        self.bus._cache_unsolicited_feedback(self.frame(0x11, is_error_frame=True))
        self.assertFalse(self.bus._state_received_at)

    def test_handshake_decodes_response_instead_of_enable_packet(self):
        arm = self.frame(0x11)
        self.bus.motors = {'joint_1': Mock()}
        self.bus._get_motor_id = Mock(return_value=1)
        self.bus._get_motor_recv_id = Mock(return_value=0x11)
        self.bus.canbus.recv.side_effect = [None, arm]
        with patch('lerobot.motors.damiao.damiao.time.sleep'):
            self.bus._handshake()
        expected = self.bus._decode_motor_state(arm.data, MotorType.DM4310)[0]
        self.assertEqual(self.bus._last_known_states['joint_1']['position'], expected)

    def test_lost_refresh_is_retried_only_for_stale_axis(self):
        robot = make_robot()
        arm = robot.left_arm
        arm.bus._state_received_at['gripper'] = time.monotonic() - 1
        queries = []

        def refresh(*, motors):
            queries.append(list(motors))
            if len(queries) == 2:
                arm.bus._state_received_at['gripper'] = time.monotonic()

        arm.bus.sync_read_all_states.side_effect = refresh
        targets, torque = arm.joint_mit_controller.prepare({'joint_1': 60.}, arm.bus)
        self.assertEqual(queries, [['gripper'], ['gripper']])
        self.assertEqual(targets['joint_1'], 60.)
        self.assertEqual(len(torque), 7)

    def test_persistent_missing_feedback_names_axis_and_sends_nothing(self):
        robot = make_robot()
        arm = robot.left_arm
        arm.bus._state_received_at['joint_3'] = time.monotonic() - 1
        with self.assertRaisesRegex(RuntimeError, 'side=left, port=can1, stale: joint_3='):
            arm.send_action({'joint_1.pos': 60.})
        self.assertEqual(arm.bus.sync_read_all_states.call_count, 2)
        arm.bus._mit_control_batch.assert_not_called()


if __name__ == '__main__':
    unittest.main()
