"""Exercise the actual modified LeRobot send path without connecting motors."""
import sys
import time
import unittest
from unittest.mock import MagicMock

import numpy as np

from lerobot.robots import make_robot_from_config
from lerobot.robots.bi_openarm_follower import BiOpenArmFollowerConfig
from lerobot.robots.openarm_follower import OpenArmFollowerConfigBase


def make_robot(version='v2'):
    robot = make_robot_from_config(BiOpenArmFollowerConfig(
        left_arm_config=OpenArmFollowerConfigBase(port='can1', side='left',
            hardware_version=version, use_motor_zero=True),
        right_arm_config=OpenArmFollowerConfigBase(port='can0', side='right',
            hardware_version=version, use_motor_zero=True)))
    for side in ('left', 'right'):
        arm = getattr(robot, f'{side}_arm')
        bus = MagicMock()
        bus.is_connected = True
        bus.motors = arm.bus.motors
        bus._last_known_states = {n: {'position': 90. if n == 'joint_4' else 0.,
                                     'velocity': 3.} for n in bus.motors}
        bus._state_received_at = {n: time.monotonic() for n in bus.motors}
        arm.bus = bus
    return robot


class NativeTests(unittest.TestCase):
    def test_zero_disables_inference_velocity_limit(self):
        from lerobot.async_inference.action_interpolator import ActionInterpolator
        from lerobot.async_inference.configs import RobotClientConfig

        config = RobotClientConfig(robot=make_robot().config, policy_type='act',
            pretrained_name_or_path='test', actions_per_chunk=50, max_action_velocity=0)
        self.assertIsNone(config.max_action_velocity)
        smoother = ActionInterpolator({'joint.pos': 0.}, 1 / 30, 1 / 150,
                                      config.max_action_velocity)
        smoother.set_target({'joint.pos': 10.}, 0.)
        self.assertAlmostEqual(smoother.sample(1 / 150)['joint.pos'], 2.)
        self.assertFalse(smoother.limited)

    def test_v2_actual_send_has_project_compensation(self):
        r = make_robot()
        self.assertEqual(type(r).__name__, 'BiOpenArmFollower')
        action = {f'{s}_{n}.pos': state['position'] + .2
                  for s in ('left', 'right')
                  for n, state in getattr(r, f'{s}_arm').bus._last_known_states.items()}
        sent = r.send_action(action)
        self.assertEqual(len(sent), 16)
        for s in ('left', 'right'):
            arm = getattr(r, f'{s}_arm')
            commands = arm.bus._mit_control_batch.call_args.args[0]
            expected = arm.joint_mit_controller.compensator.compute({
                'qpos': np.radians([0, 0, 0, 90, 0, 0, 0, 0]),
                'qvel': np.radians([3.] * 8)})
            actual = [commands[f'joint_{i}'][4] for i in range(1, 8)]
            np.testing.assert_allclose(actual, expected)
            self.assertGreater(np.max(np.abs(actual)), .01)
            self.assertEqual([commands[f'joint_{i}'][0] for i in range(1, 8)],
                             arm.joint_mit_controller.kp)
            self.assertNotIn('gripper', commands)
            arm.bus.posforce_control.assert_called_once()
        self.assertNotIn('lerobot.robots.bi_openarm_follower.ros_mit_runtime', sys.modules)

    def test_stale_feedback_sends_nothing(self):
        r = make_robot()
        r.left_arm.bus._state_received_at = {}
        with self.assertRaisesRegex(RuntimeError, 'fresh feedback'):
            r.send_action({'left_joint_1.pos': 0.})
        r.left_arm.bus._mit_control_batch.assert_not_called()

    def test_nonfinite_and_large_targets(self):
        r = make_robot()
        with self.assertRaisesRegex(ValueError, 'Non-finite'):
            r.left_arm.joint_mit_controller.prepare({'joint_1': float('nan')}, r.left_arm.bus)
        target, torque = r.left_arm.joint_mit_controller.prepare(
            {'joint_1': 60.}, r.left_arm.bus)
        self.assertEqual(target['joint_1'], 60.)
        self.assertEqual(len(torque), 7)
        # Exercise the actual send path: one large target must not reset its peers.
        sent = r.left_arm.send_action({'joint_1.pos': 60., 'joint_2.pos': 5.})
        self.assertEqual(sent, {'joint_1.pos': 60., 'joint_2.pos': 5.})
        commands = r.left_arm.bus._mit_control_batch.call_args.args[0]
        self.assertEqual(commands['joint_1'][2], 60.)
        self.assertEqual(commands['joint_2'][2], 5.)

    def test_legacy_stays_without_compensation(self):
        r = make_robot('legacy')
        self.assertIsNone(r.left_arm.joint_mit_controller)
        r.left_arm.send_action({'joint_1.pos': 0.})
        self.assertEqual(r.left_arm.bus._mit_control_batch.call_args.args[0]['joint_1'][4], 0.)


if __name__ == '__main__':
    unittest.main()
