"""Hardware-free tests of the actual asynchronous client recovery path."""
import time
import unittest
from unittest.mock import Mock, patch

import torch

from lerobot.async_inference.configs import RobotClientConfig
from lerobot.async_inference.helpers import TimedAction, TimedObservation
from lerobot.async_inference.observation_timing import clock_offset, observation_age
from lerobot.async_inference.robot_client import RobotClient
from lerobot.robots.openarm_follower import OpenArmFollowerConfig


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        robot = Mock()
        robot.observation_features = robot.action_features = {'joint.pos': float}
        robot.get_observation.return_value = {'joint.pos': 0.}
        config = RobotClientConfig(robot=OpenArmFollowerConfig(port='can0'), policy_type='act',
                                  pretrained_name_or_path='test', actions_per_chunk=50)
        with patch('lerobot.async_inference.robot_client.make_robot_from_config', return_value=robot):
            self.client = RobotClient(config)

    def tearDown(self):
        self.client.stop()

    def test_dropped_forced_observation_retries_until_actions_arrive(self):
        self.client.must_go.clear()  # Reproduce the old lost-force condition.
        for _ in range(3):
            self.client.control_loop_observation('test')
            obs = self.client.observation_queue.get_nowait()
            self.assertTrue(obs.must_go)
        self.client.action_queue.put(TimedAction(timestamp=time.time(), timestep=1,
                                                action=torch.tensor([0.])))
        self.client.control_loop_observation('test')
        self.assertFalse(self.client.observation_queue.get_nowait().must_go)

    def test_clock_offset_preserves_real_stale_detection(self):
        offset = clock_offset([('lerobot-server-time', '100.45')], 100., 100.02)
        self.assertAlmostEqual(offset, .44)
        self.assertAlmostEqual(observation_age(100., 100.49,
                               [('lerobot-clock-offset', str(offset))]), .05)
        self.assertGreater(observation_age(99., 100.49,
                           [('lerobot-clock-offset', str(offset))]), .5)
        self.assertEqual(clock_offset([], 100., 101.), 0.)

    def test_rpc_transmits_offset_and_is_bounded(self):
        self.client.stub = Mock()
        self.client._server_clock_offset = .44
        obs = TimedObservation(timestamp=time.time(), timestep=1,
                               observation={'joint.pos': 0.}, must_go=True)
        self.assertTrue(self.client.send_observation(obs))
        kwargs = self.client.stub.SendObservations.call_args.kwargs
        self.assertEqual(kwargs['timeout'], 5)
        self.assertEqual(dict(kwargs['metadata'])['lerobot-clock-offset'], '0.44')


if __name__ == '__main__':
    unittest.main()
