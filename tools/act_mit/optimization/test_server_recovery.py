"""Run on the GPU host; no model or hardware is needed."""
import pickle
import time
import unittest
from unittest.mock import Mock, patch

from lerobot.async_inference.configs import PolicyServerConfig
from lerobot.async_inference.helpers import TimedObservation
from lerobot.async_inference.policy_server import PolicyServer
from lerobot.transport import services_pb2
from lerobot.transport.utils import send_bytes_in_chunks


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.server = PolicyServer(PolicyServerConfig())
        self.server.lerobot_features = {'observation.state':
            {'dtype': 'float32', 'shape': [1], 'names': ['joint.pos']}}
        self.previous = TimedObservation(timestamp=100., timestep=1, observation={'joint.pos': 0.})
        self.server.last_processed_obs = self.previous

    def obs(self, timestamp=100., timestep=2, must_go=False):
        return TimedObservation(timestamp=timestamp, timestep=timestep,
                                observation={'joint.pos': 0.}, must_go=must_go)

    def test_similarity_off_by_default_and_can_be_enabled(self):
        self.assertFalse(self.server.config.filter_similar_observations)
        self.assertTrue(self.server._enqueue_observation(self.obs()))
        self.server.observation_queue.get_nowait()
        self.server.config.filter_similar_observations = True
        self.assertFalse(self.server._enqueue_observation(self.obs()))

    def test_duplicate_timestep_still_rejected(self):
        self.server._predicted_timesteps.add(2)
        self.assertFalse(self.server._enqueue_observation(self.obs()))
        self.assertTrue(self.server._enqueue_observation(self.obs(must_go=True)))

    def test_latest_pending_preserves_force(self):
        self.server._enqueue_observation(self.obs(must_go=True))
        newest = self.obs(timestep=3)
        self.server._enqueue_observation(newest)
        self.assertIs(self.server.observation_queue.get_nowait(), newest)
        self.assertTrue(newest.must_go)

    def send(self, observation, offset):
        context = Mock()
        context.invocation_metadata.return_value = [('lerobot-clock-offset', str(offset))]
        packets = send_bytes_in_chunks(pickle.dumps(observation), services_pb2.Observation, silent=True)
        with patch('lerobot.async_inference.policy_server.time.time', return_value=100.6):
            self.server.SendObservations(packets, context)

    def test_corrected_clock_accepted_but_actual_stale_still_dropped(self):
        self.send(self.obs(), .5)
        self.assertEqual(self.server.observation_queue.qsize(), 1)
        self.server.observation_queue.get_nowait()
        self.send(self.obs(timestamp=99., must_go=True), .5)
        self.assertTrue(self.server.observation_queue.empty())
        self.send(self.obs(timestamp=100.5, must_go=True), 0.)
        self.assertTrue(self.server.observation_queue.get_nowait().must_go)

    def test_ready_publishes_clock(self):
        context = Mock()
        self.server.Ready(services_pb2.Empty(), context)
        self.assertIn('lerobot-server-time', dict(context.set_trailing_metadata.call_args.args[0]))


if __name__ == '__main__':
    unittest.main()
