"""Real GPU inference with synthetic observations and a mocked robot; no CAN."""
import pickle
import time
from unittest.mock import Mock, patch

import numpy as np
import torch

from lerobot.async_inference.configs import RobotClientConfig
from lerobot.async_inference.helpers import TimedObservation
from lerobot.async_inference.robot_client import RobotClient
from lerobot.robots.openarm_follower import OpenArmFollowerConfig
from lerobot.transport import services_pb2

names = [f'{s}_{n}.pos' for s in ('left', 'right')
         for n in [*[f'joint_{i}' for i in range(1, 8)], 'gripper']]
shapes = {'top': (480, 640, 3), 'front': (480, 640, 3),
          'left_wrist': (600, 960, 3), 'right_wrist': (600, 960, 3)}
raw = {**dict.fromkeys(names, 0.), **{k: np.zeros(v, dtype=np.uint8) for k, v in shapes.items()}}
robot = Mock()
robot.observation_features = {**dict.fromkeys(names, float), **shapes}
robot.action_features = dict.fromkeys(names, float)
robot.get_observation.side_effect = lambda: dict(raw)
config = RobotClientConfig(robot=OpenArmFollowerConfig(port='unused'), policy_type='act',
    pretrained_name_or_path='/home/robolab/lerobot/outputs/act/openarm/clean_the_table_20261015_115715/checkpoints/060000/pretrained_model',
    actions_per_chunk=50, policy_device='cuda', server_address='192.168.8.20:18080')
with patch('lerobot.async_inference.robot_client.make_robot_from_config', return_value=robot):
    client = RobotClient(config)


def receive(expect_actions):
    response = client.stub.GetActions(services_pb2.Empty(), timeout=5)
    assert bool(response.data) == expect_actions
    if response.data:
        actions = pickle.loads(response.data)
        assert len(actions) == 50
        assert all(torch.isfinite(a.action).all() for a in actions)


try:
    assert client.start()
    print(f'Handshake clock correction: {client._server_clock_offset * 1000:.2f} ms', flush=True)
    assert client.send_observation(TimedObservation(timestamp=time.time(), timestep=0,
                                                   observation=dict(raw), must_go=True))
    receive(True)
    assert client.send_observation(TimedObservation(timestamp=time.time(), timestep=50,
                                                   observation=dict(raw), must_go=False))
    receive(True)  # Identical joint state and images must not suppress a new timestep.
    print('PASS: real ACT infers repeated identical visual observations', flush=True)
    client.latest_action = 100
    client.must_go.clear()
    client.control_loop_observation('Clean the table.')
    dropped = client.observation_queue.get_nowait()
    assert dropped.must_go
    dropped.timestamp -= 2
    assert client.send_observation(dropped)
    receive(False)  # Actual stale forced observations are still rejected.
    client.control_loop_observation('Clean the table.')
    retry = client.observation_queue.get_nowait()
    assert retry.must_go
    assert client.send_observation(retry)
    receive(True)
    print('PASS: dropped forced observation -> fresh retry -> real ACT actions recover', flush=True)
    robot.send_action.assert_not_called()
finally:
    client.stop()
