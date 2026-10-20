#!/usr/bin/env python
"""Run the existing LeRobot asynchronous client against the local ROS bridge."""
import json
import socket
from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.config import RobotConfig
from lerobot.async_inference import robot_client as client


@RobotConfig.register_subclass('bi_openarm_ros_mit')
@dataclass
class ROSMITConfig(RobotConfig):
    bridge_port: int = 8765
    cameras: dict[str, CameraConfig] = field(default_factory=dict)


class ROSMITRobot:
    name = 'bi_openarm_ros_mit'

    def __init__(self, config):
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self.action_features = {f'{s}_{n}.pos': float for s in ('left', 'right')
                                for n in [*[f'joint_{i}' for i in range(1, 8)], 'gripper']}
        self.observation_features = {**self.action_features, **{
            k: (c.height, c.width, 3) for k, c in self.cameras.items()}}
        self.connection = self.stream = None

    def rpc(self, operation, **kwargs):
        self.stream.write((json.dumps({'op': operation, **kwargs}, allow_nan=False) + '\n').encode())
        self.stream.flush()
        line = self.stream.readline()
        if not line:
            raise RuntimeError('ROS bridge disconnected')
        response = json.loads(line)
        if 'error' in response:
            raise RuntimeError(response['error'])
        return response['result']

    def connect(self):
        try:
            self.connection = socket.create_connection(('127.0.0.1', self.config.bridge_port), timeout=10.)
            self.stream = self.connection.makefile('rwb')
            self.rpc('state')  # Require fresh feedback, active MIT, and both compensations.
            for camera in self.cameras.values():
                camera.connect()
        except Exception:
            self.disconnect()
            raise

    def get_observation(self):
        return {**self.rpc('state'), **{k: c.async_read() for k, c in self.cameras.items()}}

    def send_action(self, action):
        return self.rpc('action', action=action)

    def disconnect(self):
        for camera in self.cameras.values():
            if camera.is_connected:
                camera.disconnect()
        if self.stream:
            self.stream.close()
        if self.connection:
            self.connection.close()
        self.stream = self.connection = None


if __name__ == '__main__':
    original_factory = client.make_robot_from_config
    client.make_robot_from_config = lambda cfg: (
        ROSMITRobot(cfg) if isinstance(cfg, ROSMITConfig) else original_factory(cfg))
    client.async_client()
