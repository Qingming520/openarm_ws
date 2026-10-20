"""Bimanual v2 transport for the project's compensated ROS JointMITController.

Uses degrees at the LeRobot boundary; the local bridge handles ROS radians.
Automatically starts the project ROS controllers and bridge when needed.
"""
import json
import logging
import socket
import time

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_bi_openarm_follower import BiOpenArmFollowerConfig
from .ros_mit_runtime import ROSMITRuntime

logger = logging.getLogger(__name__)


class BiOpenArmROSMITFollower(Robot):
    config_class = BiOpenArmFollowerConfig
    name = 'bi_openarm_follower'

    def __init__(self, config):
        super().__init__(config)
        self.config = config
        for side in ('left', 'right'):
            arm = getattr(config, f'{side}_arm_config')
            if arm.hardware_version != 'v2' or arm.side != side or not arm.use_motor_zero:
                raise ValueError('Compensated ROS MIT requires both v2 sides and use_motor_zero=true')
            if arm.use_velocity_and_torque:
                raise ValueError('ROS MIT transport currently exposes position-only policy features')
            if arm.max_relative_target is not None or arm.joint_limits != arm.__class__(
                    port=arm.port, side=side, hardware_version='v2').joint_limits:
                raise ValueError('ROS MIT uses project v2 limits; custom per-arm limits are unsupported')
            expected = 'can1' if side == 'left' else 'can0'
            if arm.port != expected:
                raise ValueError(f'ROS MIT requires {side} port={expected}; configure bringup to match')
        camera_configs = dict(config.cameras)
        for side in ('left', 'right'):
            arm = getattr(config, f'{side}_arm_config')
            for name, camera in arm.cameras.items():
                key = f'{side}_{name}'
                if key in camera_configs:
                    raise ValueError(f'Camera name collision: {key}')
                camera_configs[key] = camera
        self.cameras = make_cameras_from_configs(camera_configs)
        self._action_features = {f'{s}_{n}.pos': float for s in ('left', 'right')
                                 for n in [*[f'joint_{i}' for i in range(1, 8)], 'gripper']}
        self._observation_features = {**self._action_features, **{
            k: (c.height, c.width, 3) for k, c in camera_configs.items()}}
        self.connection = self.stream = None
        self.runtime = ROSMITRuntime(config.ros_mit_bridge_port, config.ros_mit_fake_hardware)

    @property
    def action_features(self):
        return self._action_features

    @property
    def observation_features(self):
        return self._observation_features

    @property
    def is_connected(self):
        return self.stream is not None and all(c.is_connected for c in self.cameras.values())

    @property
    def is_calibrated(self):
        return True

    def calibrate(self):
        pass  # The project hardware uses existing motor zeros.

    def configure(self):
        pass  # Gains and dynamics belong to ROS JointMITController.

    def _rpc(self, operation, **kwargs):
        self.stream.write((json.dumps({'op': operation, **kwargs}, allow_nan=False) + '\n').encode())
        self.stream.flush()
        line = self.stream.readline()
        if not line:
            raise RuntimeError('ROS MIT bridge disconnected')
        response = json.loads(line)
        if 'error' in response:
            raise RuntimeError(response['error'])
        return response['result']

    @check_if_already_connected
    def connect(self, calibrate=True):
        try:
            self.runtime.start()
            deadline = time.monotonic() + 60.
            while True:
                self.runtime.check()
                try:
                    if self.stream is None:
                        self.connection = socket.create_connection(
                            ('127.0.0.1', self.config.ros_mit_bridge_port), timeout=10.)
                        self.stream = self.connection.makefile('rwb')
                    self._rpc('state')
                    break
                except (ConnectionRefusedError, RuntimeError) as exc:
                    if time.monotonic() >= deadline:
                        raise RuntimeError(f'MIT startup timed out: {exc}; '
                                           f'log: {self.runtime.log_path}') from exc
                    time.sleep(.2)
            for camera in self.cameras.values():
                camera.connect()
            logger.info('OpenArm v2 uses ROS JointMITController with gravity+Coriolis compensation; '
                        'gains are configured in ROS, LeRobot position_kp/kd are not applied')
        except BaseException:
            self.disconnect()
            raise

    @check_if_not_connected
    def get_observation(self):
        return {**self._rpc('state'), **{k: c.async_read() for k, c in self.cameras.items()}}

    @check_if_not_connected
    def send_action(self, action):
        return self._rpc('action', action=action)

    def disconnect(self):
        for camera in self.cameras.values():
            if camera.is_connected:
                camera.disconnect()
        if self.stream:
            self.stream.close()
        if self.connection:
            self.connection.close()
        self.stream = self.connection = None
        if hasattr(self, 'runtime'):
            self.runtime.close()
