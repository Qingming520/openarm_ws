#!/usr/bin/python3
"""Local JSON bridge from LeRobot (degrees) to compensated ROS MIT (radians)."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socketserver
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters

SIDES = ('left', 'right')
KEYS = [f'{s}_{n}.pos' for s in SIDES
        for n in [*[f'joint_{i}' for i in range(1, 8)], 'gripper']]
JOINTS = [f'openarm_{s}_{n}' for s in SIDES
          for n in [*[f'joint{i}' for i in range(1, 8)], 'finger_joint1']]


def bounded_target(action, measured, previous, dt, speed):
    """All values here use degrees; constrain both slew and tracking error."""
    if set(action) != set(KEYS):
        raise ValueError('Expected exactly 16 named joint/gripper positions')
    output = {}
    for index, key in enumerate(KEYS):
        value = float(action[key])
        if not math.isfinite(value):
            raise ValueError('Non-finite action')
        joint_index = index % 8
        left = index < 8
        lower = [-3.4907 if left else -1.3963,
                 -3.3161 if left else -0.17453, -1.5708, 0.,
                 -1.5708, -0.7854, -1.5708, -0.4 if left else -1.0472]
        upper = [1.3963 if left else 3.4907,
                 0.17453 if left else 3.3161, 1.5708, 2.4435,
                 1.5708, 0.7854, 1.5708, 1.0472 if left else 0.4]
        value = max(math.degrees(lower[joint_index]),
                    min(math.degrees(upper[joint_index]), value))
        step = speed * min(dt, 0.02)
        value = max(previous[key] - step, min(previous[key] + step, value))
        # Stay inside JointMITController's 0.1 rad measured-position guard.
        guard = math.degrees(0.08)
        output[key] = max(measured[key] - guard, min(measured[key] + guard, value))
    return output


class Bridge(Node):
    def __init__(self, speed):
        super().__init__('act_joint_mit_bridge')
        self.lock = threading.Lock()
        self.state = {}
        self.receipt = 0.
        self.previous = None
        self.last_send = 0.
        self.speed = speed
        self.create_subscription(JointState, '/joint_states', self.feedback,
                                 qos_profile_sensor_data)
        self.arms = {s: self.create_publisher(JointTrajectory, f'/{s}_joint_mit/target', 1)
                     for s in SIDES}
        self.grippers = {s: self.create_publisher(
            JointTrajectory, f'/{s}_gripper_controller/joint_trajectory', 1) for s in SIDES}
        self.list_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.param_clients = {s: self.create_client(GetParameters, f'/{s}_joint_mit/get_parameters')
                              for s in SIDES}
        self.verified_at = 0.

    def feedback(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not all(j in values and math.isfinite(values[j]) for j in JOINTS):
            return
        with self.lock:
            self.state = {k: math.degrees(values[j]) for k, j in zip(KEYS, JOINTS)}
            self.receipt = time.monotonic()

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=2.):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(2.):
            raise RuntimeError('Controller service timed out')
        return future.result()

    def verify(self):
        if time.monotonic() - self.verified_at < 1.:
            return
        response = self.call(self.list_client, ListControllers.Request())
        active = {c.name for c in response.controller if c.state == 'active'}
        required = {f'{s}_{kind}' for s in SIDES
                    for kind in ('joint_mit', 'gripper_controller')}
        if not required <= active:
            raise RuntimeError(f'Required active controllers: {sorted(required)}; got {sorted(active)}')
        for client in self.param_clients.values():
            request = GetParameters.Request()
            request.names = ['gravity_compensation', 'coriolis_compensation']
            result = self.call(client, request)
            if len(result.values) != 2 or not all(v.type == 1 and v.bool_value for v in result.values):
                raise RuntimeError('Gravity and Coriolis compensation must both be enabled')
        self.verified_at = time.monotonic()

    def request(self, request):
        self.verify()
        with self.lock:
            measured = dict(self.state)
            age = time.monotonic() - self.receipt
        if age > 0.15 or len(measured) != 16:
            raise RuntimeError('Missing/stale joint feedback (>150 ms)')
        if request['op'] == 'state':
            return measured
        if request['op'] != 'action':
            raise ValueError('Unknown operation')
        now = time.monotonic()
        if self.previous is None or now - self.last_send > 0.2:
            self.previous = measured
            self.last_send = now - 1. / 150
        output = bounded_target(request['action'], measured, self.previous,
                                now - self.last_send, self.speed)
        for offset, side in enumerate(SIDES):
            msg = JointTrajectory()
            msg.joint_names = JOINTS[offset * 8:offset * 8 + 7]
            point = JointTrajectoryPoint()
            point.positions = [math.radians(output[k]) for k in KEYS[offset * 8:offset * 8 + 7]]
            msg.points = [point]
            self.arms[side].publish(msg)
            grip = JointTrajectory()
            grip.joint_names = [JOINTS[offset * 8 + 7]]
            gp = JointTrajectoryPoint()
            gp.positions = [math.radians(output[KEYS[offset * 8 + 7]])]
            gp.time_from_start.nanosec = 20_000_000
            grip.points = [gp]
            self.grippers[side].publish(grip)
        self.previous, self.last_send = output, now
        return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--max-velocity-deg', type=float, default=30.)
    parser.add_argument('--auto-start-controller', action='store_true')
    parser.add_argument('--fake-hardware', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.max_velocity_deg) or not 0 < args.max_velocity_deg <= 60:
        parser.error('--max-velocity-deg must be in (0,60]')
    rclpy.init()
    bridge = Bridge(args.max_velocity_deg)
    thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    thread.start()
    controller_process = None
    if args.auto_start_controller and not bridge.list_client.wait_for_service(timeout_sec=2.):
        controller_process = subprocess.Popen([
            '/opt/ros/humble/bin/ros2', 'launch', 'openarm_bringup',
            'openarm.bimanual.launch.py', 'arm_type:=v20',
            f'use_fake_hardware:={str(args.fake_hardware).lower()}',
            'robot_controller:=joint_mit_controller',
            'right_can_interface:=can0', 'left_can_interface:=can1', 'use_rviz:=false',
        ], start_new_session=True)
        print('Starting project compensated JointMITController automatically', flush=True)

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            while True:
                line = self.rfile.readline(65536)
                if not line:
                    return
                try:
                    response = {'result': bridge.request(json.loads(line))}
                except Exception as exc:
                    response = {'error': str(exc)}
                self.wfile.write((json.dumps(response, allow_nan=False) + '\n').encode())
                self.wfile.flush()

    class Server(socketserver.TCPServer):
        allow_reuse_address = True

    try:
        with Server(('127.0.0.1', args.port), Handler) as server:
            print(f'ACT MIT bridge listening on localhost:{args.port}; units=degrees', flush=True)
            server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if controller_process is not None:
            try:
                os.killpg(controller_process.pid, signal.SIGINT)
                controller_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(controller_process.pid, signal.SIGTERM)
                controller_process.wait(timeout=5)
            except ProcessLookupError:
                pass
        rclpy.try_shutdown()
        thread.join(timeout=2)
        bridge.destroy_node()


if __name__ == '__main__':
    main()
