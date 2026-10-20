"""Direct CAN joint MIT using the project's existing bias torque compensator."""
import importlib.util
import os
from pathlib import Path
import time

import numpy as np
import yaml


class NativeJointMITController:
    def __init__(self, config):
        if not config.use_motor_zero:
            raise ValueError('Compensated v2 MIT requires use_motor_zero=true')
        workspace = Path(os.environ.get('OPENARM_WS', str(Path.home() / 'openarm_ws')))
        collection = workspace / 'src/dora-openarm-data-collection'
        source = collection / 'nodes/dora-openarm/src/dora_openarm/dynamics.py'
        model = collection / 'lib/openarm_mujoco/v2/pedestal/pedestal.xml'
        params_file = workspace / 'src/openarm_ros2/openarm_cartesian_impedance/config/controllers.yaml'
        spec = importlib.util.spec_from_file_location('openarm_project_bias', source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.compensator = module.BiasTorqueCompensator(model, config.side,
                                                       gravity=True, coriolis=True)
        params = yaml.safe_load(params_file.read_text())[f'{config.side}_joint_mit']['ros__parameters']
        self.kp, self.kd = params['kp'], params['kd']
        self.side, self.port = config.side, config.port
        self.names = [*[f'joint_{i}' for i in range(1, 8)], 'gripper']

    def prepare(self, goals, bus):
        """Use measured motor q/dq (degrees), never predicted targets, for bias."""
        if not all(np.isfinite(v) for v in goals.values()):
            raise ValueError('Non-finite MIT target')
        def stale_names():
            now = time.monotonic()
            return [n for n in self.names
                    if now - bus._state_received_at.get(n, -float('inf')) > .15]

        # Only refresh missing/old axes. Two bounded query rounds handle a lost
        # refresh reply without extending the 150 ms state freshness threshold.
        stale = stale_names()
        for _ in range(2):
            if not stale:
                break
            bus.sync_read_all_states(motors=stale)
            stale = stale_names()
        if stale:
            now = time.monotonic()
            ages = ', '.join(f'{n}={1000 * (now - bus._state_received_at.get(n, -float("inf"))):.1f}ms'
                             for n in stale)
            raise RuntimeError(f'Compensated MIT requires fresh feedback (<150 ms); '
                               f'side={self.side}, port={self.port}, stale: {ages}; '
                               'two refresh attempts failed')
        states = bus._last_known_states
        q = np.array([states[n]['position'] for n in self.names], dtype=float)
        dq = np.array([states[n]['velocity'] for n in self.names], dtype=float)
        torque = self.compensator.compute({'qpos': np.radians(q), 'qvel': np.radians(dq)})
        return dict(goals), {n: float(torque[i]) for i, n in enumerate(self.names[:7])}
