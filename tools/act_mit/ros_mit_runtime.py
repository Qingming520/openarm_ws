"""Owned subprocess lifecycle for one-command v2 inference."""
import fcntl
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile


class ROSMITRuntime:
    def __init__(self, port, fake_hardware=False):
        self.port = port
        self.fake_hardware = fake_hardware
        self.process = self.lock = self.log = None
        self.log_path = None

    def start(self):
        # One native client per bridge; a second client must not share control.
        lock_path = Path(tempfile.gettempdir()) / f'openarm-mit-client-{os.getuid()}-{self.port}.lock'
        self.lock = lock_path.open('a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.close()
            raise RuntimeError('Another ACT MIT client is already using this bridge') from None
        try:
            with socket.create_connection(('127.0.0.1', self.port), timeout=.3):
                return  # Reuse an externally managed bridge without owning its processes.
        except ConnectionRefusedError:
            pass
        workspace = Path(os.environ.get('OPENARM_WS', str(Path.home() / 'openarm_ws'))).resolve()
        bridge = workspace / 'tools/act_mit/ros_bridge.py'
        setup = workspace / 'install/setup.bash'
        if not bridge.is_file() or not setup.is_file():
            raise RuntimeError(f'Project ROS installation/bridge missing under {workspace}')
        env = os.environ.copy()
        # ROS Humble requires system Python, not Conda Python or its libraries.
        for name in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH',
                     'COLCON_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'CONDA_PREFIX'):
            env.pop(name, None)
        env['PATH'] = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
        self.log = tempfile.NamedTemporaryFile(prefix='openarm-act-mit-', suffix='.log', delete=False)
        self.log_path = self.log.name
        args = ['/bin/bash', '--noprofile', '--norc', '-c',
                'source /opt/ros/humble/setup.bash && source "$1" && '
                'exec /usr/bin/python3 "$2" --auto-start-controller --port "$3" '
                '--max-velocity-deg 60 "${@:4}"',
                'openarm-act-mit', str(setup), str(bridge), str(self.port)]
        if self.fake_hardware:
            args.append('--fake-hardware')
        self.process = subprocess.Popen(args, env=env, stdout=self.log,
                                        stderr=subprocess.STDOUT, start_new_session=True)
        print(f'Automatically starting compensated OpenArm v2 MIT; log: {self.log_path}', flush=True)

    def check(self):
        if self.process is not None and self.process.poll() is not None:
            raise RuntimeError(f'Automatic MIT startup exited ({self.process.returncode}); '
                               f'see {self.log_path}')

    def close(self):
        if self.process is not None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
                self.process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=5)
            except ProcessLookupError:
                pass
            self.process = None
        if self.log is not None:
            self.log.close()
            self.log = None
        if self.lock is not None:
            self.lock.close()
            self.lock = None
