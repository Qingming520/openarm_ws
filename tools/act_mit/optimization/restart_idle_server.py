"""Apply the verified service update only when port 8080 has no active client."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

connections = subprocess.check_output(
    ['ss', '-Htn', 'state', 'established', '( sport = :8080 )'], text=True).strip()
if connections:
    raise SystemExit('Active clients on 8080; leave the running service untouched')
processes = []
tests = []
for directory in Path('/proc').iterdir():
    if not directory.name.isdigit():
        continue
    try:
        args = (directory / 'cmdline').read_bytes().split(b'\0')
    except (OSError, PermissionError):
        continue
    if b'lerobot.async_inference.policy_server' not in args:
        continue
    if b'--port=18080' in args:
        tests.append(int(directory.name))
    else:
        processes.append(int(directory.name))
if len(processes) > 1:
    raise SystemExit(f'Multiple production servers found: {processes}; review required')
for pid in processes + tests:
    os.kill(pid, signal.SIGINT)
    deadline = time.monotonic() + 10
    while Path(f'/proc/{pid}').exists():
        try:
            if 'State:\tZ' in Path(f'/proc/{pid}/status').read_text():
                break
        except FileNotFoundError:
            break
        if time.monotonic() > deadline:
            raise SystemExit(f'Process {pid} did not stop; new server not started')
        time.sleep(.1)
root = Path('/home/robolab/lerobot')
console = root / 'logs/policy_server_optimized_console.log'
with console.open('a') as output:
    process = subprocess.Popen([
        '/home/robolab/miniconda3/envs/lerobot/bin/python',
        '-m', 'lerobot.async_inference.policy_server',
        '--host=0.0.0.0', '--port=8080', '--filter_similar_observations=false',
    ], cwd=root, stdout=output, stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL, start_new_session=True)
deadline = time.monotonic() + 20
while time.monotonic() < deadline:
    if process.poll() is not None:
        raise SystemExit(f'Server startup failed; see {console}')
    try:
        with socket.create_connection(('127.0.0.1', 8080), timeout=.2):
            print(f'Updated production server listening on 8080; PID={process.pid}; log={console}')
            break
    except OSError:
        time.sleep(.2)
else:
    raise SystemExit(f'Server not ready; see {console}')
