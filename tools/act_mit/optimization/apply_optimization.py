"""Patch the respective local client or remote server without replacing other edits."""
import argparse
from pathlib import Path
import shutil

parser = argparse.ArgumentParser()
parser.add_argument('role', choices=('client', 'server'))
parser.add_argument('--root', type=Path, required=True)
args = parser.parse_args()
package = args.root / 'src/lerobot/async_inference'
updates = {}


def replace(text, old, new):
    if new in text:
        return text
    if old not in text:
        raise RuntimeError(f'Expected source changed: {old[:100]}')
    return text.replace(old, new, 1)


helpers = package / 'helpers.py'
updates[helpers] = replace(helpers.read_text(),
    '    init_logging(log_file=log_file, display_pid=False)',
    '    init_logging(log_file=log_file, display_pid=False,\n'
    '                 file_level=os.environ.get("LEROBOT_ASYNC_FILE_LOG_LEVEL", "INFO"))')

if args.role == 'client':
    path = package / 'robot_client.py'
    text = path.read_text()
    text = replace(text, 'from .configs import RobotClientConfig',
                   'from .configs import RobotClientConfig\nfrom .observation_timing import clock_offset')
    text = replace(text, '        self._send_intervals = []',
                   '        self._send_intervals = []\n        self._server_clock_offset = 0.0')
    text = replace(text, '''            start_time = time.perf_counter()
            self.stub.Ready(services_pb2.Empty())
            end_time = time.perf_counter()''', '''            start_time = time.perf_counter()
            started_wall = time.time()
            _, ready_call = self.stub.Ready.with_call(services_pb2.Empty(), timeout=10)
            finished_wall = time.time()
            self._server_clock_offset = clock_offset(
                ready_call.trailing_metadata(), started_wall, finished_wall)
            end_time = time.perf_counter()
            self.logger.info("Policy server clock offset: %.3f ms; handshake RTT: %.3f ms",
                             self._server_clock_offset * 1000,
                             (end_time - start_time) * 1000)''')
    text = replace(text, '            _ = self.stub.SendObservations(observation_iterator)',
                   '''            _ = self.stub.SendObservations(
                observation_iterator, timeout=5,
                metadata=(("lerobot-clock-offset", repr(self._server_clock_offset)),))''')
    text = replace(text, '''                observation.must_go = self.must_go.is_set() and self.action_queue.empty()
                current_queue_size = self.action_queue.qsize()

            if observation.must_go:
                self.must_go.clear()''', '''                # An RPC acknowledgement does not mean inference produced actions.
                # Keep forcing fresh observations until usable actions refill the queue.
                observation.must_go = self.action_queue.empty()
                current_queue_size = self.action_queue.qsize()''')
    text = replace(text, '''                    f"max_send_interval_ms={max(self._send_intervals, default=0.0):.2f}"''', '''                    f"max_send_interval_ms={max(self._send_intervals, default=0.0):.2f}, "
                    f"send_interval_stats={self._send_interval_stats()}"''')
    text = replace(text, '    def _ready_to_send_observation(self):', '''    def _send_interval_stats(self):
        values = sorted(self._send_intervals)
        if not values:
            return "n=0"
        mean = sum(values) / len(values)
        p95 = values[min(len(values) - 1, int((len(values) - 1) * .95))]
        p99 = values[min(len(values) - 1, int((len(values) - 1) * .99))]
        return f"n={len(values)} mean={mean:.2f}ms p95={p95:.2f}ms p99={p99:.2f}ms"

    def _ready_to_send_observation(self):''')
    text = replace(text, '''        except grpc.RpcError as e:
            self.logger.error(f"Error sending observation #{obs.get_timestep()}: {e}")''', '''        except grpc.RpcError as e:
            if not self.running:
                return False
            self.logger.error(f"Error sending observation #{obs.get_timestep()}: {e}")''')
    text = replace(text, '''            except grpc.RpcError as e:
                self.logger.error(f"Error receiving actions: {e}")''', '''            except grpc.RpcError as e:
                if not self.running:
                    break
                self.logger.error(f"Error receiving actions: {e}")
                self.shutdown_event.wait(0.1)''')
    # Always disconnect after a failed handshake or an interrupted model load.
    old = '''    if client.start():
        client.logger.info("Starting action receiver thread...")

        # Create and start action receiver thread
        action_receiver_thread = threading.Thread(target=client.receive_actions, daemon=True)

        # Start action receiver thread
        action_receiver_thread.start()

        try:
            # The main thread runs the control loop
            client.control_loop(task=cfg.task)

        finally:
            client.stop()
            action_receiver_thread.join()
            if cfg.debug_visualize_queue_size:
                visualize_action_queue_size(client.action_queue_size)
            client.logger.info("Client stopped")'''
    new = '''    action_receiver_thread = None
    try:
        if not client.start():
            return
        client.logger.info("Starting action receiver thread...")
        action_receiver_thread = threading.Thread(target=client.receive_actions, daemon=True)
        action_receiver_thread.start()
        client.control_loop(task=cfg.task)
    finally:
        client.stop()
        if action_receiver_thread is not None:
            action_receiver_thread.join(timeout=5)
        if cfg.debug_visualize_queue_size:
            visualize_action_queue_size(client.action_queue_size)
        client.logger.info("Client stopped")'''
    text = replace(text, old, new)
    updates[path] = text
else:
    config_path = package / 'configs.py'
    config_text = config_path.read_text()
    config_text = replace(config_text, '''    max_observation_age_s: float = field(''',
        '''    filter_similar_observations: bool = field(
        default=False,
        metadata={"help": "Skip observations with similar joint state (ignores camera changes)"},
    )
    max_observation_age_s: float = field(''')
    updates[config_path] = config_text
    path = package / 'policy_server.py'
    text = path.read_text()
    text = replace(text, 'from .configs import PolicyServerConfig',
                   'from .configs import PolicyServerConfig\nfrom .observation_timing import observation_age')
    text = replace(text, '''        self.shutdown_event.clear()

        return services_pb2.Empty()''', '''        self.shutdown_event.clear()
        context.set_trailing_metadata((("lerobot-server-time", repr(time.time())),))

        return services_pb2.Empty()''')
    text = replace(text, '        observation_age_s = receive_time - obs_timestamp',
                   '''        observation_age_s = observation_age(
            obs_timestamp, receive_time, context.invocation_metadata())''')
    text = replace(text,
        '        elif observations_similar(obs, previous_obs, lerobot_features=self.lerobot_features):',
        '''        elif self.config.filter_similar_observations and observations_similar(
            obs, previous_obs, lerobot_features=self.lerobot_features
        ):''')
    text = replace(text, '        self.observation_queue = Queue(maxsize=1)\n\n        self._predicted_timesteps_lock',
                   '        self.observation_queue = Queue(maxsize=1)\n'
                   '        self._observation_enqueue_lock = threading.Lock()\n\n'
                   '        self._predicted_timesteps_lock')
    start = text.index('    def _enqueue_observation(self, obs: TimedObservation) -> bool:')
    end = text.index('    def _time_action_chunk(', start)
    text = text[:start] + '''    def _enqueue_observation(self, obs: TimedObservation) -> bool:
        """Keep only the latest observation, preserving an outstanding recovery request."""
        with self._observation_enqueue_lock:
            if not (obs.must_go or self.last_processed_obs is None
                    or self._obs_sanity_checks(obs, self.last_processed_obs)):
                return False
            try:
                previous = self.observation_queue.get_nowait()
                obs.must_go = obs.must_go or previous.must_go
            except Empty:
                pass
            self.observation_queue.put_nowait(obs)
            return True

''' + text[end:]
    updates[path] = text

for path, text in updates.items():
    compile(text, str(path), 'exec')
for path, text in updates.items():
    backup = path.with_name(path.name + '.before_stream_recovery')
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_text(text)
shutil.copy2(Path(__file__).with_name('observation_timing.py'), package / 'observation_timing.py')
print(f'Applied {args.role} recovery/timing/logging optimization')
