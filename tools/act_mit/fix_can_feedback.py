"""Preserve motor feedback received during another motor's response wait."""
from pathlib import Path
import shutil

root = Path('/home/mingyang/lerobot/src/lerobot')
path = root / 'motors/damiao/damiao.py'
text = path.read_text()
marker = '    def _cache_unsolicited_feedback('
if marker not in text:
    anchor = '    def _recv_motor_response(\n'
    assert anchor in text
    text = text.replace(anchor, '''    def _cache_unsolicited_feedback(self, msg: can.Message) -> None:
        """A reply for another known axis is feedback, not a packet to discard."""
        motor = self._recv_id_to_motor.get(msg.arbitration_id)
        if motor is not None and not msg.is_error_frame and not msg.is_remote_frame and len(msg.data) == 8:
            self._process_response(motor, msg)

''' + anchor, 1)
    old = '''                    logger.debug(
                        f"Ignoring message from 0x{msg.arbitration_id:02X}, expected 0x{expected_recv_id:02X}"
                    )'''
    assert old in text
    text = text.replace(old, '                    self._cache_unsolicited_feedback(msg)', 1)
    old = '''                if msg and msg.arbitration_id in expected_set:
                    responses[msg.arbitration_id] = msg
                    if len(responses) == len(expected_recv_ids):
                        break'''
    new = '''                if msg:
                    if msg.arbitration_id in expected_set:
                        responses[msg.arbitration_id] = msg
                        if len(responses) == len(expected_recv_ids):
                            break
                    else:
                        self._cache_unsolicited_feedback(msg)'''
    assert old in text
    text = text.replace(old, new, 1)
    # Wall clock synchronization must not extend a CAN receive timeout.
    for name, following in [('_handshake', 'disconnect'),
                            ('_recv_motor_response', '_recv_all_responses'),
                            ('_recv_all_responses', '_encode_mit_packet')]:
        start = text.index(f'    def {name}(')
        end = text.index(f'    def {following}(', start)
        text = text[:start] + text[start:end].replace('time.time()', 'time.monotonic()') + text[end:]
    # Handshake must decode the received state, not the transmitted enable packet.
    start = text.index('    def _handshake(')
    end = text.index('    def disconnect(', start)
    handshake = text[start:end].replace('self._process_response(motor_name, msg)',
                                       'self._process_response(motor_name, response)')
    text = text[:start] + handshake + text[end:]
compile(text, str(path), 'exec')
native = root / 'robots/openarm_follower/native_joint_mit.py'
for p in (path, native):
    backup = p.with_name(p.name + '.before_feedback_fix')
    if not backup.exists():
        shutil.copy2(p, backup)
path.write_text(text)
shutil.copy2(Path(__file__).with_name('native_joint_mit.py'), native)
print('Fixed unsolicited CAN feedback, monotonic receive timeouts, handshake decoding and bounded refresh')
