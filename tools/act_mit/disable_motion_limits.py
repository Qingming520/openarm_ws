"""Install the requested native target behavior and explicit velocity-disable CLI."""
from pathlib import Path
import shutil

root = Path('/home/mingyang/lerobot/src/lerobot')
source = Path(__file__).resolve().parent
target = root / 'robots/openarm_follower/native_joint_mit.py'
config = root / 'async_inference/configs.py'
text = config.read_text()
old = '''        if self.max_action_velocity is not None:
            limits = self.max_action_velocity'''
new = '''        # A scalar zero explicitly disables action slew limiting.
        if self.max_action_velocity == 0:
            self.max_action_velocity = None
        if self.max_action_velocity is not None:
            limits = self.max_action_velocity'''
if new not in text:
    if old not in text:
        raise RuntimeError('Velocity configuration changed; review before applying')
    text = text.replace(old, new, 1)
compile(text, str(config), 'exec')
for path in (target, config):
    backup = path.with_name(path.name + '.before_remove_motion_limits')
    if not backup.exists():
        shutil.copy2(path, backup)
shutil.copy2(source / 'native_joint_mit.py', target)
config.write_text(text)
print('Removed native whole-arm rollback. --max_action_velocity=0 disables velocity limiting.')
