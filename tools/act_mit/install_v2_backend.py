"""Install the v2 ROS controller routing into the local modified LeRobot tree."""
from pathlib import Path
import shutil

root = Path('/home/mingyang/lerobot/src/lerobot/robots')
source = Path(__file__).resolve().parent
factory = root / 'utils.py'
config = root / 'bi_openarm_follower/config_bi_openarm_follower.py'
old = '''    elif config.type == "bi_openarm_follower":
        from .bi_openarm_follower import BiOpenArmFollower

        return BiOpenArmFollower(config)'''
new = '''    elif config.type == "bi_openarm_follower":
        versions = (config.left_arm_config.hardware_version, config.right_arm_config.hardware_version)
        if "v2" in versions:
            if versions != ("v2", "v2"):
                raise ValueError("Both arms must use v2 for compensated ROS joint MIT")
            from .bi_openarm_follower.ros_mit_follower import BiOpenArmROSMITFollower

            return BiOpenArmROSMITFollower(config)
        from .bi_openarm_follower import BiOpenArmFollower

        return BiOpenArmFollower(config)'''
factory_text = factory.read_text()
config_text = config.read_text()
if old not in factory_text and new not in factory_text:
    raise RuntimeError('Factory changed; review before installing')
anchor = '    cameras: dict[str, CameraConfig] = field(default_factory=dict)'
if anchor not in config_text:
    raise RuntimeError('Config changed; review before installing')
for path in (factory, config):
    backup = path.with_name(path.name + '.before_compensated_v2')
    if not backup.exists():
        shutil.copy2(path, backup)
shutil.copy2(source / 'ros_mit_follower.py', root / 'bi_openarm_follower/ros_mit_follower.py')
shutil.copy2(source / 'ros_mit_runtime.py', root / 'bi_openarm_follower/ros_mit_runtime.py')
factory.write_text(factory_text.replace(old, new))
if 'ros_mit_bridge_port:' not in config_text:
    config.write_text(config_text.replace(anchor, anchor + '\n\n'
        '    # Both hardware_version=v2 arms use compensated ROS JointMITController.\n'
        '    ros_mit_bridge_port: int = 8765\n'))
updated_config = config.read_text()
if 'ros_mit_fake_hardware:' not in updated_config:
    config.write_text(updated_config.replace('    ros_mit_bridge_port: int = 8765',
        '    ros_mit_bridge_port: int = 8765\n'
        '    # Explicit simulation only; real hardware is the default.\n'
        '    ros_mit_fake_hardware: bool = False'))
print('Installed local bi_openarm_follower v2 -> compensated ROS JointMITController routing')
