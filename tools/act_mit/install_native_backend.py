"""Replace the earlier ROS routing with direct compensated CAN control."""
from pathlib import Path
import shutil

root = Path('/home/mingyang/lerobot/src/lerobot')
source = Path(__file__).resolve().parent
updates = {}
factory = root / 'robots/utils.py'
text = factory.read_text()
start = text.index('    elif config.type == "bi_openarm_follower":')
end = text.index('    elif ', start + 10)
updates[factory] = text[:start] + '''    elif config.type == "bi_openarm_follower":
        from .bi_openarm_follower import BiOpenArmFollower

        return BiOpenArmFollower(config)
''' + text[end:]

config = root / 'robots/bi_openarm_follower/config_bi_openarm_follower.py'
text = config.read_text()
text = text.replace('\n    # Both hardware_version=v2 arms use compensated ROS JointMITController.\n'
                    '    ros_mit_bridge_port: int = 8765\n', '')
text = text.replace('    # Explicit simulation only; real hardware is the default.\n'
                    '    ros_mit_fake_hardware: bool = False\n', '')
updates[config] = text

arm = root / 'robots/openarm_follower/openarm_follower.py'
text = arm.read_text()
if 'NativeJointMITController' not in text:
    anchor = '        # Arm motors\n'
    assert anchor in text
    text = text.replace(anchor, '''        self.joint_mit_controller = None
        if config.hardware_version == "v2":
            from .native_joint_mit import NativeJointMITController

            self.joint_mit_controller = NativeJointMITController(config)
            logger.info("OpenArm v2 direct CAN MIT: project gravity+Coriolis compensation enabled")

''' + anchor, 1)
    anchor = '        # Use batch MIT control for arm (sends all commands, then collects responses)\n'
    assert anchor in text
    text = text.replace(anchor, '''        feedforward = {}
        if self.joint_mit_controller is not None:
            goal_pos, feedforward = self.joint_mit_controller.prepare(goal_pos, self.bus)

''' + anchor, 1)
    anchor = '            commands[motor_name] = (kp, kd, position_degrees, 0.0, 0.0)'
    assert anchor in text
    text = text.replace(anchor, '''            if self.joint_mit_controller is not None and motor_name != "gripper":
                # Use the project's compensated joint MIT gains by default.
                if custom_kp is None or motor_name not in custom_kp:
                    kp = self.joint_mit_controller.kp[idx]
                if custom_kd is None or motor_name not in custom_kd:
                    kd = self.joint_mit_controller.kd[idx]
            commands[motor_name] = (kp, kd, position_degrees, 0.0, feedforward.get(motor_name, 0.0))''')
updates[arm] = text

bus = root / 'motors/damiao/damiao.py'
text = bus.read_text()
if '_state_received_at' not in text:
    anchor = '        # Dynamic gains storage\n'
    assert anchor in text
    text = text.replace(anchor, '        self._state_received_at: dict[str, float] = {}\n\n' + anchor, 1)
    anchor = '        except Exception as e:\n            logger.warning(f"Failed to decode response from {motor}: {e}")'
    assert anchor in text
    text = text.replace(anchor, '            self._state_received_at[motor] = time.monotonic()\n' + anchor, 1)
updates[bus] = text

# Validate every edit before writing any file.
for path, text in updates.items():
    compile(text, str(path), 'exec')
for path, text in updates.items():
    backup = path.with_name(path.name + '.before_native_mit')
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_text(text)
shutil.copy2(source / 'native_joint_mit.py', root / 'robots/openarm_follower/native_joint_mit.py')
print('Installed native v2 project compensation -> MIT -> CAN; no ROS/bridge runtime')
