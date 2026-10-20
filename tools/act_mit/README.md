# Native ACT → compensated MIT → CAN

Current local ~/lerobot uses the original bi_openarm_follower entry directly.
Setting hardware_version=v2 and use_motor_zero=true selects native compensated
MIT. No ROS process, bridge, localhost connection or extra terminal is required.
The earlier ROS transport scripts here are obsolete for this entry.

Keep the original robot_client command, server address 192.168.8.20:8080,
camera definitions and angle_conversion=none. Remove the duplicate aggregate
argument if desired; its last value, weighted_average, already wins. CAN must
be configured and the remote policy server must be running as before.
Stop other CAN owners before starting inference.

The native adapter reuses the existing project BiasTorqueCompensator from
src/dora-openarm-data-collection/nodes/dora-openarm/src/dora_openarm/dynamics.py
and its v2/pedestal/pedestal.xml model. MuJoCo calculates dynamics only; no
simulation or viewer runs. It reads project joint MIT gains and step limits
from openarm_cartesian_impedance/config/controllers.yaml. It does not
instantiate the ROS C++ plugin.

Each outgoing arm MIT command is (kp, kd, target_degrees, 0, compensation_Nm).
Compensation is gravity plus Coriolis/centrifugal torque computed from actual
motor positions and velocities, including fingers. Feedback is converted
from degrees to model radians; ACT stays in degrees. Feedforward is bounded
to [8,8,8,8,3,3,3] Nm. The motor's own PD torque is outside this cap. Friction,
extra payload and acceleration are not compensated. The native 0.1 rad
whole-arm rollback has been removed: large targets are no longer replaced
with measured positions. Use `--max_action_velocity=0` (or omit the option)
to disable inference velocity limiting. Replace any existing
`--max_action_velocity=60` argument with zero. 150 Hz interpolation remains
active when `control_fps=150`. Feedback older than 150 ms is refreshed, then rejected if still stale.
The gripper retains native v2 POS_FORCE control. Legacy behavior is preserved.

install_native_backend.py installs the native source and removes the earlier
v2 ROS factory routing. Original files have .before_native_mit backups.
No remote source was changed.

Verification: mock-CAN integration tests exercise actual send_action,
matching emitted torque to project dynamics, matching joint gains, gripper
routing, stale/nonfinite rejection, and legacy zero feedforward. Original CLI
parsing also passes. No real motor motion or physical camera run was performed.
Tests also check that large targets reach the outgoing MIT position field
without resetting other joints, and that scalar zero disables action slew limiting.

Run tests with:

```bash
/home/mingyang/miniconda3/envs/lerobot/bin/python tools/act_mit/test_native_backend.py
```

## Inference recovery optimization

Local client and remote GPU server now keep forcing fresh observations while
the action queue is empty. RPC success alone does not clear recovery. The
server defaults `filter_similar_observations=false`; duplicate timesteps and
actual stale observations are still rejected. Pending observation replacement
preserves a forced recovery request.

A backward-compatible gRPC handshake metadata field measures server/client
clock offset. Observation metadata carries the correction without altering
action timestamps or model pacing. The measured offset on the wired link was
about +451 ms with 2.2 ms handshake RTT. This prevents wall-clock differences
from consuming the server's 0.5 s freshness budget. System clocks are not changed.

Async file logs now default to INFO instead of DEBUG. Set
`LEROBOT_ASYNC_FILE_LOG_LEVEL=DEBUG` before starting a process when detailed
diagnostics are needed. Five-second control metrics include sample count,
mean, P95 and P99 send intervals in addition to the maximum.

Verification: 35 existing client tests, 3 new client recovery/timing tests,
and 5 remote server tests passed. A temporary GPU service using the actual
060000 ACT checkpoint accepted identical fresh visual observations and
recovered after deliberate stale forced-observation rejection, using a mocked
robot and synthetic images. No motor commands were sent. The idle production
8080 service was restarted with similarity filtering explicitly disabled.
Patch sources, tests and guarded restart script are in `optimization/`;
modified source files have `.before_stream_recovery` backups.

## CAN feedback reception fix

Single-motor and batch waits now cache valid feedback from other known axes
instead of discarding it. This prevents a gripper response wait from losing
arm feedback and a batch arm wait from losing gripper feedback. Receive
timeouts use monotonic time. Handshake decoding uses the actual received
state packet, rather than the transmitted enable command.

Native compensation retains its 150 ms freshness check. If needed, it queries
only stale axes in up to two bounded rounds. Persistent failure reports side,
CAN interface, motor names and measured ages; it does not send stale-state
compensation or disable the freshness check. The earlier whole-arm rollback
remains removed and `max_action_velocity=0` still disables inference limiting.

`test_can_feedback.py` tests queued frames and bounded recovery without
physical CAN. The latest failure log did not identify which axis was stale,
so this fixes a verified receive-path defect, not proof that hardware packet
loss or scheduling stalls cannot still occur. No physical robot was operated.
