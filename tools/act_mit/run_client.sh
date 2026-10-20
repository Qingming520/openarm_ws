#!/usr/bin/env bash
set -euo pipefail
task_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
task_python="${LEROBOT_PYTHON:-/home/mingyang/miniconda3/envs/lerobot/bin/python}"
task_model="${1:-/home/robolab/lerobot/outputs/act/openarm/clean_the_table_20261015_115715/checkpoints/080000/pretrained_model}"
task_prompt="${2:-Clean the table.}"
task_cameras="$("$task_python" - "$task_workspace" <<'PY'
import importlib.util, json, sys
from pathlib import Path
path=Path(sys.argv[1])/'src/dora-openarm-data-collection/scripts/wrist_camera_capture.py'
spec=importlib.util.spec_from_file_location('wrist_discovery', path)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
wrists=module.discover()
cameras={}
for name,serial in [('top','CP2N1630002X'),('front','CV2L360000BG')]:
    cameras[name]=dict(type='orbbec_color',serial_number_or_name=serial,
                       width=640,height=480,fps=30,color_mode='rgb',rotation=0,warmup_s=1)
for side,path in wrists.items():
    cameras[side+'_wrist']=dict(type='opencv',index_or_path=str(path),backend='V4L2',
        fourcc='YUYV',width=960,height=600,fps=30,color_mode='rgb',rotation=0,warmup_s=1)
print(json.dumps(cameras))
PY
)"
exec "$task_python" "$task_workspace/tools/act_mit/robot_client.py" \
  --server_address="${ACT_SERVER:-192.168.3.249:8080}" \
  --policy_type=act \
  --pretrained_name_or_path="$task_model" \
  --policy_device=cuda \
  --client_device=cpu \
  --robot.type=bi_openarm_ros_mit \
  --robot.id=openarm_v2_ros_mit \
  --robot.cameras="$task_cameras" \
  --fps=30 \
  --actions_per_chunk=50 \
  --control_fps=150 \
  --observation_fps=10 \
  --max_action_velocity=30 \
  --chunk_size_threshold=0.7 \
  --aggregate_fn_name=weighted_average \
  --angle_conversion=none \
  --task="$task_prompt"
