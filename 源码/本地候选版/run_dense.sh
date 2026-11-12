#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/humble/setup.bash
# Remove only the explicitly retired S-bend files after checking the new
# implementation. Frozen files in /root/racecar/tools are outside this list.
/usr/bin/python3 - <<'PY'
import ast
from pathlib import Path
folder = Path.cwd()
required = ('dense_s_bend.py', 'planners/dense/dense_reference.py', 'core/vehicle_io.py', 'core/geometry.py')
for name in required:
    ast.parse((folder/name).read_text(encoding='utf-8'))
if not (folder/'dense_settings.json').is_file():
    raise RuntimeError('Missing dense_settings.json; legacy files left untouched')
legacy = ('s_bend.py', 'corridor.py', 'run.sh', 'first_lap_scan.yaml',
          'left_entry_scan.yaml', 'straight_scan.yaml', 'straight_scan.yaml~', 'last_run.json')
removed = []
for name in legacy:
    path = folder/name
    if path.is_file():
        path.unlink()
        removed.append(name)
for pattern in ('s_bend.*.pyc', 'corridor.*.pyc'):
    for path in (folder/'__pycache__').glob(pattern):
        if path.is_file():
            path.unlink()
if removed:
    print('Removed retired S-bend files: '+', '.join(removed))
PY
exec /usr/bin/python3 ./dense_s_bend.py "$@"
