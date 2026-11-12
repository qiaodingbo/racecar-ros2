# Directory layout

```
s_bend/
  run_dense.sh              entry point (starts dense_s_bend.py)
  dense_s_bend.py           ROS node / lifecycle / speed loop
  dense_settings.json       runtime parameters
  dense_last_run.json       latest record (overwritten each run)
  core/
    vehicle_io.py           ROS IO, safety, encoder/IMU/scan state
    geometry.py             scan clustering, clearance, path helpers
  planners/
    dense/dense_reference.py    legacy single-row planner (side vote + offset)
    dense/pairing.py            bilateral nearest-dense pairing planner (active)
    corridor/corridor_planner.py two-edge corridor planner (experimental)
  docs/
    PARAMETERS.md           parameter manual and symptom table
    STATUS.md               current status / known issues
    s_bend_tuning_guide.md  long tuning notes
  records/
    archive/                old run JSONs
    backups/s_bend_history_20261106.tar.gz   all pre-cleanup snapshots
    logs/                   sensor restart logs
```

User-facing tools stay at the top level so the usual commands still work:

```bash
./set_param.py --show
./analyze_dense_run.py dense_last_run.json --limit 35
```

Planner switch:

```bash
./set_param.py planner_mode dense      # current version (default)
./set_param.py planner_mode corridor   # experimental two-edge corridor
```