# S-bend dense planner

LiDAR boundary planner with wheel/IMU pose tracking for the S-bend / ring track.

The default bilateral planner and full-run pose journal are described in
`docs/BILATERAL.md`.

## Layout

```
dense_s_bend.py            ROS 2 node (speed PI + steering PD + records)
dense_settings.json        runtime parameters (read once at startup)
run_dense.sh               launcher (add --observe for commands-free run)
core/                      geometry, swept clearance, vehicle IO
planners/dense/            active dense boundary planner
analyze_dense_run.py       post-run tuning report
set_param.py               edit one setting safely (creates .bak)
docs/                      parameters, tuning guide, status, layout
records/                   run records and logs (pruned)
```

## Run

```bash
./run_dense.sh              # drive
./run_dense.sh --observe    # plan only, publishes no commands
```

## Analyse a run

```bash
./analyze_dense_run.py dense_last_run.json --limit 35
```

## Global map (after a run that recorded yaw/distance)

```bash
python3 make_global_map.py records/runs/your_run.jsonl global_map.png
```

It draws: global LiDAR points (used/filtered), both simulated side routes,
the planned centre route and the actually driven trajectory.

## Versions

`/root/racecar/tools/VERSION_BACKUPS.md`:

- 备份1 = 01:05 packaged version (`s_bend_ROLLBACK_20261107_0105`)
- 备份2 = last Saturday version (`s_bend_BACKUP2_LASTSAT_20261108_174639`)

## Notes

- Single active dense path; the corridor experiment was removed.
- The top-level `dense_reference.py` duplicate was removed; the active module is
  `planners/dense/dense_reference.py` and the analyser now imports that one.
- Records inside old backup directories were pruned; code snapshots remain.
