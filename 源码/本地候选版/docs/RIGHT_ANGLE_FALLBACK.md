# Right-angle blind-left fallback

Enabled in `dense_settings.json` with `right_angle_fallback_enabled=true`.
The fallback is subordinate to normal cone planning and arms only after the
car has followed a recent valid corridor. It requires four consecutive fresh
lidar frames with no cone cluster inside the forward +/-45 degree, 0.25..2.5 m
sector. It then commands a one-shot left arc at 0.25 m/s and exits back to the
normal planner after at least 75 degrees of IMU-confirmed heading change and a
valid corridor. It stops the run on timeout or more than 105 degrees.

The sector uses narrow cone clusters, not individual lidar beams. Frozen
corner turn-in is in `planners/corner/right_angle.py`; all thresholds are in
`dense_settings.json`. Do not increase speed or disable `right_angle_once`
until observe-mode and low-speed tests have passed.
