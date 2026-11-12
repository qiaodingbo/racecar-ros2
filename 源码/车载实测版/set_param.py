#!/usr/bin/env python3
"""Change one dense_settings.json value with a backup and a sanity check.

    ./set_param.py --show
    ./set_param.py target_speed_mps 0.30
    ./set_param.py max_steer_deg 30
    ./set_param.py            # same as --show

The node reads dense_settings.json only at startup, so restart
./run_dense.sh after changing anything.
"""
import argparse
import json
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PATH = HERE / 'dense_settings.json'
BACKUP = HERE / 'dense_settings.json.bak'


def validate(s):
    if not 0 < s['target_speed_mps'] <= 2.0:
        return 'target_speed_mps must be in (0, 2]'
    if s['distance_mode'] != 'half_width':
        return "distance_mode must stay 'half_width'"
    if not 1500 <= s['speed_feedforward_pwm'] <= s['maximum_pwm'] <= 1600:
        return 'need 1500 <= speed_feedforward_pwm <= maximum_pwm <= 1600'
    if not 1500 < s['launch_pwm'] <= s['maximum_pwm']:
        return 'need 1500 < launch_pwm <= maximum_pwm'
    if not 10.0 <= s.get('max_steer_deg', 16.0) <= s.get('full_steer_deg', 65.0) <= 90.0:
        return 'need 10 <= max_steer_deg <= full_steer_deg <= 90'
    if not .20 <= s.get('follow_offset_m', .35) <= .60:
        return 'need 0.20 <= follow_offset_m <= 0.60'
    if not 0 < s.get('launch_speed_mps', .4) <= s['target_speed_mps']:
        return 'need 0 < launch_speed_mps <= target_speed_mps'
    if not 1500 <= s.get('minimum_pwm', 1500) <= s['maximum_pwm']:
        return 'need 1500 <= minimum_pwm <= maximum_pwm'
    if s.get('planner_mode', 'dense') not in ('dense', 'corridor', 'freespace'):
        return "planner_mode must be 'dense', 'corridor' or 'freespace'"
    if not 0.002 <= s.get('curv_per_deg_left', 0.0089) <= 0.05:
        return 'need 0.002 <= curv_per_deg_left <= 0.05'
    if not 0.002 <= s.get('curv_per_deg_right', 0.0177) <= 0.05:
        return 'need 0.002 <= curv_per_deg_right <= 0.05'
    if not 0.0 <= s.get('angle_ff_gain', 0.0) <= 1.5:
        return 'need 0 <= angle_ff_gain <= 1.5'
    if not 40.0 <= s.get('steer_slew_deg_per_s', 150.0) <= 400.0:
        return 'need 40 <= steer_slew_deg_per_s <= 400'
    if not .25 <= s.get('corner_offset_m', .45) <= .60:
        return 'need 0.25 <= corner_offset_m <= 0.60'
    if s.get('start_side', 0) not in (-1, 0, 1):
        return 'start_side must be -1 (right), 0 (auto) or 1 (left)'
    # Speed governor and safety gate.
    if not .10 <= s.get('gate_floor_m', .30) < s.get('clearance_probe_m', 2.4) <= 3.0:
        return 'need 0.10 <= gate_floor_m < clearance_probe_m <= 3.0'
    if not .01 <= s.get('clearance_step_m', .05) <= .20:
        return 'need 0.01 <= clearance_step_m <= 0.20'
    if not 1 <= s.get('clearance_min_points', 2) <= 5:
        return 'need 1 <= clearance_min_points <= 5'
    if not 0 < s.get('heading_error_exit_deg', 18.) < s.get('heading_error_enter_deg', 28.) < 90.:
        return 'need 0 < heading_error_exit_deg < heading_error_enter_deg < 90'
    if not 0 <= s.get('v_lat_cap_mps', .65) <= s['target_speed_mps']:
        return 'need 0 <= v_lat_cap_mps <= target_speed_mps'
    if not 0 <= s.get('v_min_cruise_mps', .30) <= s['target_speed_mps']:
        return 'need 0 <= v_min_cruise_mps <= target_speed_mps'
    if not 0 <= s.get('curvature_ff_gain', .0) <= 1.5:
        return 'need 0 <= curvature_ff_gain <= 1.5'
    if not 0 < s.get('a_lat_mps2', 1.5) <= 5.0:
        return 'need 0 < a_lat_mps2 <= 5.0'
    if not 0 < s.get('a_accel_mps2', .8) <= s.get('a_decel_mps2', 3.0) <= 8.0:
        return 'need 0 < a_accel_mps2 <= a_decel_mps2 <= 8.0'
    if not 0 < s.get('coast_decel_mps2', 1.5) <= 5.0:
        return 'need 0 < coast_decel_mps2 <= 5.0'
    if not 0 <= s.get('reaction_time_s', .2) <= 1.0:
        return 'need 0 <= reaction_time_s <= 1.0'
    if not 0 <= s.get('curvature_percentile', 90) <= 100:
        return 'need 0 <= curvature_percentile <= 100'
    if not 0 < s.get('blocked_stop_s', 1.0) <= s.get('blocked_timeout_s', 2.0) <= 10.0:
        return 'need 0 < blocked_stop_s <= blocked_timeout_s <= 10.0'
    # Bilateral nearest-dense pairing.
    if s.get('steer_mode', 'pursuit') not in ('pursuit', 'pd'):
        return "steer_mode must be 'pursuit' or 'pd'"
    if not 0 < s.get('pursuit_gain', 1.0) <= 2.0:
        return 'need 0 < pursuit_gain <= 2.0'
    if not 0 <= s.get('pp_heading_gain', 0.5) <= 1.5:
        return 'need 0 <= pp_heading_gain <= 1.5'
    if not 0 < s.get('aim_ema_alpha', 0.45) <= 1.0:
        return 'need 0 < aim_ema_alpha <= 1.0'
    if s.get('pairing_mode', 'legacy') not in ('legacy', 'nearest_pair'):
        return "pairing_mode must be 'legacy' or 'nearest_pair'"
    if not 1.2 <= s.get('track_width_m', 1.5) <= 2.0:
        return 'need 1.2 <= track_width_m <= 2.0'
    if not .8 <= s.get('pair_near_range_m', 3.6) <= 5.0:
        return 'need 0.8 <= pair_near_range_m <= 4.0'
    if s.get('pairing_mode', 'legacy') == 'nearest_pair':
        activation = s.get('pair_activation_m', 2.0)
        if not 1.5 <= activation <= s.get('pair_near_range_m', 3.6):
            return 'need 1.5 <= pair_activation_m <= pair_near_range_m'
        if not activation+.4 <= s.get('pair_preview_range_m', 4.8) <= 5.0:
            return 'need pair_activation_m + 0.4 <= pair_preview_range_m <= 5.0'
        if s.get('pair_confirm_frames', 3) not in (2,3,4,5):
            return 'pair_confirm_frames must be an integer from 2 to 5'
        if not .15 <= s.get('corner_speed_mps', .35) <= 2.00:
            return 'need 0.15 <= corner_speed_mps <= 2.00'
    if not 3 <= s.get('pair_max_points', 8) <= 12:
        return 'need 3 <= pair_max_points <= 12'
    if not 1.2 <= s.get('pair_window_m', 2.4) <= 4.0:
        return 'need 1.2 <= pair_window_m <= 4.0'
    if not .4 <= s.get('pair_min_overlap_m', .6) <= 2.0:
        return 'need 0.4 <= pair_min_overlap_m <= 2.0'
    if not 0.0 <= s.get('pair_max_extension_m', .9) <= 2.0:
        return 'need 0 <= pair_max_extension_m <= 2.0'
    if not 0 < s.get('pair_max_curvature', 1.65) <= 3.0:
        return 'need 0 < pair_max_curvature <= 3.0'
    weights = sum(float(s.get(f'pair_weight_{name}', default)) for name, default in
                  (('near', .30), ('density', .25), ('similarity', .25),
                   ('length', .12), ('continuity', .08)))
    if abs(weights - 1.0) > 1e-6:
        return 'pair_weight_* must sum to 1.0 (now %.3f)' % weights
    if any(s.get('pair_weight_'+name, default) < 0 for name, default in
           (('near', .30), ('density', .25), ('similarity', .25),
            ('length', .12), ('continuity', .08))):
        return 'pair weights must be nonnegative'
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('key', nargs='?')
    ap.add_argument('value', nargs='?')
    ap.add_argument('--show', action='store_true')
    args = ap.parse_args()

    settings = json.loads(PATH.read_text(encoding='utf-8'))
    if args.show or not args.key:
        print(json.dumps(settings, indent=2, ensure_ascii=False))
        return
    if args.key not in settings:
        print('unknown key %r; known keys: %s'
              % (args.key, ', '.join(sorted(settings))))
        sys.exit(2)
    if args.value is None:
        print('%s = %r' % (args.key, settings[args.key]))
        return

    old = settings[args.key]
    if args.key in ('distance_mode', 'planner_mode', 'pairing_mode'):
        new = args.value
    elif isinstance(old, bool):
        text = args.value.strip().lower()
        if text not in ('true', 'false', '1', '0', 'on', 'off', 'yes', 'no'):
            print('value must be true or false')
            sys.exit(2)
        new = text in ('true', '1', 'on', 'yes')
    else:
        try:
            new = float(args.value)
        except ValueError:
            print('value must be a number')
            sys.exit(2)
        if isinstance(old, int) and new.is_integer():
            new = int(new)

    settings[args.key] = new
    error = validate(settings)
    if error:
        print('rejected: %s' % error)
        sys.exit(2)

    shutil.copy2(PATH, BACKUP)
    PATH.write_text(json.dumps(settings, indent=2, ensure_ascii=False) + '\n',
                    encoding='utf-8')
    print('%s: %r -> %r' % (args.key, old, new))
    print('backup: %s (previous values)' % BACKUP.name)
    print('restart ./run_dense.sh for it to take effect')


if __name__ == '__main__':
    main()




