#!/usr/bin/env python3
"""Summarise one dense S-bend run (dense_last_run.json) for tuning.

Usage:
    python3 analyze_dense_run.py [dense_last_run.json] [--limit 16]

It prints the numbers that matter when deciding what to change next:
steering saturation, PWM/speed-loop behaviour, lane width, side flips,
the car's lateral offset from the fitted centreline, and why the last
plan was rejected (shared cones / spline curvature).
"""
import argparse
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
import sys
for _sub in ('core', 'planners/dense'):
    _path = str(HERE/_sub)
    if _path not in sys.path:
        sys.path.insert(0, _path)
sys.path.insert(0, str(HERE))

try:
    from dense_reference import entrance_chain, fit_curve
except Exception:                                    # pragma: no cover
    entrance_chain = fit_curve = None
try:
    from geometry import relative_to_path
except Exception:                                    # pragma: no cover
    relative_to_path = None


class Car:
    width = 1.50
    radius = 0.83
    front = 0.40
    rear = 0.16
    half_width = 0.175
    margin = 0.055
    servo_center = 84.5
    encoder_scale = 1.009
    laser_x = 0.23


def stats(values):
    if not values:
        return "n=0"
    return ("n=%d  min=%.2f  med=%.2f  max=%.2f"
            % (len(values), min(values), float(np.median(values)), max(values)))


def longest_run(values, pred):
    best = run = 0
    start = best_start = None
    for i, v in enumerate(values):
        if pred(v):
            run += 1
            if run == 1:
                start = i
            if run > best:
                best, best_start = run, start
        else:
            run = 0
    return best, best_start


def group_episodes(times, gap=0.16):
    """Collapse event timestamps into [first, last] episodes."""
    episodes = []
    for t in sorted(times):
        if episodes and t - episodes[-1][1] <= gap:
            episodes[-1][1] = t
        else:
            episodes.append([t, t])
    return episodes


def longest_halt(decisions, t0, drive_at=None):
    """Longest stretch where the car was not allowed to drive.

    Starts at the first drive command: waiting for the route before the car
    has ever moved is the normal launch sequence, not a halt.
    """
    best, start = 0.0, None
    for d in decisions:
        t = d["at"] - t0
        if drive_at is not None and t < drive_at:
            continue
        driving = bool(d.get("clear")) and d.get("hold") == "clear"
        if driving:
            start = None
        else:
            if start is None:
                start = t
            best = max(best, t - start)
    return best


def criteria_report(stage, target, floor, a_coast, reaction, stop, decisions, t0,
                    offsets, limit, window, neutral, triples, ers, plan_ms, flips,
                    drive_at=None, pair_mode=False):
    """Print the per-stage PASS/FAIL lines from section 6 of the plan."""
    stage = str(stage)
    neutral_limit = {"1": .15, "2": .12, "3": .10}.get(stage, .15)
    bad = ("blocked_front", "cone_corridor_lost", "sensor_or_driver_unavailable",
           "other_command_publisher", "reverse_motion", "control_delay")
    results = []

    halt = longest_halt(decisions, t0, drive_at) if decisions else None
    results.append(("terminal stop", stop not in bad and (halt is None or halt <= 1.0),
                    "stop=%s, longest ALIGN/WAIT run %s"
                    % (stop or "manual", "n/a" if halt is None else "%.2f s" % halt)))

    if not decisions:
        results.append(("gate rejects", False, "no control_decision events: rerun with the new code"))
        cs = gaps = None
    else:
        rejects = [d for d in decisions if not d.get("clear")
                   and (d.get("v") or 0.0) > 0.10]
        episodes = group_episodes([d["at"] - t0 for d in rejects])
        longest = max((b - a for a, b in episodes), default=0.0)
        results.append(("gate rejects while moving", len(episodes) <= 2 and longest <= 0.2,
                        "%d episodes, longest %.2f s" % (len(episodes), longest)))
        cs = [d["clearance"] for d in decisions if d.get("clearance") is not None]
        below = [c for c in cs if c < floor - 1e-9]
        results.append(("min clearance", bool(cs) and len(below) <= 1,
                        "min C=%.2f m (floor %.2f), %d frame(s) below"
                        % (min(cs) if cs else -1, floor, len(below))))
        gaps = [d["clearance"] - (d["vt"] * reaction + d["vt"] ** 2 / (2 * a_coast) + floor)
                for d in decisions
                if d.get("clearance") is not None and d.get("vt") is not None]
        results.append(("stopping distance fits", bool(gaps) and min(gaps) >= -0.05,
                        "min C-H(vt)=%s m"
                        % ("n/a" if not gaps else "%+.2f" % min(gaps))))

    if window:
        share = neutral / float(len(window))
        results.append(("neutral share in drive window", share <= neutral_limit,
                        "%.0f%% (%d/%d), limit %.0f%%"
                        % (100 * share, neutral, len(window), 100 * neutral_limit)))
    else:
        results.append(("neutral share in drive window", False, "the car never drove"))

    if ers:
        vals = [abs(e) for _, e in ers]
        med, p90 = float(np.median(vals)), float(np.percentile(vals, 90))
        med_lim = 20.0 if stage == "3" else 15.0
        p90_lim = 38.0 if stage == "3" else 32.0
        results.append(("tracking (heading error)", med <= med_lim and p90 <= p90_lim,
                        "median %.0f deg (limit %.0f), p90 %.0f deg (limit %.0f)"
                        % (med, med_lim, p90, p90_lim)))
    else:
        results.append(("tracking (heading error)", False, "no plan targets"))

    if offsets:
        held = sum(1 for o in offsets if abs(o) >= limit - 0.5) / float(len(offsets))
        results.append(("steering at limit", held < .15, "%.0f%% at +/-%.0f deg" % (100 * held, limit)))
    if plan_ms:
        p95 = float(np.percentile(plan_ms, 95))
        results.append(("plan time", p95 < 25 and max(plan_ms) < 50,
                        "p95=%.1f ms max=%.1f ms" % (p95, max(plan_ms))))

    if triples:
        vs = [v for _, v, _, _ in triples]
        straight = [v for _, v, _, servo in triples if abs(servo - 84.5) < 5.0]
        p10 = float(np.percentile(vs, 10))
        med = float(np.median(straight)) if straight else 0.0
        results.append(("speed profile",
                        p10 >= .60 * target and med >= .85 * target,
                        "straight median %.2f m/s (%.0f%% of target), p10 %.2f m/s (%.0f%%)"
                        % (med, 100 * med / target, p10, 100 * p10 / target)))
        low = sum(1 for v in vs if v < 0)
        results.append(("no reverse", low == 0, "%d negative-speed samples" % low))
    else:
        results.append(("speed profile", False, "no command v samples: rerun with the new code"))

    if pair_mode:
        # Bilateral pairing has no left/right state to flip; the centreline is
        # built from both rows every frame.
        results.append(("side flips", True, "pair mode: bilateral centreline"))
    else:
        results.append(("side flips", flips == 1, "%d flips (S bend expects 1)" % flips))

    print("criteria (stage %s, target %.2f m/s)" % (stage, target))
    for name, ok, detail in results:
        print("  %s %-32s %s" % ("PASS" if ok else "FAIL", name, detail))
    print("  => %s" % ("PASS" if all(ok for _, ok, _ in results) else "FAIL"))
    print()


def centreline_offset(cones, side, width):
    """Lateral position of the fitted centreline at x=1.0 m (car frame)."""
    if entrance_chain is None or cones is None or len(cones) < 3 or side not in (1, -1):
        return None
    car = Car()
    row = entrance_chain(np.asarray(cones, float), side, car, None, width)
    fit = fit_curve(row)
    if fit is None:
        return None
    curve, tangent, _ = fit
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    centres = curve - side * (width / 2.0) * normal
    if not (centres[0, 0] <= 1.0 <= centres[-1, 0]):
        return None
    return float(np.interp(1.0, centres[:, 0], centres[:, 1]))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("record", nargs="?", default=str(HERE / "dense_last_run.json"))
    ap.add_argument("--limit", type=float, default=None,
                    help="steering limit used in AnglePD, deg offset "
                         "(defaults to max_steer_deg from the record)")
    ap.add_argument("--criteria", metavar="STAGE", default=None,
                    help="print the stage 1/2/3 PASS/FAIL lines from the plan")
    args = ap.parse_args()
    path = Path(args.record)
    record = json.loads(path.read_text(encoding="utf-8"))
    if args.limit is None:
        args.limit = float(record.get("settings", {}).get("max_steer_deg", 16.0))
    events = record["events"]
    plans = [e for e in events if e["kind"] == "dense_plan"]
    commands = [e for e in events if e["kind"] == "command"]
    if not events:
        print("empty record")
        return
    t0 = events[0]["at"]
    end = max(e["at"] for e in events) - t0

    print("file      :", path)
    print("mode/stop :", record.get("mode"), "/", record.get("stop"))
    print("duration  : %.2f s   frames=%d  commands=%d" % (end, len(plans), len(commands)))
    print("settings  :", json.dumps(record.get("settings", {}), ensure_ascii=False))
    print()

    # ---- steering -------------------------------------------------------
    servo = [c["servo"] for c in commands]
    offsets = [s - Car.servo_center for s in servo]
    print("steering")
    bins = Counter(int(round(o / 2.0)) * 2 for o in offsets)
    print("  offset histogram (2 deg bins):",
          "  ".join("%+d:%d" % (k, bins[k]) for k in sorted(bins)))
    held = sum(1 for o in offsets if abs(o) >= args.limit - 0.5)
    print("  at +/-%.0f limit: %d/%d (%.0f%%)"
          % (args.limit, held, len(offsets), 100.0 * held / max(1, len(offsets))))
    run, start = longest_run(offsets, lambda o: abs(o) >= args.limit - 0.5)
    if run and start is not None:
        print("  longest run at limit: %d commands (%.2f s)"
              % (run, commands[start + run - 1]["at"] - commands[start]["at"]))
    print()

    # ---- speed ----------------------------------------------------------
    pwms = [c["pwm"] for c in commands]
    print("speed loop (PWM us)")
    print("  ", stats(pwms))
    print("  at 1500: %d   at >=1557.5: %d" % (sum(1 for p in pwms if p <= 1500.5),
                                               sum(1 for p in pwms if p >= 1557.5)))
    print()

    # ---- governor / safety gate ----------------------------------------
    settings = record.get("settings", {})
    floor = float(settings.get("gate_floor_m", .30))
    a_coast = float(settings.get("coast_decel_mps2", 1.5))
    reaction = float(settings.get("reaction_time_s", .20))
    target = float(settings.get("target_speed_mps", 1.0))
    decisions = [e for e in events if e.get("kind") == "control_decision"]
    drive = [c for c in commands if c.get("pwm", 0) > 1500.5]
    drive_at = drive[0]["at"] - t0 if drive else None
    window = [c for c in commands
              if drive_at is not None and c["at"] - t0 >= drive_at]
    neutral = sum(1 for c in window if c.get("pwm", 0) <= 1500.5)
    triples = [(c["at"] - t0, c.get("v"), c.get("vt"), c.get("servo", 84.5))
               for c in commands if c.get("v") is not None]
    # The PD heading error is the tracking signal the governor latches on. A
    # centreline-relative lateral error cannot be measured from the plan: the
    # visible centreline starts up to 1.1 m ahead of the car.
    weight = float(settings.get("preview_heading_weight", .35))
    ers = []
    for p in plans:
        if p.get("target") is not None:
            err = math.degrees(math.atan2(p["target"][1], p["target"][0])
                               + weight * p["heading"])
            ers.append((p["at"] - t0, err))
    print("governor")
    if drive_at is None:
        print("  never drove: no command above 1500 us")
    else:
        print("  first drive command: t=%.2f s" % drive_at)
    if window:
        print("  neutral in drive window: %d/%d (%.0f%%)"
              % (neutral, len(window), 100.0 * neutral / len(window)))
    if triples:
        vs = [v for _, v, _, _ in triples]
        vts = [vt for _, _, vt, _ in triples]
        print("  speed m/s     :", stats(vs))
        print("  v_target m/s  :", stats(vts))
        straight = [v for _, v, _, servo in triples if abs(servo - 84.5) < 5.0]
        if straight:
            print("  straight-leg median: %.2f m/s (%.0f%% of target)"
                  % (float(np.median(straight)),
                     100.0 * float(np.median(straight)) / target))
    else:
        print("  (no command v samples: record predates the governor)")
    if ers:
        vals = [abs(e) for _, e in ers]
        print("  |error| deg   : n=%d med=%.0f p90=%.0f max=%.0f  >%0.f deg: %d"
              % (len(vals), float(np.median(vals)), float(np.percentile(vals, 90)),
                 max(vals), settings.get("heading_error_enter_deg", 28.0),
                 sum(1 for v in vals if v > settings.get("heading_error_enter_deg", 28.0))))
    if decisions:
        reasons = Counter(d.get("reason") for d in decisions)
        print("  v_target reasons:", ", ".join("%s=%d" % (k, v)
                                               for k, v in reasons.most_common()))
        cs = [d["clearance"] for d in decisions if d.get("clearance") is not None]
        if cs:
            print("  clearance C m : min=%.2f med=%.2f (floor %.2f)"
                  % (min(cs), float(np.median(cs)), floor))
        gaps = [d["clearance"] - (d["vt"] * reaction + d["vt"] ** 2 / (2 * a_coast) + floor)
                for d in decisions
                if d.get("clearance") is not None and d.get("vt") is not None]
        if gaps:
            print("  C - H(v_target): min=%+.2f m (negative = asked for more than it had)"
                  % min(gaps))
        rejects = [d["at"] - t0 for d in decisions if not d.get("clear")]
        episodes = group_episodes(rejects)
        longest = max((b - a for a, b in episodes), default=0.0)
        print("  gate rejections: %d frames in %d episodes, longest %.2f s"
              % (len(rejects), len(episodes), longest))
    else:
        print("  (no control_decision events: record predates the governor)")
    print()

    # ---- planning -------------------------------------------------------
    widths = [p["width"] for p in plans if p["width"]]
    looks = [p["lookahead"] for p in plans if p["lookahead"]]
    targets_x = [p["target"][0] for p in plans if p["target"]]
    plan_ms = [p["plan_ms"] for p in plans]
    flips = sum(1 for a, b in zip(plans, plans[1:])
                if a["side"] is not None and b["side"] is not None and a["side"] != b["side"])
    pair_mode = any(p.get("side") == 0 for p in plans)
    print("planner")
    print("  width m    :", stats(widths))
    print("  lookahead m:", stats(looks))
    print("  target x m :", stats(targets_x))
    print("  plan ms    :", stats(plan_ms))
    print("  side flips :", flips)
    none_targets = sum(1 for p in plans if p["target"] is None)
    print("  no-target frames:", none_targets)

    # Width freezing is how a run usually ends: measure_width() returns None
    # for a few frames, then the 0.55 s stale guard drops the route.
    seq = [(p["at"] - t0, p["width"]) for p in plans if p["width"]]
    frozen_n, gap = 0, 0.0
    if seq:
        last_t, last_w = seq[0]
        frozen_from = None
        for t, w in seq[1:]:
            if abs(w - last_w) > 0.002:
                last_t, last_w = t, w
                frozen_from = None
            elif frozen_from is None:
                frozen_from = t
        stop_t = plans[-1]["at"] - t0 if plans else max(e["at"] for e in events) - t0
        gap = stop_t - last_t
        frozen_n = sum(1 for t, w in seq if t > last_t + 1e-6)
        print("  last width change: t=%.2fs  %.1f s before stop  (%d frozen frames)"
              % (last_t, gap, frozen_n))
        if frozen_n >= 3:
            print("  WARNING: width frozen for %d frames (measure_width failed);"
                  " the stale guard (1.0 s now, 0.55 s at run time) drops the route"
                  % frozen_n)
    print()

    # ---- lateral offset over time --------------------------------------
    if entrance_chain is not None:
        default_w = float(np.median(widths)) if widths else 1.42
        print("lateral offset of centreline at x=1.0 m (car frame; + = line is left of car)")
        rows = []
        for p in plans:
            w = p["width"] or default_w
            off = centreline_offset(p["cones"], p["side"], w)
            rows.append((p["at"] - t0, off))
        for sec in range(int(end) + 1):
            vals = [o for t, o in rows if sec <= t < sec + 1 and o is not None]
            if vals:
                print("  t=%2ds  n=%2d  mean=%+.2f  min=%+.2f  max=%+.2f"
                      % (sec, len(vals), float(np.mean(vals)), min(vals), max(vals)))
        print()

    # ---- both-row stats (row_stats) -----------------------------------
    if plans and plans[0].get("row_stats"):
        print("both-row stats (L/R: points, density, end heading deg)")
        series = []
        for p in plans:
            raw = p.get("row_stats") or {}
            row_stats = {int(k): v for k, v in raw.items()}
            series.append((p["at"]-t0, row_stats))
        for sec in range(int(end)+1):
            sel = [(t, e) for t, e in series if sec <= t < sec+1]
            if not sel:
                continue
            for side, name in ((1, "L"), (-1, "R")):
                pts = [e[side].get("points") for _, e in sel
                       if side in e and e[side].get("points") is not None]
                den = [e[side].get("density") for _, e in sel
                       if side in e and e[side].get("density") is not None]
                hd = [e[side].get("heading_end_deg") for _, e in sel
                      if side in e and e[side].get("heading_end_deg") is not None]
                if pts and den and hd:
                    print("  t=%2ds %s n=%d..%d dens=%.2f..%.2f head=%+.0f..%+.0f"
                          % (sec, name, min(pts), max(pts),
                             min(den), max(den), min(hd), max(hd)))
        print()

    # ---- last plan diagnosis -------------------------------------------
    last = plans[-1] if plans else None
    if last is not None and (last["target"] is None or "corridor" in str(record.get("stop"))):
        print("last plan diagnosis (why the route may have been rejected)")
        cones = np.asarray(last["cones"], float) if last["cones"] else np.empty((0, 2))
        print("  cones in frame:", len(cones), " side:", last["side"])
        if entrance_chain is not None and len(cones) >= 3:
            car = Car()
            mean_w = float(np.median(widths)) if widths else 1.42
            left = entrance_chain(cones, 1, car, None, mean_w)
            right = entrance_chain(cones, -1, car, None, mean_w)
            print("  rows: left=%d  right=%d  (width used %.2f m)" % (len(left), len(right), mean_w))
            if len(left) and len(right):
                shared = np.linalg.norm(left[:, None, :] - right[None, :, :], axis=2) < 0.12
                print("  cones shared by both rows (<12 cm):", int(np.any(shared)))
            for name, row, side in (("left", left, 1), ("right", right, -1)):
                fit = fit_curve(row)
                if fit is None:
                    continue
                curve, tangent, curvature = fit
                distance = mean_w / 2.0
                denominator = 1.0 + side * distance * curvature
                normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
                centres = curve - side * distance * normal
                print("  %s fit: curvature %+.2f..%+.2f 1/m  min denominator %.3f  min dx %+.3f"
                      % (name, float(np.min(curvature)), float(np.max(curvature)),
                         float(np.min(denominator)), float(np.min(np.diff(centres[:, 0])))))
            print("  (denominator = 1 + side*distance*curvature; <= 0.15 means the")
            print("   fitted centreline would be tighter than the car's minimum radius)")
        print()

    # ---- verdict --------------------------------------------------------
    print("verdict")
    if offsets and held / len(offsets) > 0.20:
        print("  * steering sits on the +/-%.0f limit for %.0f%% of the run:"
              " raise the limit or soften the demand" % (args.limit, 100.0 * held / len(offsets)))
    if seq and frozen_n >= 3 and gap > 0.45:
        print("  * width was frozen for %d frames and the stop came %.2f s after"
              " the last width change: the stale guard is the likely trigger"
              % (frozen_n, gap))
    if widths and max(widths) - min(widths) > 0.10:
        print("  * lane width jumps %.2f m: check shared-cone dedup at the S transition"
              % (max(widths) - min(widths)))
    if flips > 2:
        print("  * %d side flips: the densest-row decision is noisy" % flips)
    if plan_ms and max(plan_ms) > 25:
        print("  * plan_ms peaks at %.0f ms: CPU/scan load" % max(plan_ms))
    if not any([offsets, widths, plan_ms]):
        print("  (no data)")

    if args.criteria:
        criteria_report(args.criteria, target, floor, a_coast, reaction,
                        str(record.get("stop", "")), decisions, t0, offsets,
                        args.limit, window, neutral, triples, ers, plan_ms, flips,
                        drive_at, pair_mode)


if __name__ == "__main__":
    main()
