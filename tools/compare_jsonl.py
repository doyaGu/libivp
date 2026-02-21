#!/usr/bin/env python3
"""compare_jsonl.py — Compare two IVP scenario JSONL snapshot files.

Computes per-frame per-object metrics:
  - Position error   (L2 norm)
  - Orientation error (quaternion geodesic distance in radians)
  - Linear velocity error (L2 norm)
  - Angular velocity error (L2 norm)
  - Energy error (absolute and relative)

Exits 0 on success (all within thresholds), 1 on failure.

Usage:
  compare_jsonl.py REFERENCE.jsonl TEST.jsonl [options]

Options:
  --pos-eps FLOAT        Position tolerance (default: 1e-3)
  --vel-eps FLOAT        Linear velocity tolerance (default: 1e-2)
  --orient-eps FLOAT     Orientation tolerance in radians (default: 1e-3)
  --angvel-eps FLOAT     Angular velocity tolerance (default: 1e-2)
  --energy-eps FLOAT     Absolute energy tolerance (default: 1e-1)
  --energy-rel-eps FLOAT Relative energy tolerance (default: 1e-2)
  --time-eps FLOAT       Time tolerance (default: 1e-9)
  --config FILE          JSON config with per-scenario thresholds
  --scenario NAME        Scenario name (for config lookup)
  --adaptive             Use adaptive thresholds (grow with time)
  --save-metrics FILE    Save per-scenario metrics summary to JSON
  --baseline FILE        Baseline metrics JSON for drift comparison
  --max-drift-percent F  Max allowed drift from baseline (default: 10.0)
    --timing-step-eps N    Allowed step mismatch for onset timing checks (default: 0)
    --timing-vel-jump-eps FLOAT
                                                 Velocity jump threshold for timing checks (default: 5e-3)
    --timing-angvel-jump-eps FLOAT
                                                 Angular-velocity jump threshold for timing checks (default: 5e-3)
    --disable-timing-check Disable onset timing checks
  --verbose              Print per-frame details
  --summary              Print summary statistics (default when not --verbose)
"""

import argparse
import json
import math
import sys
from pathlib import Path


def parse_vec3(d):
    """Parse {"x": ..., "y": ..., "z": ...} to (x, y, z) tuple."""
    return (d["x"], d["y"], d["z"])


def parse_quat(d):
    """Parse {"x":...,"y":...,"z":...,"w":...} to (x,y,z,w) tuple."""
    return (d["x"], d["y"], d["z"], d["w"])


def vec3_dist(a, b):
    """L2 distance between two 3-vectors."""
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def quat_geodesic(q1, q2):
    """Geodesic angle between two unit quaternions in radians.

    Returns a value in [0, pi].
    """
    dot = sum(a * b for a, b in zip(q1, q2))
    # Handle double cover: q and -q represent same rotation
    dot = abs(dot)
    dot = min(dot, 1.0)  # clamp for numerical safety
    return 2.0 * math.acos(dot)


def load_jsonl(path):
    """Load a JSONL file into a list of dicts."""
    frames = []
    with open(path, "r") as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                frames.append(json.loads(line))
            except json.JSONDecodeError as e:
                print(f"ERROR: {path}:{line_num}: {e}", file=sys.stderr)
                sys.exit(2)
    return frames


def load_config(config_path, scenario_name):
    """Load per-scenario thresholds from config JSON."""
    with open(config_path, "r") as f:
        config = json.load(f)

    # Start with defaults
    thresholds = dict(config.get("defaults", {}))

    # Override with scenario-specific values
    if scenario_name and scenario_name in config.get("scenarios", {}):
        thresholds.update(config["scenarios"][scenario_name])

    return thresholds


class CompareMetrics:
    """Accumulates comparison metrics across all frames."""

    def __init__(self):
        self.pos_errors = []
        self.orient_errors = []
        self.vel_errors = []
        self.angvel_errors = []
        self.energy_abs_errors = []
        self.energy_rel_errors = []
        self.time_errors = []
        self.frame_count = 0
        self.object_count = 0
        self.timing_check_count = 0
        self.timing_mismatch_count = 0
        self.failures = []

    def add_frame_time_error(self, step, ref_time, test_time):
        err = abs(ref_time - test_time)
        self.time_errors.append(err)

    def add_object_error(self, step, obj_id, pos_err, orient_err, vel_err,
                         angvel_err, energy_abs_err, energy_rel_err):
        self.pos_errors.append(pos_err)
        self.orient_errors.append(orient_err)
        self.vel_errors.append(vel_err)
        self.angvel_errors.append(angvel_err)
        self.energy_abs_errors.append(energy_abs_err)
        self.energy_rel_errors.append(energy_rel_err)
        self.object_count += 1

    def record_failure(self, msg):
        self.failures.append(msg)

    def max_pos_error(self):
        return max(self.pos_errors) if self.pos_errors else 0.0

    def max_orient_error(self):
        return max(self.orient_errors) if self.orient_errors else 0.0

    def max_vel_error(self):
        return max(self.vel_errors) if self.vel_errors else 0.0

    def max_angvel_error(self):
        return max(self.angvel_errors) if self.angvel_errors else 0.0

    def max_energy_abs_error(self):
        return max(self.energy_abs_errors) if self.energy_abs_errors else 0.0

    def max_energy_rel_error(self):
        return max(self.energy_rel_errors) if self.energy_rel_errors else 0.0

    def max_time_error(self):
        return max(self.time_errors) if self.time_errors else 0.0

    def rms(self, values):
        if not values:
            return 0.0
        return math.sqrt(sum(v * v for v in values) / len(values))

    def mean(self, values):
        if not values:
            return 0.0
        return sum(values) / len(values)

    def to_dict(self):
        return {
            "frame_count": self.frame_count,
            "object_comparisons": self.object_count,
            "position": {
                "max": self.max_pos_error(),
                "mean": self.mean(self.pos_errors),
                "rms": self.rms(self.pos_errors),
            },
            "orientation": {
                "max": self.max_orient_error(),
                "mean": self.mean(self.orient_errors),
                "rms": self.rms(self.orient_errors),
            },
            "linear_velocity": {
                "max": self.max_vel_error(),
                "mean": self.mean(self.vel_errors),
                "rms": self.rms(self.vel_errors),
            },
            "angular_velocity": {
                "max": self.max_angvel_error(),
                "mean": self.mean(self.angvel_errors),
                "rms": self.rms(self.angvel_errors),
            },
            "energy_absolute": {
                "max": self.max_energy_abs_error(),
                "mean": self.mean(self.energy_abs_errors),
                "rms": self.rms(self.energy_abs_errors),
            },
            "energy_relative": {
                "max": self.max_energy_rel_error(),
                "mean": self.mean(self.energy_rel_errors),
                "rms": self.rms(self.energy_rel_errors),
            },
            "time": {
                "max": self.max_time_error(),
            },
            "timing": {
                "checks": self.timing_check_count,
                "mismatches": self.timing_mismatch_count,
            },
            "failure_count": len(self.failures),
            "failures": self.failures[:20],  # cap for readability
        }


def vec3_length(v):
    return math.sqrt(sum(x * x for x in v))


def build_object_step_series(frames):
    """Return {obj_id: [(step, object_dict), ...]} across all frames."""
    series = {}
    for i, frame in enumerate(frames):
        step = frame.get("step", i)
        for j, obj in enumerate(frame.get("objects", [])):
            obj_id = obj.get("id", j)
            series.setdefault(obj_id, []).append((step, obj))
    return series


def first_jump_step(step_series, field_name, jump_eps):
    """First step where successive vector magnitudes jump beyond jump_eps."""
    if len(step_series) < 2:
        return None
    prev_mag = None
    for step, obj in step_series:
        vec = parse_vec3(obj[field_name])
        mag = vec3_length(vec)
        if prev_mag is not None and abs(mag - prev_mag) > jump_eps:
            return step
        prev_mag = mag
    return None


def compare_onset_timing(ref_frames, test_frames, metrics,
                         timing_step_eps,
                         vel_jump_eps,
                         angvel_jump_eps,
                         verbose=False):
    """Compare first onset steps of velocity/ang-velocity jumps per object id."""
    ref_series = build_object_step_series(ref_frames)
    test_series = build_object_step_series(test_frames)
    common_ids = sorted(set(ref_series.keys()) & set(test_series.keys()))

    for obj_id in common_ids:
        ref_steps = ref_series[obj_id]
        test_steps = test_series[obj_id]

        ref_vel_step = first_jump_step(ref_steps, "lin_vel", vel_jump_eps)
        test_vel_step = first_jump_step(test_steps, "lin_vel", vel_jump_eps)
        ref_ang_step = first_jump_step(ref_steps, "ang_vel", angvel_jump_eps)
        test_ang_step = first_jump_step(test_steps, "ang_vel", angvel_jump_eps)

        for label, r, t in (
            ("lin_vel_jump", ref_vel_step, test_vel_step),
            ("ang_vel_jump", ref_ang_step, test_ang_step),
        ):
            if r is None and t is None:
                continue
            metrics.timing_check_count += 1
            if r is None or t is None or abs(r - t) > timing_step_eps:
                metrics.timing_mismatch_count += 1
                msg = (f"timing obj {obj_id} {label}: ref={r} test={t} "
                       f"(allowed_step_delta={timing_step_eps})")
                metrics.record_failure(msg)
                if verbose:
                    print(f"  FAIL: {msg}", file=sys.stderr)


def compare_frames(ref_frames, test_frames, thresholds, adaptive=False,
                   verbose=False,
                   timing_step_eps=0,
                   timing_vel_jump_eps=5e-3,
                   timing_angvel_jump_eps=5e-3,
                   timing_check_enabled=True):
    """Compare two lists of JSONL frames. Returns CompareMetrics."""
    metrics = CompareMetrics()

    pos_eps = thresholds.get("pos_eps", 1e-3)
    orient_eps = thresholds.get("orient_eps", 1e-3)
    vel_eps = thresholds.get("vel_eps", 1e-2)
    angvel_eps = thresholds.get("angvel_eps", 1e-2)
    energy_eps = thresholds.get("energy_eps", 1e-1)
    energy_rel_eps = thresholds.get("energy_rel_eps", 1e-2)
    time_eps = thresholds.get("time_eps", 1e-9)

    if len(ref_frames) != len(test_frames):
        metrics.record_failure(
            f"Frame count mismatch: ref={len(ref_frames)} test={len(test_frames)}"
        )
        # Compare up to the shorter length
        n = min(len(ref_frames), len(test_frames))
    else:
        n = len(ref_frames)

    metrics.frame_count = n

    for i in range(n):
        ref = ref_frames[i]
        test = test_frames[i]

        step = ref.get("step", i)

        # Adaptive scaling: thresholds grow linearly with step count
        if adaptive and step > 0:
            adapt_factor = 1.0 + 0.01 * step  # 1% per step
        else:
            adapt_factor = 1.0

        # Compare time
        ref_time = ref.get("time", 0.0)
        test_time = test.get("time", 0.0)
        time_err = abs(ref_time - test_time)
        metrics.add_frame_time_error(step, ref_time, test_time)
        if time_err > time_eps * adapt_factor:
            msg = f"step {step}: time error {time_err:.2e} > {time_eps * adapt_factor:.2e}"
            metrics.record_failure(msg)
            if verbose:
                print(f"  FAIL: {msg}", file=sys.stderr)

        # Compare objects
        ref_objs = ref.get("objects", [])
        test_objs = test.get("objects", [])
        if len(ref_objs) != len(test_objs):
            msg = f"step {step}: object count mismatch ref={len(ref_objs)} test={len(test_objs)}"
            metrics.record_failure(msg)
            continue

        test_by_id = {}
        duplicate_test_ids = set()
        for j, to in enumerate(test_objs):
            tid = to.get("id", j)
            if tid in test_by_id:
                duplicate_test_ids.add(tid)
            test_by_id[tid] = to
        if duplicate_test_ids:
            msg = f"step {step}: duplicate object ids in test={sorted(duplicate_test_ids)}"
            metrics.record_failure(msg)
            continue

        for j in range(len(ref_objs)):
            ro = ref_objs[j]
            obj_id = ro.get("id", j)
            to = test_by_id.get(obj_id)
            if not to:
                msg = f"step {step}: missing object id in test: {obj_id}"
                metrics.record_failure(msg)
                continue

            ref_type = ro.get("type")
            test_type = to.get("type")
            if ref_type is not None and test_type is not None and ref_type != test_type:
                msg = f"step {step} obj {obj_id}: type mismatch ref={ref_type} test={test_type}"
                metrics.record_failure(msg)

            # Position
            rp = parse_vec3(ro["pos"])
            tp = parse_vec3(to["pos"])
            pos_err = vec3_dist(rp, tp)

            # Orientation
            rq = parse_quat(ro["quat"])
            tq = parse_quat(to["quat"])
            orient_err = quat_geodesic(rq, tq)

            # Linear velocity
            rv = parse_vec3(ro["lin_vel"])
            tv = parse_vec3(to["lin_vel"])
            vel_err = vec3_dist(rv, tv)

            # Angular velocity
            raw = parse_vec3(ro["ang_vel"])
            taw = parse_vec3(to["ang_vel"])
            angvel_err = vec3_dist(raw, taw)

            # Energy
            re = ro["energy"]
            te = to["energy"]
            energy_abs_err = abs(re - te)
            energy_ref_mag = abs(re) if abs(re) > 1e-12 else 1.0
            energy_rel_err = energy_abs_err / energy_ref_mag

            metrics.add_object_error(step, obj_id, pos_err, orient_err,
                                     vel_err, angvel_err,
                                     energy_abs_err, energy_rel_err)

            # Check thresholds
            eff_pos = pos_eps * adapt_factor
            eff_orient = orient_eps * adapt_factor
            eff_vel = vel_eps * adapt_factor
            eff_angvel = angvel_eps * adapt_factor
            eff_energy = energy_eps * adapt_factor
            eff_energy_rel = energy_rel_eps * adapt_factor

            fails_here = []
            if pos_err > eff_pos:
                fails_here.append(f"pos={pos_err:.2e}>{eff_pos:.2e}")
            if orient_err > eff_orient:
                fails_here.append(f"orient={orient_err:.2e}>{eff_orient:.2e}")
            if vel_err > eff_vel:
                fails_here.append(f"vel={vel_err:.2e}>{eff_vel:.2e}")
            if angvel_err > eff_angvel:
                fails_here.append(f"angvel={angvel_err:.2e}>{eff_angvel:.2e}")
            if energy_abs_err > eff_energy:
                fails_here.append(f"energy_abs={energy_abs_err:.2e}>{eff_energy:.2e}")
            if energy_rel_err > eff_energy_rel:
                fails_here.append(f"energy_rel={energy_rel_err:.2e}>{eff_energy_rel:.2e}")

            if fails_here:
                msg = f"step {step} obj {obj_id}: {'; '.join(fails_here)}"
                metrics.record_failure(msg)
                if verbose:
                    print(f"  FAIL: {msg}", file=sys.stderr)

        ref_ids = {ro.get("id", idx) for idx, ro in enumerate(ref_objs)}
        extra_test_ids = [tid for tid in test_by_id.keys() if tid not in ref_ids]
        if extra_test_ids:
            msg = f"step {step}: extra object ids in test={sorted(extra_test_ids)}"
            metrics.record_failure(msg)
            if verbose:
                print(f"  FAIL: {msg}", file=sys.stderr)

    if timing_check_enabled:
        compare_onset_timing(ref_frames, test_frames, metrics,
                             timing_step_eps,
                             timing_vel_jump_eps,
                             timing_angvel_jump_eps,
                             verbose=verbose)

    return metrics


def check_baseline_drift(metrics, baseline_path, max_drift_pct):
    """Compare current metrics against a saved baseline.

    Returns list of drift warnings/failures.
    """
    with open(baseline_path, "r") as f:
        baseline = json.load(f)

    current = metrics.to_dict()
    drift_failures = []

    for category in ["position", "orientation", "linear_velocity",
                     "angular_velocity", "energy_absolute", "energy_relative"]:
        for stat in ["max", "rms"]:
            base_val = baseline.get(category, {}).get(stat, 0.0)
            curr_val = current.get(category, {}).get(stat, 0.0)

            if base_val > 1e-15:
                drift_pct = ((curr_val - base_val) / base_val) * 100.0
                if drift_pct > max_drift_pct:
                    drift_failures.append(
                        f"{category}.{stat}: {curr_val:.2e} vs baseline {base_val:.2e} "
                        f"(+{drift_pct:.1f}% > {max_drift_pct:.1f}% limit)"
                    )
            elif curr_val > 1e-12:
                drift_failures.append(
                    f"{category}.{stat}: {curr_val:.2e} vs baseline ~0 (new regression)"
                )

    return drift_failures


def main():
    parser = argparse.ArgumentParser(
        description="Compare two IVP scenario JSONL snapshot files"
    )
    parser.add_argument("reference", help="Reference JSONL file")
    parser.add_argument("test", help="Test JSONL file")
    parser.add_argument("--pos-eps", type=float, default=1e-3)
    parser.add_argument("--vel-eps", type=float, default=1e-2)
    parser.add_argument("--orient-eps", type=float, default=1e-3)
    parser.add_argument("--angvel-eps", type=float, default=1e-2)
    parser.add_argument("--energy-eps", type=float, default=1e-1)
    parser.add_argument("--energy-rel-eps", type=float, default=1e-2)
    parser.add_argument("--time-eps", type=float, default=1e-9)
    parser.add_argument("--config", help="JSON config with per-scenario thresholds")
    parser.add_argument("--scenario", help="Scenario name (for config lookup)")
    parser.add_argument("--adaptive", action="store_true",
                        help="Use adaptive thresholds that grow with step count")
    parser.add_argument("--save-metrics", help="Save metrics summary to JSON file")
    parser.add_argument("--baseline", help="Baseline metrics JSON for drift comparison")
    parser.add_argument("--max-drift-percent", type=float, default=10.0,
                        help="Max allowed drift from baseline (%%)")
    parser.add_argument("--timing-step-eps", type=int, default=0,
                        help="Allowed step mismatch for onset timing checks")
    parser.add_argument("--timing-vel-jump-eps", type=float, default=5e-3,
                        help="Velocity jump threshold for timing checks")
    parser.add_argument("--timing-angvel-jump-eps", type=float, default=5e-3,
                        help="Angular-velocity jump threshold for timing checks")
    parser.add_argument("--disable-timing-check", action="store_true",
                        help="Disable onset timing checks")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--summary", action="store_true",
                        help="Print summary (default when not --verbose)")
    args = parser.parse_args()

    # Build thresholds
    thresholds = {
        "pos_eps": args.pos_eps,
        "orient_eps": args.orient_eps,
        "vel_eps": args.vel_eps,
        "angvel_eps": args.angvel_eps,
        "energy_eps": args.energy_eps,
        "energy_rel_eps": args.energy_rel_eps,
        "time_eps": args.time_eps,
    }

    # Override from config if provided
    if args.config:
        config_thresholds = load_config(args.config, args.scenario)
        thresholds.update(config_thresholds)

    # Load files
    ref_frames = load_jsonl(args.reference)
    test_frames = load_jsonl(args.test)

    if not ref_frames:
        print("ERROR: Reference file is empty", file=sys.stderr)
        sys.exit(2)
    if not test_frames:
        print("ERROR: Test file is empty", file=sys.stderr)
        sys.exit(2)

    # Compare
    metrics = compare_frames(ref_frames, test_frames, thresholds,
                             adaptive=args.adaptive,
                             verbose=args.verbose,
                             timing_step_eps=args.timing_step_eps,
                             timing_vel_jump_eps=args.timing_vel_jump_eps,
                             timing_angvel_jump_eps=args.timing_angvel_jump_eps,
                             timing_check_enabled=not args.disable_timing_check)

    # Save metrics if requested
    if args.save_metrics:
        metrics_dict = metrics.to_dict()
        if args.scenario:
            metrics_dict["scenario"] = args.scenario
        Path(args.save_metrics).parent.mkdir(parents=True, exist_ok=True)
        with open(args.save_metrics, "w") as f:
            json.dump(metrics_dict, f, indent=2)

        # Print summary
        if args.verbose or args.summary or not args.verbose:
          d = metrics.to_dict()
          scenario_label = args.scenario or "unknown"
          print(f"=== compare_jsonl: {scenario_label} ===")
          print(f"  frames: {d['frame_count']}  objects/frame: "
              f"{d['object_comparisons'] // max(d['frame_count'], 1)}")
          print(f"  position     max={d['position']['max']:.2e}  "
              f"rms={d['position']['rms']:.2e}")
          print(f"  orientation  max={d['orientation']['max']:.2e}  "
              f"rms={d['orientation']['rms']:.2e}")
          print(f"  lin_velocity max={d['linear_velocity']['max']:.2e}  "
              f"rms={d['linear_velocity']['rms']:.2e}")
          print(f"  ang_velocity max={d['angular_velocity']['max']:.2e}  "
              f"rms={d['angular_velocity']['rms']:.2e}")
          print(f"  energy_abs   max={d['energy_absolute']['max']:.2e}  "
              f"rms={d['energy_absolute']['rms']:.2e}")
          print(f"  energy_rel   max={d['energy_relative']['max']:.2e}  "
              f"rms={d['energy_relative']['rms']:.2e}")
          print(f"  time         max={d['time']['max']:.2e}")
          print(f"  timing       checks={d['timing']['checks']}  "
              f"mismatches={d['timing']['mismatches']}")
          print(f"  failures: {d['failure_count']}")
          if d['failures']:
            for fail in d['failures'][:10]:
                print(f"    {fail}")
            if d['failure_count'] > 10:
                print(f"    ... and {d['failure_count'] - 10} more")

    # Check baseline drift
    ok = True
    if args.baseline and Path(args.baseline).exists():
        drift_failures = check_baseline_drift(
            metrics, args.baseline, args.max_drift_percent
        )
        if drift_failures:
            print(f"\nBASELINE DRIFT FAILURES ({len(drift_failures)}):",
                  file=sys.stderr)
            for df in drift_failures:
                print(f"  {df}", file=sys.stderr)
            ok = False

    # Final verdict
    if metrics.failures:
        ok = False

    if ok:
        print("PASS")
        sys.exit(0)
    else:
        print("FAIL")
        sys.exit(1)


if __name__ == "__main__":
    main()
