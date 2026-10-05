"""Validate complete native trajectories and identify the first differing PSI."""
from __future__ import annotations

import re
import math
from typing import Any
from benchmark import BenchmarkError

CONTRACT = "native-trajectory-v1"
HASH_FIELDS = ("object_hash", "constraint_hash", "event_hash", "statistics_hash", "trajectory_hash")


def validate(records: list[dict[str, Any]]) -> tuple[list[dict], dict]:
    if not records or records[-1].get("kind") != "native-verification":
        raise BenchmarkError("verification trace has no terminal record")
    steps, end = records[:-1], records[-1]
    count = end.get("steps_per_world")
    if type(count) is not int or count < 1 or len(steps) != count + 1:
        raise BenchmarkError("verification trace is incomplete")
    if end.get("verification_contract") != CONTRACT:
        raise BenchmarkError("unsupported verification contract")
    if not isinstance(end.get("dt"), (float, int)) or not math.isfinite(end["dt"]) or end["dt"] <= 0:
        raise BenchmarkError("invalid verification timestep")
    previous_events = 0
    for i, step in enumerate(steps):
        if step.get("kind") != "native-step" or step.get("step") != i:
            raise BenchmarkError("verification steps are missing, duplicated or reordered")
        for key in ("implementation", "case", "scale", "verification_contract"):
            if step.get(key) != end.get(key):
                raise BenchmarkError(f"verification metadata differs: {key}")
        if any(not isinstance(step.get(key), str) or not re.fullmatch(r"[0-9a-f]{16}", step[key])
               for key in HASH_FIELDS):
            raise BenchmarkError("invalid verification digest")
        events = step.get("event_count")
        if type(events) is not int or events < previous_events:
            raise BenchmarkError("invalid verification event count")
        previous_events = events
        if type(step.get("objects")) is not int or step["objects"] < 0:
            raise BenchmarkError("invalid verification object count")
    if steps[-1]["trajectory_hash"] != end.get("trajectory_hash"):
        raise BenchmarkError("terminal trajectory digest differs")
    return steps, end


def compare(left: list[dict], right: list[dict], *, cross_engine: bool = False) -> dict:
    a, a_end = validate(left)
    b, b_end = validate(right)
    for key in ("case", "scale", "steps_per_world", "dt", "verification_contract", "contact_height"):
        if a_end.get(key) != b_end.get(key):
            raise BenchmarkError(f"verification workloads differ: {key}")
    fields = ("objects", "object_hash", "event_hash", "event_count", "statistics_hash")
    # Internal constraint representations differ between ports. Cross-engine
    # comparison covers their resulting object states and public event stream.
    if not cross_engine:
        fields += ("constraint_hash", "trajectory_hash")
    for x, y in zip(a, b):
        for field in fields:
            if x[field] != y[field]:
                return {"passed": False, "step": x["step"], "field": field,
                        "left": x[field], "right": y[field]}
    return {"passed": True, "steps": len(a), "fields": list(fields)}
