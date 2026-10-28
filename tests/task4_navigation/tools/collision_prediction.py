"""Swept-body collision audit for the restricted A-to-open-waypoint test leg.

The nominal path is fixed first. The vehicle footprint is swept continuously
along straight travel through the latest event window plus configured sensor,
control and braking travel. Wall endpoints receive shared-coordinate X/Y
perturbations so connected board joints do not acquire artificial gaps.
"""
import json
import math
import random
from pathlib import Path

import navigation_geometry as ng

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "records"


def scenario_offsets(vertices, rng, kind):
    if kind == "random":
        return {p: (rng.uniform(-50, 50), rng.uniform(-50, 50)) for p in vertices}
    offsets = {p: (0.0, 0.0) for p in vertices}
    # Directed corridor narrowing: the MID_H line and all its shared junctions
    # move down 50 mm; the start-side A_SIDE moves left 50 mm.
    if kind in ("mid_h_down", "combined_narrow"):
        for p in vertices:
            if abs(p[1] - 1200.0) < 1.0 and p[0] >= 1200.0:
                offsets[p] = (offsets[p][0], -50.0)
    if kind in ("a_side_left", "combined_narrow"):
        for p in vertices:
            if abs(p[0] - 2400.0) < 1.0 and p[1] <= 600.0:
                offsets[p] = (-50.0, offsets[p][1])
    if kind == "mid_h_tilt":
        for p in vertices:
            if abs(p[1] - 1200.0) < 1.0 and p[0] >= 1200.0:
                offsets[p] = (offsets[p][0], -50.0 if p[0] < 2100 else 50.0)
    return offsets


def main():
    OUT.mkdir(exist_ok=True)
    cfg = ng.C
    body_radius = math.hypot(320.0, 120.0)
    start_error = math.hypot(ng.M["assumptions"]["start_xy_error_mm"],
                             ng.M["assumptions"]["start_xy_error_mm"])
    yaw_corner = body_radius * math.sin(math.radians(2.0))
    fixed_inflation = start_error + yaw_corner + cfg["tof_error_mm"] + cfg["minimum_clearance_mm"]
    reaction_mm = cfg["initial_speed_mm_s"] * (cfg["range_age_ms"] + cfg["control_latency_ms"]) / 1000.0
    brake_mm = cfg["braking_distance_mm"]
    # Event may arrive at the upper distance-window bound. Travel after that
    # event is added once as geometry, not again as an uncertainty dilation.
    start = ng.M["assumptions"]["start_xy_mm"]
    target = (2700.0, 750.0)
    worst_stop_y = target[1] + (430.0 - 450.0) + reaction_mm + brake_mm
    # Route starts at y=300; upper event bound 430 mm is relative to A.
    worst_stop_y = start[1] + 430.0 + reaction_mm + brake_mm
    swept = ng.hull(ng.rect(start[0], start[1], 0) + ng.rect(target[0], worst_stop_y, 0))
    body = ng.expand(swept, fixed_inflation)

    vertices = sorted({tuple(w[k]) for w in ng.W for k in ("a", "b")})
    directed = ["nominal", "mid_h_down", "a_side_left", "combined_narrow", "mid_h_tilt"]
    rows = []
    worst = {"margin_mm": float("inf"), "scene": None, "wall": None}
    for kind in directed:
        off = {} if kind == "nominal" else scenario_offsets(vertices, random.Random(0), kind)
        ww = ng.walls(offsets=off) if off else ng.walls()
        ok, clearance, wall = ng.check_poly(body, ww, True)
        row = {"scene": kind, "passed": ok, "minimum_clearance_mm": round(clearance, 3), "limiting_wall": wall}
        rows.append(row)
        if clearance < worst["margin_mm"]:
            worst = {"margin_mm": clearance, "scene": kind, "wall": wall}

    rng = random.Random(cfg["random_seed"])
    random_passes = 0
    for i in range(cfg["random_scenarios"]):
        off = scenario_offsets(vertices, rng, "random")
        ww = ng.walls(offsets=off)
        ok, clearance, wall = ng.check_poly(body, ww, True)
        random_passes += int(ok)
        rows.append({"scene": i, "kind": "random_shared_endpoint_xy", "passed": ok,
                     "minimum_clearance_mm": round(clearance, 3), "limiting_wall": wall})
        if clearance < worst["margin_mm"]:
            worst = {"margin_mm": clearance, "scene": i, "wall": wall}

    summary = {
        "status": "PASS_SINGLE_LEG_GEOMETRY_ONLY" if random_passes == cfg["random_scenarios"] and all(x["passed"] for x in rows[:len(directed)]) else "FAIL_OR_UNPROVEN",
        "map_version": ng.M["map_version"],
        "leg": {"from": "A", "to": "OPEN_WAYPOINT_1", "start_mm": list(start), "target_mm": list(target),
                "upper_event_distance_mm": 430, "worst_stop_y_mm": round(worst_stop_y, 3),
                "speed_mm_s": cfg["initial_speed_mm_s"]},
        "body_mm": {"front": 320, "rear": 0, "half_width": 120},
        "wall_thickness_mm": 3,
        "endpoint_error_xy_mm": 50,
        "fixed_pose_sensor_clearance_inflation_mm": round(fixed_inflation, 3),
        "feedback_reaction_travel_mm": round(reaction_mm, 3),
        "braking_distance_mm_assumption": brake_mm,
        "directed_scene_count": len(directed),
        "directed_passes": sum(x["passed"] for x in rows[:len(directed)]),
        "random_seed": cfg["random_seed"],
        "random_scenarios": cfg["random_scenarios"],
        "random_passes": random_passes,
        "random_failures": cfg["random_scenarios"] - random_passes,
        "minimum_clearance_mm": round(worst["margin_mm"], 3),
        "minimum_clearance_scene": worst,
        "board_route_enable": False,
        "limitations": [
            "This result covers only A to OPEN_WAYPOINT_1, not A-B, A-C, A-D, A-E, or patrol.",
            "Endpoint, ToF, control-latency, tracking and braking values are conservative simulation assumptions; the dynamic values require board measurement.",
            "The map is image-derived and not a survey; this audit does not change the caller-supplied 3 m field, 600 mm boards, 3 mm thickness, or 320x240 mm vehicle outline.",
        ],
        "scenarios": rows,
    }
    (OUT / "collision_prediction_report.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in summary.items() if k != "scenarios"}, indent=2))


if __name__ == "__main__":
    main()
