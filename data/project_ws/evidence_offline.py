#!/usr/bin/env python3
"""Offline verification of the recalibrated stereo yaml (no sim required).

Checks that left_sim.yaml / right_sim.yaml are internally consistent and that
their intrinsics match the camera the simulator actually renders with (the
<lens><intrinsics> block in the xacro). Prints a report and writes JSON.
"""
import os
import json

import numpy as np
import yaml

CALIB_DIR = os.environ.get("CALIB_DIR", "/root/data/project_ws/calibration_sim")
OUT = os.environ.get("EVIDENCE_OUT", "/root/data/project_ws/calibration_sim/evidence")
os.makedirs(OUT, exist_ok=True)

# Intended intrinsics from the xacro <lens><intrinsics> (both lenses identical).
INT = dict(fx=253.090204, fy=253.090204, cx=358.540828, cy=194.131092)
PHYS_BASELINE = 2 * 0.03014  # 0.06028 m, from xacro camera joint origins.


def block(doc, key):
    b = doc[key]
    return np.array(b["data"], dtype=float).reshape(b["rows"], b["cols"])


def load(side):
    with open(os.path.join(CALIB_DIR, f"{side}_sim.yaml")) as f:
        d = yaml.safe_load(f)
    return dict(
        K=block(d, "camera_matrix"),
        D=np.array(d["distortion_coefficients"]["data"], dtype=float),
        R=block(d, "rectification_matrix"),
        P=block(d, "projection_matrix"),
        w=int(d["image_width"]),
        h=int(d["image_height"]),
    )


def pct(a, b):
    return (a - b) / b * 100.0


def ang_deg(a, b):
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    c = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))


L = load("left")
R = load("right")
report = {}

print("=== 1. Intrinsics vs intended (xacro <lens><intrinsics>) ===")
report["intrinsics"] = {}
for side, C in (("left", L), ("right", R)):
    fx, fy, cx, cy = C["K"][0, 0], C["K"][1, 1], C["K"][0, 2], C["K"][1, 2]
    report["intrinsics"][side] = {
        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
        "fx_err_pct": pct(fx, INT["fx"]), "fy_err_pct": pct(fy, INT["fy"]),
        "cx_err_pct": pct(cx, INT["cx"]), "cy_err_pct": pct(cy, INT["cy"]),
    }
    print(f"  {side:5s}: fx={fx:8.3f} ({pct(fx, INT['fx']):+5.2f}%)  "
          f"fy={fy:8.3f} ({pct(fy, INT['fy']):+5.2f}%)  "
          f"cx={cx:7.2f} ({pct(cx, INT['cx']):+5.2f}%)  "
          f"cy={cy:7.2f} ({pct(cy, INT['cy']):+5.2f}%)")
    print(f"        D = {np.array2string(C['D'], precision=6, suppress_small=True)}")

print("\n=== 2. Rectification matrices are proper rotations ===")
report["rotations"] = {}
for side, C in (("left", L), ("right", R)):
    Rm = C["R"]
    ortho = float(np.linalg.norm(Rm @ Rm.T - np.eye(3)))
    det = float(np.linalg.det(Rm))
    report["rotations"][side] = {"ortho_err": ortho, "det": det}
    print(f"  {side:5s}: ||R R^T - I|| = {ortho:.2e}   det(R) = {det:.6f}")

print("\n=== 3. Rectified epipolar axes are parallel (horizontal epipolar lines) ===")
# After rectification the two rectified cameras must share parallel x- and y-axes;
# that is exactly the condition for epipolar lines to be horizontal & aligned.
Ml = L["R"] @ L["K"]
Mr = R["R"] @ R["K"]
a0 = ang_deg(Ml[0], Mr[0])
a1 = ang_deg(Ml[1], Mr[1])
report["epipolar_axis_angle_deg"] = {"row0": a0, "row1": a1}
print(f"  angle(rectified x-axes) = {a0:.4f} deg")
print(f"  angle(rectified y-axes) = {a1:.4f} deg")
print(f"  (both ~0 => epipolar lines are horizontal in both rectified views)")

print("\n=== 4. Projection matrices (shared rectified intrinsics, zero disparity) ===")
Pl, Pr = L["P"], R["P"]
print(f"  P1 = \n{np.array2string(Pl, precision=4, suppress_small=True)}")
print(f"  P2 = \n{np.array2string(Pr, precision=4, suppress_small=True)}")
shared_fx = bool(np.isclose(Pl[0, 0], Pr[0, 0], atol=1e-3))
shared_cx = bool(np.isclose(Pl[0, 2], Pr[0, 2], atol=1e-3))
shared_cy = bool(np.isclose(Pl[1, 2], Pr[1, 2], atol=1e-3))
zero_disp = bool(np.isclose(Pl[2, 3], 0, atol=1e-3) and np.isclose(Pr[2, 3], 0, atol=1e-3))
calib_base = float(abs(Pr[0, 3]) / Pr[0, 0])
report["projection"] = {
    "shared_rectified_fx": shared_fx,
    "shared_cx": shared_cx,
    "shared_cy": shared_cy,
    "zero_disparity": zero_disp,
    "calibrated_baseline_m": calib_base,
    "physical_baseline_m": PHYS_BASELINE,
    "baseline_scale": calib_base / PHYS_BASELINE,
}
print(f"  shared rectified fx : {Pl[0,0]:.4f} == {Pr[0,0]:.4f}  -> {shared_fx}")
print(f"  shared cx           : {Pl[0,2]:.4f} == {Pr[0,2]:.4f}  -> {shared_cx}")
print(f"  shared cy           : {Pl[1,2]:.4f} == {Pr[1,2]:.4f}  -> {shared_cy}")
print(f"  zero disparity (P[2,3]=0) -> {zero_disp}")
print(f"  calibrated baseline = |P2[0,3]|/fx = {calib_base:.5f} m")
print(f"  physical  baseline  (2*0.03014)   = {PHYS_BASELINE:.5f} m")
print(f"  baseline scale (calibrated/physical) = {calib_base/PHYS_BASELINE:.4f}")

with open(os.path.join(OUT, "evidence_offline.json"), "w") as f:
    json.dump(report, f, indent=2)
print(f"\n[offline] wrote {os.path.join(OUT, 'evidence_offline.json')}")
