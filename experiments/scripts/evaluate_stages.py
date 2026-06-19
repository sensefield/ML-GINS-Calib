#!/usr/bin/env python3
"""Per-stage absolute (base_link -> LiDAR) error report.

Reads the calibrator console log and the top-relative extrinsic output, then
compares each pipeline stage against the base_link GT:

    Stage1 (Coarse)  : 1st 4x4 matrix in each "########## <lidar> ##########"
                       section of calibrator_output.txt
    Stage1+2 (Joint) : 2nd 4x4 matrix in that section
    Multi-LiDAR      : T_top(Stage1+2) composed with the top-relative front/back
                       from extrinsic_parameters.txt
                       (top is the anchor, so its Multi result == Stage1+2)

Errors: rotation = angle of R_est * R_gt^T; dRx/dRy/dRz = SO(3)-log components
(deg); translation = ||t_est - t_gt|| with dTx/dTy/dTz components (m).

Usage:
    python3 experiments/scripts/evaluate_stages.py
    python3 experiments/scripts/evaluate_stages.py --log <path> --ext <path> --gt <path>
"""
import argparse
import numpy as np
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
LIDARS = ["top", "front", "back"]


def xyzq_to_T(row):
    tx, ty, tz, qx, qy, qz, qw = [float(v) for v in row]
    n = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    R = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


def parse_stages(text):
    """Return {lidar: {'coarse': T, 'joint': T}} from the per-LiDAR sections."""
    out = {}
    for lidar in LIDARS:
        marker = f"########## {lidar} ##########"
        idx = text.index(marker)
        end = text.find("##########", idx + len(marker))
        if end == -1:
            end = text.find("Ground Alignment", idx)
        section = text[idx: end if end != -1 else None]
        lines = section.splitlines()
        mats, i = [], 0
        while i < len(lines):
            try:
                rows = []
                for j in range(4):
                    parts = lines[i + j].split()
                    if len(parts) != 4:
                        raise ValueError
                    rows.append([float(p) for p in parts])
                mats.append(np.array(rows))
                i += 4
            except (ValueError, IndexError):
                i += 1
        assert len(mats) >= 2, f"Expected >=2 matrices for {lidar}, got {len(mats)}"
        out[lidar] = {"coarse": mats[0], "joint": mats[1]}
    return out


def parse_multi(stages, ext_path):
    rows = ext_path.read_text().strip().splitlines()
    T_rel = {l: xyzq_to_T(rows[i].split()) for i, l in enumerate(LIDARS)}
    T_abs_top = stages["top"]["joint"]
    return {
        "top": T_abs_top,                       # anchor == Stage1+2
        "front": T_abs_top @ T_rel["front"],
        "back": T_abs_top @ T_rel["back"],
    }


def rot_err(R_est, R_gt):
    cos = np.clip((np.trace(R_est @ R_gt.T) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos)))


def rot_components(R_est, R_gt):
    Rd = R_est @ R_gt.T
    cos = np.clip((np.trace(Rd) - 1.0) * 0.5, -1.0, 1.0)
    th = np.arccos(cos)
    if th < 1e-9:
        return 0.0, 0.0, 0.0
    s = 2.0 * np.sin(th)
    rx = (Rd[2, 1] - Rd[1, 2]) / s * th
    ry = (Rd[0, 2] - Rd[2, 0]) / s * th
    rz = (Rd[1, 0] - Rd[0, 1]) / s * th
    return tuple(float(np.degrees(v)) for v in (rx, ry, rz))


def print_row(lidar, stage, T_est, T_gt):
    rot = rot_err(T_est[:3, :3], T_gt[:3, :3])
    dt = T_est[:3, 3] - T_gt[:3, 3]
    trans = float(np.linalg.norm(dt))
    drx, dry, drz = rot_components(T_est[:3, :3], T_gt[:3, :3])
    print(f"{lidar:<6} {stage:<10} {rot:>8.3f} {trans:>9.4f}  "
          f"{drx:>+7.3f} {dry:>+7.3f} {drz:>+7.3f}  "
          f"{dt[0]:>+8.4f} {dt[1]:>+8.4f} {dt[2]:>+8.4f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default=REPO / "experiments/output/calibrator_output.txt", type=Path)
    ap.add_argument("--ext", default=REPO / "experiments/output/extrinsic_parameters.txt", type=Path)
    ap.add_argument("--gt", default=REPO / "experiments/gt/gt_extrinsic_abs_base_to_lidar.txt", type=Path)
    args = ap.parse_args()

    text = args.log.read_text()
    gt_T = {l: xyzq_to_T(line.split())
            for l, line in zip(LIDARS, args.gt.read_text().strip().splitlines())}
    stages = parse_stages(text)
    multi = parse_multi(stages, args.ext)

    hdr = (f"{'LiDAR':<6} {'Stage':<10} {'rot°':>8} {'trans m':>9}  "
           f"{'dRx°':>7} {'dRy°':>7} {'dRz°':>7}  "
           f"{'dTx m':>8} {'dTy m':>8} {'dTz m':>8}")
    print(hdr)
    print("-" * len(hdr))
    for lidar in LIDARS:
        print_row(lidar, "Stage1", stages[lidar]["coarse"], gt_T[lidar])
        print_row(lidar, "Stage1+2", stages[lidar]["joint"], gt_T[lidar])
        if lidar != "top":
            print_row(lidar, "Multi", multi[lidar], gt_T[lidar])
        print()


if __name__ == "__main__":
    main()
