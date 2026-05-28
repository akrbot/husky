#!/usr/bin/env python3
"""
generate_training_dataset.py
=============================
STAGE 2 — offline label generation and feature packaging.

Takes raw CSVs from training_data_collector_node.py and produces:

  X.npy          (N, 17)  float32   model input features  (Section A only)
  Y.npy          (N, 2)   float32   [w_imu, w_odom] soft reliability labels
  meta.csv       (N rows)           timestamp, fault_label, run_id, split
  splits.json                       train/val/test run IDs  (split BY RUN)
  feature_names.json                column index → feature name

How labels are computed
-----------------------
For each timestep i, look at the last WINDOW_SEC seconds of data:
  e_imu  = RMSE( imu_dr_position   vs  gt_position )  over window
  e_odom = RMSE( odom_position      vs  gt_position )  over window

  w_imu  = (1/e_imu)  / (1/e_imu + 1/e_odom)
  w_odom = (1/e_odom) / (1/e_imu + 1/e_odom)

So the sensor with lower position error gets a higher weight.
Both weights are in (0, 1) and sum to 1.

The model (X → Y) learns to predict these weights from raw sensor signals
alone — no ground truth at inference time.

Usage
-----
  # Single file:
  python3 generate_training_dataset.py --input results/training_clean_*.csv

  # Whole folder:
  python3 generate_training_dataset.py --input results/ --output dataset/

  # Change window:
  python3 generate_training_dataset.py --input results/ --window_sec 3.0
"""

import os, json, glob, argparse
from pathlib import Path
from collections import defaultdict

import numpy  as np
import pandas as pd


# ─────────────────────────────────────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────────────────────────────────────

SAMPLE_RATE = 50      # Hz — must match collector
WINDOW_SEC  = 2.0     # seconds of history for both features and labels
EPS         = 1e-3    # prevents division by zero in label softmax

SPLIT_FRACTIONS = dict(train=0.70, val=0.15, test=0.15)

# ── Section A column names (model input features) ──────────────────────────
# These are read directly from the CSV — no transformation needed.
FEATURE_COLS = [
    "imu_ax", "imu_ay", "imu_az",
    "imu_gx", "imu_gy", "imu_gz",
    "imu_cov",
    "odom_v", "odom_w",
    "odom_a", "odom_alpha",
    "diff_imu_odom_w",
    "diff_imu_odom_a",
    "diff_gps_odom_v",
    "gps_hdop",
    "gps_cov",
]
# Note: timestamp is NOT a feature.  It is used only for meta.

# ── Section B column names (label generation only) ─────────────────────────
LABEL_COLS_GT   = ["gt_x",       "gt_y"]
LABEL_COLS_IMU  = ["imu_dr_x",   "imu_dr_y"]
LABEL_COLS_ODOM = ["odom_pos_x", "odom_pos_y"]


# ─────────────────────────────────────────────────────────────────────────────
# Label computation
# ─────────────────────────────────────────────────────────────────────────────

def compute_labels(window: pd.DataFrame) -> np.ndarray:
    """
    Soft reliability weights [w_imu, w_odom] for one timestep.
    Uses the last WINDOW rows' position estimates vs ground truth.
    This function is the ONLY place ground truth is used.
    """
    gt   = window[LABEL_COLS_GT].values    # (W, 2)
    imu  = window[LABEL_COLS_IMU].values   # (W, 2)
    odom = window[LABEL_COLS_ODOM].values  # (W, 2)

    e_imu  = np.sqrt(np.mean(np.sum((imu  - gt) ** 2, axis=1)))
    e_odom = np.sqrt(np.mean(np.sum((odom - gt) ** 2, axis=1)))

    inv_imu  = 1.0 / (e_imu  + EPS)
    inv_odom = 1.0 / (e_odom + EPS)
    total    = inv_imu + inv_odom

    return np.array([inv_imu / total, inv_odom / total], dtype=np.float32)


# ─────────────────────────────────────────────────────────────────────────────
# Per-run processing
# ─────────────────────────────────────────────────────────────────────────────

REQUIRED_COLS = FEATURE_COLS + ["timestamp"] + \
                LABEL_COLS_GT + LABEL_COLS_IMU + LABEL_COLS_ODOM + ["fault_label"]


def process_run(csv_path: str, run_id: int, window: int):
    """
    Slide the window over one run's CSV.
    Returns (X_run, Y_run, meta_rows)  or  (None, None, None) on error.

      X_run  shape: (N, 16)   — Section A features, no timestamp
      Y_run  shape: (N, 2)    — [w_imu, w_odom]
    """
    fname = os.path.basename(csv_path)
    print(f"  run {run_id:02d}: {fname}")

    df = pd.read_csv(csv_path)

    missing = [c for c in REQUIRED_COLS if c not in df.columns]
    if missing:
        print(f"    SKIP — missing columns: {missing}")
        return None, None, None

    # Drop startup NaN rows (IMU DR takes a moment to initialise)
    df = df.dropna(subset=FEATURE_COLS + LABEL_COLS_IMU).reset_index(drop=True)

    if len(df) < window + 10:
        print(f"    SKIP — only {len(df)} rows after NaN drop (need ≥ {window+10})")
        return None, None, None

    fault_label = str(df["fault_label"].iloc[0])

    X_rows, Y_rows, meta_rows = [], [], []

    for i in range(window, len(df)):
        win = df.iloc[i - window : i]   # history window (exclusive of current row)

        # X: raw sensor features from the window's final row
        # (instantaneous reading, not aggregated — the window is used for labels)
        # If you want windowed statistics as features, extend extract_features().
        x_row = df.iloc[i][FEATURE_COLS].values.astype(np.float32)

        y_row = compute_labels(win)

        X_rows.append(x_row)
        Y_rows.append(y_row)
        meta_rows.append({
            "timestamp":   df.iloc[i]["timestamp"],
            "fault_label": fault_label,
            "run_id":      run_id,
            "source_file": fname,
        })

    X = np.array(X_rows, dtype=np.float32)
    Y = np.array(Y_rows, dtype=np.float32)

    w_imu_mean  = Y[:, 0].mean()
    w_odom_mean = Y[:, 1].mean()
    print(f"    → {len(X):5d} samples | fault={fault_label:20s} | "
          f"w_imu={w_imu_mean:.3f}  w_odom={w_odom_mean:.3f}")

    return X, Y, meta_rows


# ─────────────────────────────────────────────────────────────────────────────
# Train / val / test split — STRATIFIED BY RUN
# ─────────────────────────────────────────────────────────────────────────────

def split_runs(run_ids, fault_labels):
    """
    Ensures each fault type appears in all three splits.
    Never splits within a run.
    """
    by_fault = defaultdict(list)
    for rid, fl in zip(run_ids, fault_labels):
        by_fault[fl].append(rid)

    train_ids, val_ids, test_ids = [], [], []

    for fault, ids in sorted(by_fault.items()):
        rng = np.random.default_rng(42)
        ids = rng.permutation(ids).tolist()
        n      = len(ids)
        n_val  = max(1, round(n * SPLIT_FRACTIONS["val"]))
        n_test = max(1, round(n * SPLIT_FRACTIONS["test"]))
        n_train = n - n_val - n_test

        if n_train < 1:
            print(f"  WARNING: '{fault}' has only {n} run(s) → all to train.")
            train_ids += ids
        else:
            train_ids += ids[:n_train]
            val_ids   += ids[n_train : n_train + n_val]
            test_ids  += ids[n_train + n_val :]

    return {"train": sorted(train_ids),
            "val":   sorted(val_ids),
            "test":  sorted(test_ids)}


# ─────────────────────────────────────────────────────────────────────────────
# Sanity checks
# ─────────────────────────────────────────────────────────────────────────────

def sanity_check(Y, meta_df):
    print()
    print("── Label sanity check ─────────────────────────────────────────────")
    print(f"  {'Fault label':<24} {'w_imu mean':>12} {'w_odom mean':>12}  Expected")
    print(f"  {'─'*24} {'─'*12} {'─'*12}  {'─'*30}")

    expectations = {
        "clean":       "both ≈ 0.50",
        "imu_lowend":  "w_imu << 0.5, w_odom >> 0.5",
        "imu_highend": "both ≈ 0.50",
        "odom_slip":   "w_odom << 0.5, w_imu >> 0.5",
        "odom_bias":   "w_odom << 0.5, w_imu >> 0.5",
    }

    all_ok = True
    for fault in sorted(meta_df["fault_label"].unique()):
        mask  = meta_df["fault_label"] == fault
        y_sub = Y[mask.values]
        wi    = y_sub[:, 0].mean()
        wo    = y_sub[:, 1].mean()
        exp   = expectations.get(fault, "—")

        # Simple heuristic pass/fail
        ok = "✓"
        if "imu" in fault and "high" not in fault:
            if wi > 0.45:
                ok = "✗ w_imu should be lower"
                all_ok = False
        elif "odom" in fault:
            if wo > 0.45:
                ok = "✗ w_odom should be lower"
                all_ok = False

        print(f"  {fault:<24} {wi:>12.3f} {wo:>12.3f}  {ok}")

    print()
    if not all_ok:
        print("  ⚠ Some labels look wrong.  Before retraining:")
        print("    1. Run pose_benchmark_analyzer.py on the raw CSV.")
        print("    2. Verify the fault was actually degrading position accuracy.")
        print("    3. Try --window_sec 3.0 if faults build up slowly.")
    else:
        print("  Labels look correct.  Proceed to training.")
    print("────────────────────────────────────────────────────────────────────")
    print()


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input",      required=True,
                        help="CSV file or folder of training_*.csv files")
    parser.add_argument("--output",     default="dataset",
                        help="Output directory (default: ./dataset)")
    parser.add_argument("--window_sec", type=float, default=WINDOW_SEC,
                        help=f"Label window in seconds (default {WINDOW_SEC})")
    parser.add_argument("--seed",       type=int,   default=42)
    args = parser.parse_args()

    np.random.seed(args.seed)
    window = int(args.window_sec * SAMPLE_RATE)

    print(f"\nWindow: {args.window_sec}s = {window} samples")
    print(f"Features: {len(FEATURE_COLS)}  (Section A raw sensor signals only)")
    print(f"Labels: 2  [w_imu, w_odom]  (computed from Section B + ground truth)\n")

    # ── Gather CSV paths ──────────────────────────────────────────────────────
    inp = Path(args.input)
    if inp.is_file():
        csv_paths = [str(inp)]
    elif inp.is_dir():
        csv_paths = sorted(glob.glob(str(inp / "training_*.csv")))
    else:
        raise FileNotFoundError(args.input)

    if not csv_paths:
        raise RuntimeError(f"No training_*.csv found in {args.input}")

    print(f"Found {len(csv_paths)} file(s):")
    for p in csv_paths:
        print(f"  {p}")
    print()

    # ── Process runs ──────────────────────────────────────────────────────────
    all_X, all_Y, all_meta   = [], [], []
    run_ids, fault_per_run   = [], []

    for run_id, path in enumerate(csv_paths):
        Xr, Yr, meta_r = process_run(path, run_id, window)
        if Xr is None:
            continue
        all_X.append(Xr)
        all_Y.append(Yr)
        all_meta.extend(meta_r)
        run_ids.append(run_id)
        fault_per_run.append(meta_r[0]["fault_label"])

    if not all_X:
        raise RuntimeError("No valid runs processed.")

    X       = np.concatenate(all_X, axis=0)
    Y       = np.concatenate(all_Y, axis=0)
    meta_df = pd.DataFrame(all_meta)

    print(f"\nTotal:  {len(X)} samples  |  X shape {X.shape}  |  Y shape {Y.shape}")

    sanity_check(Y, meta_df)

    # ── Split ─────────────────────────────────────────────────────────────────
    splits = split_runs(run_ids, fault_per_run)

    run_to_split = {rid: sp
                    for sp, ids in splits.items()
                    for rid in ids}
    meta_df["split"] = meta_df["run_id"].map(run_to_split)

    print("Split breakdown:")
    for sp in ("train", "val", "test"):
        mask   = meta_df["split"] == sp
        counts = meta_df[mask]["fault_label"].value_counts().to_dict()
        print(f"  {sp:6s}: {mask.sum():6d} samples  {counts}")

    # ── Save ──────────────────────────────────────────────────────────────────
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)

    np.save(out / "X.npy", X)
    np.save(out / "Y.npy", Y)
    meta_df.to_csv(out / "meta.csv", index=False)

    with open(out / "splits.json", "w") as f:
        json.dump(splits, f, indent=2)

    with open(out / "feature_names.json", "w") as f:
        json.dump(FEATURE_NAMES := FEATURE_COLS, f, indent=2)

    print(f"\nSaved to {out}/")
    print(f"  X.npy            {X.shape}   ← {len(FEATURE_COLS)} raw sensor features")
    print(f"  Y.npy            {Y.shape}    ← [w_imu, w_odom]  soft labels")
    print(f"  meta.csv         {len(meta_df)} rows")
    print(f"  splits.json")
    print(f"  feature_names.json")
    print(f"\nNext:  python3 train_reliability_mlp.py --dataset {out}")


if __name__ == "__main__":
    main()