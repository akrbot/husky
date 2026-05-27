#!/usr/bin/env python3

import os
import sys
import glob
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_csv(filepath):
    data = np.genfromtxt(filepath, delimiter=',', names=True)
    return data


def angle_wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def compute_metrics(data):
    gt_x, gt_y, gt_yaw = data['gt_x'], data['gt_y'], data['gt_yaw']

    valid = ~np.isnan(gt_x)

    methods = {
        'Odometry': ('odom_x', 'odom_y', 'odom_yaw'),
        'IMU DR': ('imu_x', 'imu_y', 'imu_yaw'),
        'EKF Local': ('ekf_x', 'ekf_y', 'ekf_yaw'),
        'EKF Global': ('ekf_global_x', 'ekf_global_y', 'ekf_global_yaw'),
    }

    results = {}
    for name, (xk, yk, ywk) in methods.items():
        if xk not in data.dtype.names:
            continue
        mx, my, myaw = data[xk], data[yk], data[ywk]
        v = valid & ~np.isnan(mx)

        if v.sum() == 0:
            results[name] = {
                'pos_rmse': float('nan'), 'yaw_rmse_deg': float('nan'),
                'max_pos_err': float('nan'), 'max_yaw_err_deg': float('nan'),
                'drift_rate': float('nan'),
            }
            continue

        pos_err = np.sqrt((mx[v] - gt_x[v])**2 + (my[v] - gt_y[v])**2)
        yaw_err = np.abs(angle_wrap(myaw[v] - gt_yaw[v]))

        dx = np.diff(gt_x[v])
        dy = np.diff(gt_y[v])
        seg_dist = np.sqrt(dx**2 + dy**2)
        cum_dist = np.concatenate([[0.0], np.cumsum(seg_dist)])

        drift_rate = float('nan')
        if cum_dist[-1] > 1.0:
            mask = cum_dist > 0.5
            if mask.sum() > 2:
                coeffs = np.polyfit(cum_dist[mask], pos_err[mask], 1)
                drift_rate = coeffs[0]

        results[name] = {
            'pos_rmse': float(np.sqrt(np.mean(pos_err**2))),
            'yaw_rmse_deg': float(np.degrees(np.sqrt(np.mean(yaw_err**2)))),
            'max_pos_err': float(np.max(pos_err)),
            'max_yaw_err_deg': float(np.degrees(np.max(yaw_err))),
            'drift_rate': drift_rate,
        }

    return results


def plot_trajectories(data, output_dir):
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))

    v_gt = ~np.isnan(data['gt_x'])
    ax.plot(data['gt_x'][v_gt], data['gt_y'][v_gt], 'k-', linewidth=2, label='Ground Truth')

    styles = [
        ('odom_x', 'odom_y', 'b-', 'Odometry'),
        ('imu_x', 'imu_y', 'r-', 'IMU DR'),
        ('ekf_x', 'ekf_y', 'g-', 'EKF Local'),
        ('ekf_global_x', 'ekf_global_y', 'm-', 'EKF Global'),
    ]
    for xk, yk, style, label in styles:
        if xk not in data.dtype.names:
            continue
        v = ~np.isnan(data[xk])
        if v.sum() > 0:
            ax.plot(data[xk][v], data[yk][v], style, linewidth=1, alpha=0.8, label=label)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Trajectory Comparison')
    ax.legend()
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'trajectory_comparison.png'), dpi=150)
    plt.close(fig)


def plot_position_error(data, output_dir):
    gt_x, gt_y = data['gt_x'], data['gt_y']
    t = data['timestamp']
    v_gt = ~np.isnan(gt_x)

    fig, ax = plt.subplots(1, 1, figsize=(10, 5))

    methods = [
        ('odom_x', 'odom_y', 'b', 'Odometry'),
        ('imu_x', 'imu_y', 'r', 'IMU DR'),
        ('ekf_x', 'ekf_y', 'g', 'EKF Local'),
        ('ekf_global_x', 'ekf_global_y', 'm', 'EKF Global'),
    ]
    for xk, yk, color, label in methods:
        if xk not in data.dtype.names:
            continue
        v = v_gt & ~np.isnan(data[xk])
        if v.sum() > 0:
            err = np.sqrt((data[xk][v] - gt_x[v])**2 + (data[yk][v] - gt_y[v])**2)
            ax.plot(t[v], err, color=color, linewidth=1, alpha=0.8, label=label)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Error (m)')
    ax.set_title('Position Error Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'position_error.png'), dpi=150)
    plt.close(fig)


def plot_heading_error(data, output_dir):
    gt_yaw = data['gt_yaw']
    t = data['timestamp']
    v_gt = ~np.isnan(gt_yaw)

    fig, ax = plt.subplots(1, 1, figsize=(10, 5))

    methods = [
        ('odom_yaw', 'b', 'Odometry'),
        ('imu_yaw', 'r', 'IMU DR'),
        ('ekf_yaw', 'g', 'EKF Local'),
        ('ekf_global_yaw', 'm', 'EKF Global'),
    ]
    for ywk, color, label in methods:
        if ywk not in data.dtype.names:
            continue
        v = v_gt & ~np.isnan(data[ywk])
        if v.sum() > 0:
            err = np.abs(angle_wrap(data[ywk][v] - gt_yaw[v]))
            ax.plot(t[v], np.degrees(err), color=color, linewidth=1, alpha=0.8, label=label)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Heading Error (deg)')
    ax.set_title('Heading Error Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'heading_error.png'), dpi=150)
    plt.close(fig)


def plot_summary_bar(results, output_dir):
    names = list(results.keys())
    pos_rmse = [results[n]['pos_rmse'] for n in names]
    yaw_rmse = [results[n]['yaw_rmse_deg'] for n in names]

    x = np.arange(len(names))
    width = 0.35

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    all_colors = ['#4285f4', '#ea4335', '#34a853', '#ab47bc']
    colors = all_colors[:len(names)]
    ax1.bar(x, pos_rmse, width, color=colors)
    ax1.set_ylabel('RMSE (m)')
    ax1.set_title('Position RMSE')
    ax1.set_xticks(x)
    ax1.set_xticklabels(names)
    ax1.grid(True, alpha=0.3, axis='y')
    for i, v in enumerate(pos_rmse):
        if not np.isnan(v):
            ax1.text(i, v + 0.01, '{:.3f}'.format(v), ha='center', fontsize=9)

    ax2.bar(x, yaw_rmse, width, color=colors)
    ax2.set_ylabel('RMSE (deg)')
    ax2.set_title('Heading RMSE')
    ax2.set_xticks(x)
    ax2.set_xticklabels(names)
    ax2.grid(True, alpha=0.3, axis='y')
    for i, v in enumerate(yaw_rmse):
        if not np.isnan(v):
            ax2.text(i, v + 0.1, '{:.2f}'.format(v), ha='center', fontsize=9)

    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'rmse_summary.png'), dpi=150)
    plt.close(fig)


def find_latest_csv(results_dir):
    csvs = sorted(glob.glob(os.path.join(results_dir, 'benchmark_*.csv')))
    if not csvs:
        return None
    return csvs[-1]


def main():
    parser = argparse.ArgumentParser(description='Analyze pose estimation benchmark results')
    parser.add_argument('--input', type=str, default=None, help='Path to benchmark CSV file')
    parser.add_argument('--output', type=str, default=None, help='Output directory for plots')
    args = parser.parse_args()

    if args.input is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        results_dir = os.path.join(script_dir, '..', 'results')
        args.input = find_latest_csv(results_dir)
        if args.input is None:
            print("ERROR: No benchmark CSV files found in {}".format(results_dir))
            sys.exit(1)

    if args.output is None:
        args.output = os.path.dirname(args.input)

    print("Loading: {}".format(args.input))
    data = load_csv(args.input)
    print("Loaded {} samples over {:.1f} seconds".format(len(data), data['timestamp'][-1]))

    results = compute_metrics(data)

    print("\n{:<12} {:>12} {:>14} {:>12} {:>16} {:>12}".format(
        'Method', 'Pos RMSE(m)', 'Yaw RMSE(deg)', 'Max Pos(m)', 'Max Yaw(deg)', 'Drift(m/m)'))
    print("-" * 82)
    for name, m in results.items():
        print("{:<12} {:>12.4f} {:>14.2f} {:>12.4f} {:>16.2f} {:>12.4f}".format(
            name, m['pos_rmse'], m['yaw_rmse_deg'], m['max_pos_err'],
            m['max_yaw_err_deg'], m['drift_rate']))

    plot_trajectories(data, args.output)
    plot_position_error(data, args.output)
    plot_heading_error(data, args.output)
    plot_summary_bar(results, args.output)

    print("\nPlots saved to: {}".format(args.output))


if __name__ == '__main__':
    main()
