#!/usr/bin/env python3
import argparse
import csv
import glob
import math
import os
import re
from bisect import bisect_right

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


TOPIC_EE_STATE = "/cartesian_velocity_controller_sim/ee_state"
TOPIC_WRENCH_EE = "/wrench_input"
TOPIC_WRENCH_RAW = "/ekf_wrench"
TOPIC_BEHAVIOR_LOG = "/rosout"

BEHAVIOR_PATTERN = re.compile(r"Triggered behavior:\s*(\S+)")


def bag_timestamp(msg_t):
    return msg_t.to_sec()


def quat_to_rot_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n

    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s

    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=float,
    )


def quat_to_euler_xyz(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


def wrap_to_0_2pi(values):
    values = np.asarray(values, dtype=float)
    return np.where(values < 0.0, values + 2.0 * np.pi, values)


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def parse_bag(bag_path):
    ee_rows = []
    wrench_ee_rows = []
    wrench_raw_rows = []
    behavior_rows = []

    ee_t = []
    ee_quat = []

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(
            topics=[TOPIC_EE_STATE, TOPIC_WRENCH_EE, TOPIC_WRENCH_RAW, TOPIC_BEHAVIOR_LOG]
        ):
            ts = bag_timestamp(t)

            if topic == TOPIC_EE_STATE:
                px = msg.pose.position.x
                py = msg.pose.position.y
                pz = msg.pose.position.z
                qx = msg.pose.orientation.x
                qy = msg.pose.orientation.y
                qz = msg.pose.orientation.z
                qw = msg.pose.orientation.w
                vx = msg.twist.linear.x
                vy = msg.twist.linear.y
                vz = msg.twist.linear.z
                wx = msg.twist.angular.x
                wy = msg.twist.angular.y
                wz = msg.twist.angular.z
                speed_linear = math.sqrt(vx * vx + vy * vy + vz * vz)
                speed_angular = math.sqrt(wx * wx + wy * wy + wz * wz)
                roll, pitch, yaw = quat_to_euler_xyz(qx, qy, qz, qw)

                ee_rows.append(
                    [
                        ts,
                        px,
                        py,
                        pz,
                        qx,
                        qy,
                        qz,
                        qw,
                        roll,
                        pitch,
                        yaw,
                        vx,
                        vy,
                        vz,
                        wx,
                        wy,
                        wz,
                        speed_linear,
                        speed_angular,
                    ]
                )
                ee_t.append(ts)
                ee_quat.append((qx, qy, qz, qw))

            elif topic == TOPIC_WRENCH_EE:
                fx = msg.wrench.force.x
                fy = msg.wrench.force.y
                fz = msg.wrench.force.z
                tx = msg.wrench.torque.x
                ty = msg.wrench.torque.y
                tz = msg.wrench.torque.z
                fn = math.sqrt(fx * fx + fy * fy + fz * fz)
                tn = math.sqrt(tx * tx + ty * ty + tz * tz)
                wrench_ee_rows.append([ts, fx, fy, fz, tx, ty, tz, fn, tn])

            elif topic == TOPIC_WRENCH_RAW:
                fx = msg.wrench.force.x
                fy = msg.wrench.force.y
                fz = msg.wrench.force.z
                tx = msg.wrench.torque.x
                ty = msg.wrench.torque.y
                tz = msg.wrench.torque.z
                fn = math.sqrt(fx * fx + fy * fy + fz * fz)
                tn = math.sqrt(tx * tx + ty * ty + tz * tz)
                wrench_raw_rows.append([ts, fx, fy, fz, tx, ty, tz, fn, tn])

            elif topic == TOPIC_BEHAVIOR_LOG:
                text = msg.msg
                m = BEHAVIOR_PATTERN.search(text)
                if m:
                    behavior_name = m.group(1)
                    behavior_rows.append([ts, behavior_name, msg.name, msg.level])

    # Convert wrench in EE frame to estimated base frame using nearest previous ee_state quaternion
    wrench_base_rows = []
    if ee_t:
        for row in wrench_ee_rows:
            ts = row[0]
            idx = bisect_right(ee_t, ts) - 1
            if idx < 0:
                continue
            qx, qy, qz, qw = ee_quat[idx]
            R = quat_to_rot_matrix(qx, qy, qz, qw)

            f_ee = np.array(row[1:4], dtype=float)
            t_ee = np.array(row[4:7], dtype=float)
            f_base = R.dot(f_ee)
            t_base = R.dot(t_ee)
            fn = np.linalg.norm(f_base)
            tn = np.linalg.norm(t_base)
            wrench_base_rows.append(
                [ts, f_base[0], f_base[1], f_base[2], t_base[0], t_base[1], t_base[2], fn, tn]
            )

    return ee_rows, wrench_ee_rows, wrench_base_rows, wrench_raw_rows, behavior_rows


def plot_outputs(output_dir, bag_name, ee_rows, wrench_ee_rows, wrench_base_rows, behavior_rows):
    if not ee_rows:
        return

    ee = np.array(ee_rows, dtype=float)
    t0 = ee[0, 0]
    t = ee[:, 0] - t0
    roll_plot = wrap_to_0_2pi(ee[:, 8])
    yaw_plot = wrap_to_0_2pi(ee[:, 10])

    behavior_times = np.array([r[0] - t0 for r in behavior_rows], dtype=float) if behavior_rows else np.array([])
    behavior_labels = [r[1] for r in behavior_rows]
    unique_labels = list(dict.fromkeys(behavior_labels))
    colors = plt.cm.tab10(np.linspace(0.0, 1.0, max(len(unique_labels), 1)))
    color_map = {label: colors[i] for i, label in enumerate(unique_labels)}

    fig, axes = plt.subplots(4, 1, figsize=(16, 12), sharex=True)
    fig.suptitle(f"{bag_name}: EE State / Velocity / Force Norms / Behavior")

    axes[0].plot(t, ee[:, 1], label="x")
    axes[0].plot(t, ee[:, 2], label="y")
    axes[0].plot(t, ee[:, 3], label="z")
    axes[0].set_ylabel("Position [m]")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, roll_plot, label="roll (0..2pi)")
    axes[1].plot(t, yaw_plot, label="yaw (0..2pi)")
    axes[1].set_ylabel("Euler [rad]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, ee[:, 17], label="linear_speed")
    axes[2].plot(t, ee[:, 18], label="angular_speed")
    axes[2].set_ylabel("Speed")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.3)

    if wrench_ee_rows:
        wr_ee = np.array(wrench_ee_rows, dtype=float)
        tw = wr_ee[:, 0] - t0
        axes[3].plot(tw, wr_ee[:, 7], label="|F| EE")
    axes[3].set_ylabel("Force norm [N]")
    axes[3].set_xlabel("Time from bag start [s]")
    axes[3].legend(loc="upper right")
    axes[3].grid(True, alpha=0.3)

    if behavior_times.size > 0:
        for axis in axes:
            for bt, lbl in zip(behavior_times, behavior_labels):
                color = color_map[lbl]
                axis.axvspan(bt - 0.06, bt + 0.06, color=color, alpha=0.18)
                axis.axvline(bt, color=color, alpha=0.85, linewidth=1.8)

        y_min, y_max = axes[0].get_ylim()
        text_y = y_max - 0.06 * (y_max - y_min if y_max > y_min else 1.0)
        for bt, lbl in zip(behavior_times, behavior_labels):
            axes[0].text(
                bt,
                text_y,
                lbl,
                rotation=90,
                va="top",
                ha="center",
                fontsize=8,
                color=color_map[lbl],
                bbox={"facecolor": "white", "edgecolor": color_map[lbl], "alpha": 0.8, "boxstyle": "round,pad=0.18"},
            )

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(os.path.join(output_dir, "overview.png"), dpi=150)
    plt.close(fig)

    if wrench_ee_rows and wrench_base_rows:
        wr_ee = np.array(wrench_ee_rows, dtype=float)
        wr_b = np.array(wrench_base_rows, dtype=float)
        tw_ee = wr_ee[:, 0] - t0
        tw_b = wr_b[:, 0] - t0

        fig2, axes2 = plt.subplots(2, 1, figsize=(16, 9), sharex=True)
        fig2.suptitle(f"{bag_name}: Wrench Components (EE vs Base estimated)")

        axes2[0].plot(tw_ee, wr_ee[:, 1], label="Fx_ee")
        axes2[0].plot(tw_ee, wr_ee[:, 2], label="Fy_ee")
        axes2[0].plot(tw_ee, wr_ee[:, 3], label="Fz_ee")
        axes2[0].plot(tw_b, wr_b[:, 1], "--", label="Fx_base")
        axes2[0].plot(tw_b, wr_b[:, 2], "--", label="Fy_base")
        axes2[0].plot(tw_b, wr_b[:, 3], "--", label="Fz_base")
        axes2[0].set_ylabel("Force [N]")
        axes2[0].legend(loc="upper right", ncol=3)
        axes2[0].grid(True, alpha=0.3)

        axes2[1].plot(tw_ee, wr_ee[:, 4], label="Tx_ee")
        axes2[1].plot(tw_ee, wr_ee[:, 5], label="Ty_ee")
        axes2[1].plot(tw_ee, wr_ee[:, 6], label="Tz_ee")
        axes2[1].plot(tw_b, wr_b[:, 4], "--", label="Tx_base")
        axes2[1].plot(tw_b, wr_b[:, 5], "--", label="Ty_base")
        axes2[1].plot(tw_b, wr_b[:, 6], "--", label="Tz_base")
        axes2[1].set_ylabel("Torque [Nm]")
        axes2[1].set_xlabel("Time from bag start [s]")
        axes2[1].legend(loc="upper right", ncol=3)
        axes2[1].grid(True, alpha=0.3)

        if behavior_times.size > 0:
            for axis in axes2:
                for bt, lbl in zip(behavior_times, behavior_labels):
                    color = color_map[lbl]
                    axis.axvspan(bt - 0.06, bt + 0.06, color=color, alpha=0.18)
                    axis.axvline(bt, color=color, alpha=0.85, linewidth=1.8)

            y2_min, y2_max = axes2[0].get_ylim()
            text_y2 = y2_max - 0.06 * (y2_max - y2_min if y2_max > y2_min else 1.0)
            for bt, lbl in zip(behavior_times, behavior_labels):
                axes2[0].text(
                    bt,
                    text_y2,
                    lbl,
                    rotation=90,
                    va="top",
                    ha="center",
                    fontsize=8,
                    color=color_map[lbl],
                    bbox={"facecolor": "white", "edgecolor": color_map[lbl], "alpha": 0.8, "boxstyle": "round,pad=0.18"},
                )

        fig2.tight_layout(rect=[0, 0, 1, 0.97])
        fig2.savefig(os.path.join(output_dir, "wrench_components.png"), dpi=150)
        plt.close(fig2)


def process_bag(bag_path):
    bag_name = os.path.basename(bag_path)
    bag_stem = os.path.splitext(bag_name)[0]
    output_dir = os.path.join(os.path.dirname(bag_path), f"{bag_stem}_analysis")
    ensure_dir(output_dir)

    ee_rows, wrench_ee_rows, wrench_base_rows, wrench_raw_rows, behavior_rows = parse_bag(bag_path)

    write_csv(
        os.path.join(output_dir, "ee_state.csv"),
        [
            "t_sec",
            "pos_x",
            "pos_y",
            "pos_z",
            "quat_x",
            "quat_y",
            "quat_z",
            "quat_w",
            "roll",
            "pitch",
            "yaw",
            "vel_x",
            "vel_y",
            "vel_z",
            "omega_x",
            "omega_y",
            "omega_z",
            "linear_speed",
            "angular_speed",
        ],
        ee_rows,
    )

    write_csv(
        os.path.join(output_dir, "wrench_ee.csv"),
        ["t_sec", "fx", "fy", "fz", "tx", "ty", "tz", "force_norm", "torque_norm"],
        wrench_ee_rows,
    )

    write_csv(
        os.path.join(output_dir, "wrench_base_estimated.csv"),
        ["t_sec", "fx", "fy", "fz", "tx", "ty", "tz", "force_norm", "torque_norm"],
        wrench_base_rows,
    )

    write_csv(
        os.path.join(output_dir, "wrench_raw_ekf.csv"),
        ["t_sec", "fx", "fy", "fz", "tx", "ty", "tz", "force_norm", "torque_norm"],
        wrench_raw_rows,
    )

    write_csv(
        os.path.join(output_dir, "behavior_events.csv"),
        ["t_sec", "behavior_name", "rosout_node", "rosout_level"],
        behavior_rows,
    )

    plot_outputs(output_dir, bag_name, ee_rows, wrench_ee_rows, wrench_base_rows, behavior_rows)

    return {
        "bag": bag_path,
        "out": output_dir,
        "ee_samples": len(ee_rows),
        "wrench_samples": len(wrench_ee_rows),
        "behavior_events": len(behavior_rows),
    }


def collect_bags(inputs):
    bag_files = []
    for p in inputs:
        if os.path.isdir(p):
            bag_files.extend(sorted(glob.glob(os.path.join(p, "*.bag"))))
        elif os.path.isfile(p) and p.endswith(".bag"):
            bag_files.append(p)
        else:
            bag_files.extend(sorted(glob.glob(p)))
    # stable dedup
    seen = set()
    unique = []
    for b in bag_files:
        ab = os.path.abspath(b)
        if ab not in seen:
            seen.add(ab)
            unique.append(ab)
    return unique


def main():
    parser = argparse.ArgumentParser(
        description="Convert ROS bag files to CSV and generate plots for EE state/wrench/behaviors."
    )
    parser.add_argument(
        "inputs",
        nargs="*",
        default=[
            "rosbag_data/no5",
        ],
        help="Bag files, directories, or glob patterns.",
    )
    args = parser.parse_args()

    bag_files = collect_bags(args.inputs)
    if not bag_files:
        print("No bag files found.")
        return 1

    print(f"Found {len(bag_files)} bag files.")
    summaries = []
    for i, bag_path in enumerate(bag_files, 1):
        print(f"[{i}/{len(bag_files)}] Processing: {bag_path}")
        try:
            s = process_bag(bag_path)
            summaries.append(s)
        except Exception as e:
            print(f"  ERROR: {e}")

    print("\nDone.")
    for s in summaries:
        print(
            f"- {os.path.basename(s['bag'])}: ee={s['ee_samples']}, "
            f"wrench={s['wrench_samples']}, behavior_events={s['behavior_events']} "
            f"-> {s['out']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())