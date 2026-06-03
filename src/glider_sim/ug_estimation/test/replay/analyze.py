#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 1 离线评估：对比 ground_truth / robot_localization / ug_eskf。

输入一个 bag（通常由 run_phase1_eval.sh 回放录制而成），其中含:
  - /ug_glider/ground_truth/pose         nav_msgs/Odometry  (ENU, 真值)
  - /ug_glider/odometry/filtered_eskf    nav_msgs/Odometry  (ENU, 自研 ESKF)
  - /ug_glider/odometry/filtered         nav_msgs/Odometry  (ENU, robot_localization, 可选)
  - /ug_glider/glider_ekf/diagnostics    ug_msgs/EskfDiagnostics (可选)

输出: depth/roll/pitch RMSE (ENU→NED 后比较)、NIS 合规率、reject 次数。

用法:
  python3 analyze.py <bag> [--ns ug_glider] [--plot out.png]
"""

import argparse
import math
import sys

import numpy as np
import rosbag


def quat_to_matrix(qx, qy, qz, qw):
    """单位四元数 → 3x3 旋转矩阵 (body→world)。"""
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),     2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ])


def enu_quat_to_ned_rpy(qx, qy, qz, qw):
    """
    复刻 state_converter_node 的 ENU(body→world) → NED RPY 变换:
        R_ned = R_w^n · R_b^w · T_b
        R_w^n = [[0,1,0],[1,0,0],[0,0,-1]]
        T_b   = diag(1,-1,-1)
    返回 (roll, pitch, yaw) [rad]，ZYX 顺序。
    """
    R_bw = quat_to_matrix(qx, qy, qz, qw)
    R_wn = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    T_b = np.diag([1.0, -1.0, -1.0])
    R = R_wn @ R_bw @ T_b
    # ZYX 提取 (与 tf2 Matrix3x3::getRPY 一致)
    # pitch = asin(-R[2,0])
    sp = -R[2, 0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(sp) < 0.99999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # gimbal lock
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def read_odom(bag, topic):
    """返回 dict: t -> (depth_ned, roll, pitch, yaw)。depth_ned = -z_enu。"""
    out = []
    for _, msg, _ in bag.read_messages(topics=[topic]):
        t = msg.header.stamp.to_sec()
        z_enu = msg.pose.pose.position.z
        depth = -z_enu
        o = msg.pose.pose.orientation
        roll, pitch, yaw = enu_quat_to_ned_rpy(o.x, o.y, o.z, o.w)
        out.append((t, depth, roll, pitch, yaw))
    return out


def read_diag(bag, topic):
    nis, rej, acc, reset = [], 0, 0, 0
    last = None
    for _, msg, _ in bag.read_messages(topics=[topic]):
        nis.append(msg.nis_depth)
        last = msg
    if last is not None:
        rej, acc, reset = last.reject_depth, last.accept_depth, last.reset_count
    return np.array(nis), acc, rej, reset


def nearest_align(ref, est):
    """对 est 的每个点找 ref 中最近时间戳，返回对齐后的 (ref_arr, est_arr)。"""
    if not ref or not est:
        return None, None
    ref_t = np.array([r[0] for r in ref])
    ref_v = np.array([r[1:] for r in ref])
    pairs_ref, pairs_est = [], []
    for e in est:
        idx = int(np.argmin(np.abs(ref_t - e[0])))
        if abs(ref_t[idx] - e[0]) < 0.1:  # 100ms 容差
            pairs_ref.append(ref_v[idx])
            pairs_est.append(e[1:])
    if not pairs_ref:
        return None, None
    return np.array(pairs_ref), np.array(pairs_est)


def rmse(a, b):
    d = a - b
    return float(np.sqrt(np.mean(d * d)))


def wrap_err(a, b):
    """角度差，wrap 到 [-pi, pi]。"""
    d = a - b
    return np.arctan2(np.sin(d), np.cos(d))


def eval_estimator(name, ref, est, warmup_s=5.0):
    ref_a, est_a = nearest_align(ref, est)
    if ref_a is None:
        print(f"[{name}] 无可对齐数据，跳过")
        return
    # 跳过前 warmup（用相对索引近似：去掉前 N 个）
    n_skip = 0
    if est:
        t0 = est[0][0]
        for e in est:
            if e[0] - t0 < warmup_s:
                n_skip += 1
            else:
                break
    n_skip = min(n_skip, len(ref_a) - 1)
    ref_a, est_a = ref_a[n_skip:], est_a[n_skip:]

    depth_rmse = rmse(ref_a[:, 0], est_a[:, 0])
    roll_rmse = float(np.sqrt(np.mean(wrap_err(est_a[:, 1], ref_a[:, 1])**2)))
    pitch_rmse = float(np.sqrt(np.mean(wrap_err(est_a[:, 2], ref_a[:, 2])**2)))
    print(f"[{name}] N={len(ref_a)} (skip {n_skip})")
    print(f"    depth RMSE = {depth_rmse:.4f} m      "
          f"({'PASS' if depth_rmse <= 0.05 else 'FAIL'} ≤0.05)")
    print(f"    roll  RMSE = {math.degrees(roll_rmse):.4f}°    "
          f"({'PASS' if math.degrees(roll_rmse) <= 1.0 else 'FAIL'} ≤1°)")
    print(f"    pitch RMSE = {math.degrees(pitch_rmse):.4f}°    "
          f"({'PASS' if math.degrees(pitch_rmse) <= 1.0 else 'FAIL'} ≤1°)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--ns", default="ug_glider")
    ap.add_argument("--plot", default=None)
    args = ap.parse_args()

    ns = args.ns
    gt_topic = f"/{ns}/ground_truth/pose"
    eskf_topic = f"/{ns}/odometry/filtered_eskf"
    rl_topic = f"/{ns}/odometry/filtered"
    diag_topic = f"/{ns}/glider_ekf/diagnostics"

    bag = rosbag.Bag(args.bag)
    types_topics = bag.get_type_and_topic_info().topics
    print(f"=== bag: {args.bag} ===")
    print("topics:", ", ".join(sorted(types_topics.keys())))

    gt = read_odom(bag, gt_topic)
    if not gt:
        print(f"!! 未找到真值话题 {gt_topic}，无法评估")
        bag.close()
        sys.exit(1)

    if eskf_topic in types_topics:
        eval_estimator("ug_eskf", gt, read_odom(bag, eskf_topic))
    if rl_topic in types_topics:
        eval_estimator("robot_localization", gt, read_odom(bag, rl_topic))

    if diag_topic in types_topics:
        nis, acc, rej, reset = read_diag(bag, diag_topic)
        if len(nis):
            in_env = float(np.mean(nis < 3.84))   # χ²(0.95,1)
            print(f"[diagnostics] NIS_depth: N={len(nis)} "
                  f"mean={np.mean(nis):.3f} "
                  f"95%包络合规率={in_env*100:.1f}% "
                  f"({'PASS' if in_env >= 0.90 else 'FAIL'} ≥90%)")
            print(f"    accept={acc} reject={rej} reset={reset}")

    bag.close()


if __name__ == "__main__":
    main()
