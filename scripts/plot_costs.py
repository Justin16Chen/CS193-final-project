#!/usr/bin/env python3
"""Snapshot the planner's cost grid as a PNG (and a window) for tuning controller_params.yaml.

Workflow:
  1. run the sim and drive into a situation worth looking at
  2. pause Gazebo (the controller stops planning, so its last grid is the paused instant)
  3. from a terminal with the workspace sourced:
       python3 scripts/plot_costs.py
  4. edit predict_params in controller_params.yaml and run again with --reload:
       python3 scripts/plot_costs.py --reload
     This pushes predict_params.* from the yaml to the running controller, asks it
     to re-plan from the inputs of its last scan (the paused instant), and plots
     the new grid. The live planner uses the same parameters once you resume.

The controller publishes crowd_interfaces/CostGrid on /cost_grid with transient-local
QoS, depth 1, so a late-joining subscriber still receives the most recent grid.
Images go to crowd_sim/tuning/, named by sim time (plus wall-clock time for --reload runs).
"""
import argparse
import os
import sys
import time

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger
from crowd_interfaces.msg import CostGrid

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "crowd_sim", "tuning"))
DEFAULT_PARAMS = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "crowd_sim", "config", "controller_params.yaml"))
CONTROLLER = "/controller"
PARAM_PREFIX = "predict_params."

COMPONENTS = [
    ("total", "total"),
    ("dist", "dist_cost"),
    ("away_from_goal", "away_from_goal_cost"),
    ("diff_from_prev", "diff_from_prev_output_cost"),
    ("diff_from_preferred", "diff_from_preferred_speed_cost"),
]


GRID_QOS = QoSProfile(depth=1,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def load_predict_params(yaml_path):
    """predict_params.* from controller_params.yaml as rclpy Parameters."""
    with open(yaml_path) as f:
        doc = yaml.safe_load(f)
    try:
        section = doc["controller"]["ros__parameters"]["predict_params"]
    except (KeyError, TypeError):
        raise SystemExit(f"{yaml_path}: expected controller.ros__parameters.predict_params")
    return [Parameter(PARAM_PREFIX + k, Parameter.Type.DOUBLE, float(v)) for k, v in section.items()]


def call(node, client, request, what, timeout_s):
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise SystemExit(f"{what}: service {client.srv_name} not available. Is the controller running?")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done():
        raise SystemExit(f"{what}: no response within {timeout_s:.0f}s")
    return future.result()


def fetch_grid(timeout_s, reload_yaml=None):
    """Return the latest CostGrid, or None on timeout.

    With reload_yaml, first push predict_params.* to the controller and call its
    replan service, then wait for a grid published *after* that call. The topic is
    transient-local, so the previously cached grid arrives immediately on
    subscribing; it is drained before replanning so a stale grid is never plotted.
    """
    node = Node("plot_costs")
    got = []
    node.create_subscription(CostGrid, "cost_grid", got.append, GRID_QOS)
    deadline = time.monotonic() + timeout_s

    if reload_yaml:
        t_end = time.monotonic() + 0.5   # drain whatever is cached
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
        got.clear()

        params = load_predict_params(reload_yaml)
        set_cli = node.create_client(SetParameters, CONTROLLER + "/set_parameters")
        req = SetParameters.Request(parameters=[p.to_parameter_msg() for p in params])
        res = call(node, set_cli, req, "set_parameters", timeout_s)
        failed = [p.name for p, r in zip(params, res.results) if not r.successful]
        if failed:
            raise SystemExit("controller rejected parameters: " + ", ".join(failed))
        print(f"pushed {len(params)} predict_params to {CONTROLLER}")

        replan_cli = node.create_client(Trigger, CONTROLLER + "/replan")
        res = call(node, replan_cli, Trigger.Request(), "replan", timeout_s)
        if not res.success:
            raise SystemExit(f"replan failed: {res.message}")
        print(f"replanned: {res.message}")

    while not got and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    return got[-1] if got else None


def as_grid(values, n_speeds, n_ang):
    return np.asarray(values, dtype=float).reshape(n_speeds, n_ang)


def plot(msg, out_path, surface):
    speeds = np.asarray(msg.speeds)
    ang_deg = np.degrees(np.asarray(msg.ang_vels))
    n_s, n_a = len(speeds), len(ang_deg)
    best = (msg.best_speed_idx, msg.best_ang_vel_idx)
    sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    # pixel edges so each cell is centred on its (ang_vel, speed) value
    def edges(v):
        step = v[1] - v[0] if len(v) > 1 else 1.0
        return np.concatenate([v - step / 2, [v[-1] + step / 2]])
    extent = [edges(ang_deg)[0], edges(ang_deg)[-1], edges(speeds)[0], edges(speeds)[-1]]

    fig, axes = plt.subplots(2, 3, figsize=(17, 9), constrained_layout=True)
    axes = axes.ravel()
    for ax, (field, label) in zip(axes, COMPONENTS):
        g = as_grid(getattr(msg, field), n_s, n_a)
        im = ax.imshow(g, origin="lower", aspect="auto", extent=extent, cmap="viridis")
        fig.colorbar(im, ax=ax, shrink=0.85)
        ax.plot(ang_deg[best[1]], speeds[best[0]], marker="*", color="red",
                markersize=14, markeredgecolor="white")
        ax.set_title(f"{label}  (min {g.min():.3g}, max {g.max():.3g})")
        ax.set_xlabel("angular velocity [deg/s]")
        ax.set_ylabel("speed [m/s]")

    # last panel: optional 3D surface of the total, else a slice through the chosen row/col
    ax = axes[-1]
    total = as_grid(msg.total, n_s, n_a)
    if surface:
        ax.remove()
        ax = fig.add_subplot(2, 3, 6, projection="3d")
        A, S = np.meshgrid(ang_deg, speeds)
        ax.plot_surface(A, S, total, cmap="viridis", linewidth=0, antialiased=True)
        ax.set_xlabel("ang vel [deg/s]")
        ax.set_ylabel("speed [m/s]")
        ax.set_zlabel("total cost")
        ax.set_title("total (surface)")
    else:
        ax.plot(ang_deg, total[best[0], :], label=f"total @ speed={speeds[best[0]]:.2f}")
        ax.set_xlabel("angular velocity [deg/s]")
        ax.set_ylabel("total cost")
        ax2 = ax.twiny()
        ax2.plot(speeds, total[:, best[1]], color="tab:orange",
                 label=f"total @ ang_vel={ang_deg[best[1]]:.1f} deg/s")
        ax2.set_xlabel("speed [m/s]")
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, loc="upper right")
        ax.set_title("slices through the chosen cell")

    fig.suptitle(
        f"sim t = {sim_t:.3f} s   |   robot ({msg.robot_x:.2f}, {msg.robot_y:.2f}) "
        f"yaw {np.degrees(msg.robot_yaw):.1f} deg   |   goal ({msg.goal_x:.2f}, {msg.goal_y:.2f})   |   "
        f"{msg.num_agents} agents   |   chosen: speed {speeds[best[0]]:.2f} m/s, "
        f"ang vel {ang_deg[best[1]]:.1f} deg/s\n"
        f"weights: dist {msg.dist_cost_weight:g}   away_from_goal {msg.away_from_goal_weight:g}   "
        f"diff_from_prev {msg.diff_from_prev_weight:g}   diff_from_preferred {msg.diff_from_preferred_weight:g}"
        f"   |   ahead {msg.ahead_time:g} s   ignore_dist {msg.ignore_dist_threshold:g} m",
        fontsize=11)
    fig.savefig(out_path, dpi=130)
    return fig


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--output", "-o", help="output PNG path (default: crowd_sim/tuning/cost_grid_<simtime>.png)")
    ap.add_argument("--timeout", type=float, default=5.0, help="seconds to wait for a grid (default 5)")
    ap.add_argument("--surface", action="store_true", help="draw a 3D surface of the total instead of slices")
    ap.add_argument("--no-show", action="store_true", help="only save the PNG, do not open a window")
    ap.add_argument("--reload", nargs="?", const=DEFAULT_PARAMS, metavar="YAML",
                    help="push predict_params from controller_params.yaml (or YAML) to the running "
                         "controller, replan from its last scan, then plot")
    args = ap.parse_args()

    if args.no_show:
        matplotlib.use("Agg")   # headless; must be set before the first figure is created

    rclpy.init()
    try:
        msg = fetch_grid(args.timeout, reload_yaml=args.reload)
    finally:
        rclpy.shutdown()

    if msg is None:
        print(f"no CostGrid received on /cost_grid within {args.timeout:.0f}s. "
              "Is the controller running, and has it processed at least one lidar scan?", file=sys.stderr)
        return 1

    sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    # several --reload runs happen at the same paused sim time, so add a wall-clock suffix
    suffix = time.strftime("_%H%M%S") if args.reload else ""
    out_path = args.output or os.path.join(DEFAULT_OUT_DIR, f"cost_grid_{sim_t:09.3f}s{suffix}.png")
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    fig = plot(msg, out_path, args.surface)
    print(out_path)
    if not args.no_show:
        fig.canvas.manager.set_window_title(os.path.basename(out_path))
        plt.show()   # blocks until the window is closed
    plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
