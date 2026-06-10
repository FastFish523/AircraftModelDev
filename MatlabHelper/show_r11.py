#!/usr/bin/env python3
"""Plot R11 trajectory data saved by Results/R11/result.dat."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


COLUMNS = [
    "time",
    "xg",
    "yg",
    "zg",
    "v",
    "yaw",
    "pitch",
    "roll",
    "theta",
    "psi",
    "alpha",
    "beta",
    "accx",
    "accy",
    "accz",
    "mass",
    "P",
    "acc_cmd_by",
    "acc_cmd_bz",
    "xgt",
    "ygt",
    "zgt",
    "dx",
    "dy",
    "dz",
    "vx",
    "vy",
    "vz",
    "wx",
    "wy",
    "wz",
    "lon",
    "lat",
    "h",
    "sigma_evl",
    "sigma_az",
    "sigma_evl_dot",
    "sigma_az_dot",
    "p_body_x",
    "p_body_y",
    "p_body_z",
    "m_body_x",
    "m_body_y",
    "m_body_z",
    "hold_phase_active",
    "hold_moment_active",
    "hold_inside_cone",
    "hold_view_angle_deg",
]

REQUIRED_COLUMNS = 38


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def load_result(path: Path) -> dict[str, np.ndarray]:
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < REQUIRED_COLUMNS:
        raise ValueError(f"{path} has {data.shape[1]} columns, expected at least {REQUIRED_COLUMNS}")
    available_columns = COLUMNS[: data.shape[1]]
    return {name: data[:, index] for index, name in enumerate(available_columns)}


def print_result_summary(path: Path, data: dict[str, np.ndarray]) -> None:
    time = data["time"]
    print(f"Result file: {path}")
    print(f"Samples: {len(time)}")
    print(f"Columns: {len(data)}")
    if len(time) > 0:
        print(f"Time range: {time[0]:.6f} s -> {time[-1]:.6f} s")
        print(f"Final position NUE: x={data['xg'][-1]:.3f} m, y={data['yg'][-1]:.3f} m, z={data['zg'][-1]:.3f} m")
        print(f"Final LLA: lon={data['lon'][-1]:.6f} deg, lat={data['lat'][-1]:.6f} deg, h={data['h'][-1]:.3f} m")


def attach_close_shortcuts() -> None:
    def close_all(event) -> None:
        if event.key in {"q", "escape"}:
            plt.close("all")

    for num in plt.get_fignums():
        plt.figure(num).canvas.mpl_connect("key_press_event", close_all)


def set_grid(axes) -> None:
    for ax in np.ravel(axes):
        ax.grid(True)


def get_hold_mask(d: dict[str, np.ndarray]) -> np.ndarray | None:
    if "hold_phase_active" not in d:
        return None
    return d["hold_phase_active"] > 0.5


def plot_trajectory(d: dict[str, np.ndarray]) -> None:
    fig, axes = plt.subplots(2, 2, num="R11 trajectory", constrained_layout=True)
    fig.suptitle("R11 trajectory")
    hold_mask = get_hold_mask(d)

    axes[0, 0].plot(d["xg"], d["yg"], label="missile")
    if hold_mask is not None and np.any(hold_mask):
        axes[0, 0].plot(d["xg"][hold_mask], d["yg"][hold_mask], color="orange", linewidth=2.5, label="attitude hold")
    axes[0, 0].plot(d["xgt"], d["ygt"], "r", label="target")
    axes[0, 0].plot(d["xgt"][-1], d["ygt"][-1], "r*", label="target end")
    axes[0, 0].set_title("N-E plane")
    axes[0, 0].set_xlabel("xg / m")
    axes[0, 0].set_ylabel("yg / m")
    axes[0, 0].legend()

    axes[0, 1].plot(d["zg"], d["yg"], label="missile")
    if hold_mask is not None and np.any(hold_mask):
        axes[0, 1].plot(d["zg"][hold_mask], d["yg"][hold_mask], color="orange", linewidth=2.5, label="attitude hold")
    axes[0, 1].plot(d["zgt"], d["ygt"], "r", label="target")
    axes[0, 1].plot(d["zgt"][-1], d["ygt"][-1], "r*")
    axes[0, 1].set_title("U-E plane")
    axes[0, 1].set_xlabel("zg / m")
    axes[0, 1].set_ylabel("yg / m")

    axes[1, 0].plot(d["zg"], d["xg"], label="missile")
    if hold_mask is not None and np.any(hold_mask):
        axes[1, 0].plot(d["zg"][hold_mask], d["xg"][hold_mask], color="orange", linewidth=2.5, label="attitude hold")
    axes[1, 0].plot(d["zgt"], d["xgt"], "r", label="target")
    axes[1, 0].plot(d["zgt"][-1], d["xgt"][-1], "r*")
    axes[1, 0].set_title("U-N plane")
    axes[1, 0].set_xlabel("zg / m")
    axes[1, 0].set_ylabel("xg / m")

    ax3d = fig.add_subplot(2, 2, 4, projection="3d")
    ax3d.plot(d["xg"], d["zg"], d["yg"], label="missile")
    if hold_mask is not None and np.any(hold_mask):
        ax3d.plot(d["xg"][hold_mask], d["zg"][hold_mask], d["yg"][hold_mask], color="orange", linewidth=2.5, label="attitude hold")
    ax3d.plot(d["xgt"], d["zgt"], d["ygt"], "r", label="target")
    ax3d.scatter(d["xgt"][-1], d["zgt"][-1], d["ygt"][-1], marker="*", c="r")
    ax3d.set_title("3D trajectory")
    ax3d.set_xlabel("xg / m")
    ax3d.set_ylabel("zg / m")
    ax3d.set_zlabel("yg / m")
    ax3d.legend()
    set_grid(axes)


def shade_hold_phase(axes, d: dict[str, np.ndarray]) -> None:
    hold_mask = get_hold_mask(d)
    if hold_mask is None or not np.any(hold_mask):
        return

    time = d["time"]
    for ax in np.ravel(axes):
        ymin, ymax = ax.get_ylim()
        ax.fill_between(time, ymin, ymax, where=hold_mask, color="orange", alpha=0.12, linewidth=0)
        ax.set_ylim(ymin, ymax)


def plot_time_series(d: dict[str, np.ndarray], cone_angle: float | None = None) -> None:
    time = d["time"]

    fig, axes = plt.subplots(2, 1, num="R11 speed and altitude", constrained_layout=True)
    axes[0].plot(time, d["v"])
    axes[0].set_title("Time vs speed")
    axes[0].set_ylabel("speed / m/s")
    axes[1].plot(time, d["h"], "r")
    axes[1].set_title("Time vs altitude")
    axes[1].set_xlabel("time / s")
    axes[1].set_ylabel("altitude / m")
    set_grid(axes)
    shade_hold_phase(axes, d)

    fig, axes = plt.subplots(2, 2, num="R11 angles and rudder", constrained_layout=True)
    axes[0, 0].plot(time, d["alpha"], label="alpha")
    axes[0, 0].plot(time, d["pitch"], "r", label="pitch")
    axes[0, 0].plot(time, d["theta"], "c", label="theta")
    axes[0, 0].set_title("Longitudinal angles")
    axes[0, 0].legend()

    axes[0, 1].plot(time, d["beta"], "r", label="beta")
    axes[0, 1].plot(time, d["yaw"], "b", label="yaw")
    axes[0, 1].plot(time, d["psi"], "m", label="psi")
    axes[0, 1].set_title("Lateral angles")
    axes[0, 1].legend()

    axes[1, 0].plot(time, d["roll"])
    axes[1, 0].set_title("Roll")

    axes[1, 1].plot(time, d["dx"], "r", label="dx")
    axes[1, 1].plot(time, d["dy"], "k", label="dy")
    axes[1, 1].plot(time, d["dz"], label="dz")
    axes[1, 1].set_title("Rudder")
    axes[1, 1].legend()
    set_grid(axes)
    shade_hold_phase(axes, d)

    fig, axes = plt.subplots(2, 1, num="R11 velocity and angular rate", constrained_layout=True)
    axes[0].plot(time, d["vx"], "r", label="vx")
    axes[0].plot(time, d["vy"], "k", label="vy")
    axes[0].plot(time, d["vz"], label="vz")
    axes[0].set_title("Velocity components")
    axes[0].legend()
    axes[1].plot(time, d["wx"], "r", label="wx")
    axes[1].plot(time, d["wy"], "k", label="wy")
    axes[1].plot(time, d["wz"], label="wz")
    axes[1].set_title("Angular rate components")
    axes[1].legend()
    set_grid(axes)
    shade_hold_phase(axes, d)

    fig, axes = plt.subplots(2, 1, num="R11 acceleration", constrained_layout=True)
    axes[0].plot(time, d["accx"], "r")
    axes[0].set_title("Body x acceleration")
    axes[1].plot(time, d["accy"], "g", label="accy")
    axes[1].plot(time, d["accz"], "k", label="accz")
    axes[1].plot(time, d["acc_cmd_by"], "-.", label="acc cmd by")
    axes[1].plot(time, d["acc_cmd_bz"], "--", label="acc cmd bz")
    axes[1].set_title("Body y/z acceleration and commands")
    axes[1].legend()
    set_grid(axes)
    shade_hold_phase(axes, d)

    fig, axes = plt.subplots(2, 1, num="R11 mass and thrust", constrained_layout=True)
    axes[0].plot(time, d["mass"])
    axes[0].set_title("Mass")
    axes[1].plot(time, d["P"])
    axes[1].set_title("Thrust")
    set_grid(axes)
    shade_hold_phase(axes, d)

    fig, axes = plt.subplots(2, 2, num="R11 line of sight", constrained_layout=True)
    axes[0, 0].plot(time, d["sigma_evl"], "k")
    axes[0, 0].set_title("LOS elevation")
    axes[0, 1].plot(time, d["sigma_az"], "k")
    axes[0, 1].set_title("LOS azimuth")
    axes[1, 0].plot(time, d["sigma_evl_dot"], "k")
    axes[1, 0].set_title("LOS elevation rate")
    axes[1, 1].plot(time, d["sigma_az_dot"], "k")
    axes[1, 1].set_title("LOS azimuth rate")
    set_grid(axes)
    shade_hold_phase(axes, d)

    fig, axes = plt.subplots(2, 1, num="R11 geodetic track", constrained_layout=True)
    axes[0].plot(d["lon"], d["lat"], "b-", linewidth=2)
    axes[0].scatter([d["lon"][0], d["lon"][-1]], [d["lat"][0], d["lat"][-1]], c="r", s=30)
    axes[0].set_title("Longitude-latitude track")
    axes[0].set_xlabel("longitude / deg")
    axes[0].set_ylabel("latitude / deg")
    axes[1].plot(d["lon"], d["h"], "r-", linewidth=2)
    axes[1].set_title("Altitude profile")
    axes[1].set_xlabel("longitude / deg")
    axes[1].set_ylabel("altitude / m")
    set_grid(axes)
    shade_hold_phase(axes, d)

    if {"p_body_x", "p_body_y", "p_body_z", "m_body_x", "m_body_y", "m_body_z"}.issubset(d):
        fig, axes = plt.subplots(2, 1, num="R11 control force and moment", constrained_layout=True)
        axes[0].plot(time, d["p_body_x"], "r", label="p body x")
        axes[0].plot(time, d["p_body_y"], "g", label="p body y")
        axes[0].plot(time, d["p_body_z"], "b", label="p body z")
        axes[0].set_title("Body force command")
        axes[0].set_ylabel("force / N")
        axes[0].legend()

        axes[1].plot(time, d["m_body_x"], "r", label="m body x")
        axes[1].plot(time, d["m_body_y"], "g", label="m body y")
        axes[1].plot(time, d["m_body_z"], "b", label="m body z")
        axes[1].set_title("Body moment command")
        axes[1].set_xlabel("time / s")
        axes[1].set_ylabel("moment / N*m")
        axes[1].legend()
        set_grid(axes)
        shade_hold_phase(axes, d)

    if {"hold_phase_active", "hold_moment_active", "hold_inside_cone", "hold_view_angle_deg"}.issubset(d):
        fig, axes = plt.subplots(2, 1, num="R11 terminal view constraint", constrained_layout=True)
        axes[0].plot(time, d["hold_view_angle_deg"], "k", label="view angle")
        if cone_angle is not None:
            axes[0].axhline(cone_angle, color="r", linestyle="--", label="cone angle")
        axes[0].set_title("View axis to target angle")
        axes[0].set_ylabel("angle / deg")
        axes[0].legend()

        axes[1].plot(time, d["hold_phase_active"], label="hold phase")
        axes[1].plot(time, d["hold_moment_active"], label="moment active")
        axes[1].plot(time, d["hold_inside_cone"], label="inside cone")
        axes[1].set_title("Terminal hold states")
        axes[1].set_xlabel("time / s")
        axes[1].set_ylim(-0.1, 1.1)
        axes[1].legend()
        set_grid(axes)
        shade_hold_phase(axes, d)


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize R11 result.dat data.")
    parser.add_argument(
        "result",
        nargs="?",
        type=Path,
        default=repo_root() / "Results" / "R11" / "result.dat",
        help="Path to result.dat. Defaults to Results/R11/result.dat under repo root.",
    )
    parser.add_argument("--save-dir", type=Path, help="Save figures as PNG files instead of only showing them.")
    parser.add_argument("--cone-angle", type=float, help="Draw the terminal view cone half-angle threshold in degrees.")
    args = parser.parse_args()

    result_path = args.result.resolve()
    if not result_path.exists():
        raise FileNotFoundError(f"Result file not found: {result_path}")

    data = load_result(result_path)
    print_result_summary(result_path, data)
    plot_trajectory(data)
    plot_time_series(data, args.cone_angle)
    attach_close_shortcuts()

    if args.save_dir:
        args.save_dir.mkdir(parents=True, exist_ok=True)
        for num in plt.get_fignums():
            fig = plt.figure(num)
            name = fig.canvas.manager.get_window_title().lower().replace(" ", "_")
            fig.savefig(args.save_dir / f"{name}.png", dpi=150)
        print(f"Saved figures: {args.save_dir.resolve()}")

    plt.show()


if __name__ == "__main__":
    main()
