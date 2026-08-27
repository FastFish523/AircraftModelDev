"""Offline trajectory and attitude replay tool for HTV2 result.dat files."""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np


# Columns in src/HTV2/FileSaver.cpp (zero-based here).
RESULT_COLUMNS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 42, 47)
MAX_PLOT_POINTS = 5000
MAX_MODEL_TRIANGLES = 10000
EARTH_RADIUS_M = 6_371_000.0
SHOW_EARTH = False
DEFAULT_STL = Path(__file__).resolve().parent / "assets" / "Fighter_jet_concept.stl"


@dataclass(frozen=True)
class ReplayData:
    time: np.ndarray
    north: np.ndarray
    up: np.ndarray
    east: np.ndarray
    speed: np.ndarray
    yaw: np.ndarray
    pitch: np.ndarray
    roll: np.ndarray
    velocity_theta: np.ndarray
    velocity_psi: np.ndarray
    alpha: np.ndarray
    beta: np.ndarray
    altitude: np.ndarray
    phase: np.ndarray

    @property
    def enu(self) -> np.ndarray:
        return np.column_stack((self.east, self.north, self.up))

    @property
    def size(self) -> int:
        return int(self.time.size)


def load_result_file(path: str | Path) -> ReplayData:
    """Load only the columns needed by the replay UI."""
    source = Path(path)
    if not source.is_file():
        raise ValueError(f"文件不存在：{source}")

    try:
        raw = np.loadtxt(source, dtype=np.float64, usecols=RESULT_COLUMNS, ndmin=2)
    except (OSError, ValueError) as exc:
        raise ValueError(f"无法读取 result.dat，文件必须包含至少 48 列：{exc}") from exc

    if raw.shape[0] == 0:
        raise ValueError("result.dat 为空。")

    finite_time = np.isfinite(raw[:, 0])
    if not np.any(finite_time):
        raise ValueError("result.dat 中没有有效的时间数据。")
    raw = raw[finite_time]

    if np.any(np.diff(raw[:, 0]) < 0.0):
        raise ValueError("时间列不是单调递增，无法可靠回放。")

    positions = raw[:, (3, 1, 2)]  # selected columns -> East, North, Up
    if not np.any(np.all(np.isfinite(positions), axis=1)):
        raise ValueError("result.dat 中没有有效的 NUE 位置数据。")

    return ReplayData(*(raw[:, index] for index in range(raw.shape[1])))


def rotation_body_to_nue(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """Reproduce CoordinateHelper::euler231ToQuaternion as Ry * Rz * Rx."""
    yaw, pitch, roll = np.deg2rad([yaw_deg, pitch_deg, roll_deg])
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)

    ry = np.array(((cy, 0.0, sy), (0.0, 1.0, 0.0), (-sy, 0.0, cy)))
    rz = np.array(((cp, -sp, 0.0), (sp, cp, 0.0), (0.0, 0.0, 1.0)))
    rx = np.array(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
    return ry @ rz @ rx


def body_axes_enu(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """Return body front/up/right axes as columns expressed in ENU coordinates."""
    c_body_to_nue = rotation_body_to_nue(yaw_deg, pitch_deg, roll_deg)
    nue_to_enu = np.array(((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)))
    return nue_to_enu @ c_body_to_nue


def chase_view_angles(body_axes: np.ndarray) -> tuple[float, float]:
    """Return elevation/azimuth for a camera behind, above, and right of the body."""
    axes = np.asarray(body_axes, dtype=np.float64)
    if axes.shape != (3, 3) or not np.all(np.isfinite(axes)):
        raise ValueError("Body axes must be a finite 3x3 matrix.")
    front, up, right = axes.T
    camera_direction = -front + 0.35 * up + 0.25 * right
    camera_direction /= np.linalg.norm(camera_direction)
    elevation = math.degrees(math.asin(float(np.clip(camera_direction[2], -1.0, 1.0))))
    azimuth = math.degrees(math.atan2(float(camera_direction[1]), float(camera_direction[0])))
    return elevation, azimuth


def trajectory_frame_enu(points: np.ndarray, row: int, half_window: int = 100) -> np.ndarray:
    """Return a front/up/right frame aligned with the local trajectory tangent."""
    positions = np.asarray(points, dtype=np.float64)
    if positions.ndim != 2 or positions.shape[1] != 3 or not 0 <= row < positions.shape[0]:
        raise ValueError("Trajectory positions must be an Nx3 array with a valid row.")
    start = max(0, row - half_window)
    stop = min(positions.shape[0], row + half_window + 1)
    local = positions[start:stop]
    local = local[np.all(np.isfinite(local), axis=1)]
    if local.shape[0] < 2:
        raise ValueError("At least two finite local trajectory points are required.")
    front = local[-1] - local[0]
    front_norm = float(np.linalg.norm(front))
    if front_norm <= 0.0:
        raise ValueError("Local trajectory direction is zero.")
    front /= front_norm
    world_up = np.array((0.0, 0.0, 1.0))
    right = np.cross(front, world_up)
    if np.linalg.norm(right) < 1e-9:
        right = np.array((1.0, 0.0, 0.0))
    else:
        right /= np.linalg.norm(right)
    up = np.cross(right, front)
    return np.column_stack((front, up, right))


def plot_indices(row_count: int, maximum: int = MAX_PLOT_POINTS) -> np.ndarray:
    if row_count <= 0:
        return np.empty(0, dtype=np.int64)
    if row_count <= maximum:
        return np.arange(row_count, dtype=np.int64)
    return np.unique(np.linspace(0, row_count - 1, maximum, dtype=np.int64))


def load_stl_triangles(path: str | Path, maximum: int = MAX_MODEL_TRIANGLES) -> np.ndarray:
    """Load binary or ASCII STL triangles and return a centered unit-length mesh."""
    source = Path(path)
    if not source.is_file():
        raise ValueError(f"STL file does not exist: {source}")
    payload = source.read_bytes()
    if len(payload) < 84:
        raise ValueError("Invalid STL file: file is too short.")

    triangle_count = int.from_bytes(payload[80:84], "little")
    expected_size = 84 + triangle_count * 50
    if triangle_count > 0 and expected_size == len(payload):
        record = np.dtype([
            ("normal", "<f4", (3,)),
            ("vertices", "<f4", (3, 3)),
            ("attribute", "<u2"),
        ])
        triangles = np.frombuffer(payload, dtype=record, count=triangle_count, offset=84)["vertices"].astype(np.float64)
    else:
        vertices = []
        for line in payload.decode("utf-8", errors="ignore").splitlines():
            fields = line.strip().split()
            if len(fields) == 4 and fields[0].lower() == "vertex":
                try:
                    vertices.append([float(value) for value in fields[1:]])
                except ValueError as exc:
                    raise ValueError("Invalid numeric value in ASCII STL.") from exc
        if not vertices or len(vertices) % 3 != 0:
            raise ValueError("Invalid STL file: no complete triangles found.")
        triangles = np.asarray(vertices, dtype=np.float64).reshape(-1, 3, 3)

    if not np.all(np.isfinite(triangles)):
        raise ValueError("Invalid STL file: mesh contains non-finite vertices.")
    if triangles.shape[0] > maximum:
        triangles = triangles[plot_indices(triangles.shape[0], maximum)]

    points = triangles.reshape(-1, 3)
    minimum, maximum_xyz = points.min(axis=0), points.max(axis=0)
    extent = maximum_xyz - minimum
    longitudinal_axis = int(np.argmax(extent))
    remaining = [axis for axis in range(3) if axis != longitudinal_axis]
    lateral_axis = remaining[int(np.argmax(extent[remaining]))]
    vertical_axis = next(axis for axis in remaining if axis != lateral_axis)
    # Model coordinates follow the simulation body-axis order: front, up, right.
    triangles = triangles[..., [longitudinal_axis, vertical_axis, lateral_axis]]
    # This STL points toward its negative longitudinal axis; body +X points forward.
    triangles[..., 0] *= -1.0
    model_points = triangles.reshape(-1, 3)
    triangles -= (model_points.min(axis=0) + model_points.max(axis=0)) / 2.0
    length = float(np.ptp(triangles[..., 0]))
    if length <= 0.0:
        raise ValueError("Invalid STL file: model has zero length.")
    return triangles / length


def curved_earth_surface(points: np.ndarray, grid_size: int = 36) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build a spherical Earth patch below a trajectory expressed in local ENU."""
    finite = points[np.all(np.isfinite(points), axis=1)]
    if finite.size == 0:
        raise ValueError("Cannot build Earth surface without valid positions.")
    minimum, maximum = finite[:, :2].min(axis=0), finite[:, :2].max(axis=0)
    span = np.maximum(maximum - minimum, 1.0)
    margin = np.maximum(span * 0.08, 1000.0)
    east = np.linspace(minimum[0] - margin[0], maximum[0] + margin[0], grid_size)
    north = np.linspace(minimum[1] - margin[1], maximum[1] + margin[1], grid_size)
    east_grid, north_grid = np.meshgrid(east, north)
    radial_sq = east_grid**2 + north_grid**2
    up_grid = np.sqrt(np.maximum(EARTH_RADIUS_M**2 - radial_sq, 0.0)) - EARTH_RADIUS_M
    up_grid[radial_sq >= (0.995 * EARTH_RADIUS_M) ** 2] = np.nan
    return east_grid, north_grid, up_grid


def earth_sphere(longitudes: int = 96, latitudes: int = 49) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build a complete Earth sphere tangent to the local ENU origin."""
    longitude = np.linspace(0.0, 2.0 * math.pi, longitudes)
    colatitude = np.linspace(0.0, math.pi, latitudes)
    longitude_grid, colatitude_grid = np.meshgrid(longitude, colatitude)
    east = EARTH_RADIUS_M * np.sin(colatitude_grid) * np.cos(longitude_grid)
    north = EARTH_RADIUS_M * np.sin(colatitude_grid) * np.sin(longitude_grid)
    up = EARTH_RADIUS_M * np.cos(colatitude_grid) - EARTH_RADIUS_M
    return east, north, up

def _fmt(value: float, unit: str = "") -> str:
    return f"{value:.3f}{unit}" if np.isfinite(value) else "--"


class ReplayApp:
    TIMER_MS = 33

    def __init__(self, initial_file: str | None = None) -> None:
        import tkinter as tk
        from tkinter import ttk
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
        from matplotlib.figure import Figure
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        self.tk = tk
        self.ttk = ttk
        self.Poly3DCollection = Poly3DCollection
        self.root = tk.Tk()
        self.root.title("HTV2 飞行轨迹与姿态回放")
        self.root.geometry("1280x800")
        self.root.minsize(900, 600)

        self.data: ReplayData | None = None
        self.display_indices = np.empty(0, dtype=np.int64)
        self.current_time = 0.0
        self.playing = False
        self.last_tick = time.perf_counter()
        self.arrow_artists: list[object] = []
        self.model_triangles = load_stl_triangles(DEFAULT_STL) if DEFAULT_STL.is_file() else None
        self.model_artist: Poly3DCollection | None = None
        self._updating_slider = False
        self._slider_dragging = False

        controls = ttk.Frame(self.root, padding=6)
        controls.pack(fill=tk.X)
        ttk.Button(controls, text="加载 result.dat", command=self.choose_file).pack(side=tk.LEFT)
        self.play_button = ttk.Button(controls, text="播放", command=self.toggle_play, state=tk.DISABLED)
        self.play_button.pack(side=tk.LEFT, padx=(8, 0))
        self.reset_button = ttk.Button(controls, text="复位", command=self.reset, state=tk.DISABLED)
        self.reset_button.pack(side=tk.LEFT, padx=(4, 0))
        ttk.Label(controls, text="倍速：").pack(side=tk.LEFT, padx=(16, 2))
        self.speed_var = tk.StringVar(value="1×")
        ttk.Combobox(
            controls, textvariable=self.speed_var, values=("1×", "5×", "20×"),
            width=5, state="readonly"
        ).pack(side=tk.LEFT)
        self.file_label = ttk.Label(controls, text="尚未加载文件")
        self.file_label.pack(side=tk.LEFT, padx=16)

        content = ttk.Frame(self.root)
        content.pack(fill=tk.BOTH, expand=True)
        plot_frame = ttk.Frame(content)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        info_frame = ttk.LabelFrame(content, text="飞行状态", padding=12, width=210)
        info_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(4, 8), pady=4)
        info_frame.pack_propagate(False)

        self.figure = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.figure.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        NavigationToolbar2Tk(self.canvas, plot_frame).update()
        self._prepare_axes()

        self.info_vars: dict[str, tk.StringVar] = {}
        for label, key in (
            ("攻角", "alpha"), ("侧滑角", "beta"),
            ("速度倾角", "velocity_theta"), ("速度偏角", "velocity_psi"),
            ("时间", "time"), ("速度", "speed"), ("高度", "altitude"),
            ("Yaw", "yaw"), ("Pitch", "pitch"), ("Roll", "roll"), ("阶段", "phase")
        ):
            row = ttk.Frame(info_frame)
            row.pack(fill=tk.X, pady=5)
            ttk.Label(row, text=f"{label}：", width=8).pack(side=tk.LEFT)
            var = tk.StringVar(value="--")
            ttk.Label(row, textvariable=var).pack(side=tk.LEFT)
            self.info_vars[key] = var

        slider_frame = ttk.Frame(self.root, padding=(10, 4, 10, 10))
        slider_frame.pack(fill=tk.X)
        self.slider = tk.Scale(
            slider_frame, from_=0.0, to=1.0, resolution=0.001,
            orient=tk.HORIZONTAL, showvalue=False, command=self.on_slider,
            state=tk.DISABLED
        )
        self.slider.pack(fill=tk.X)
        self.slider.bind("<ButtonPress-1>", self._begin_slider_drag)
        self.slider.bind("<ButtonRelease-1>", self._end_slider_drag)
        self.status_var = tk.StringVar(value="请选择仿真生成的 result.dat。")
        ttk.Label(slider_frame, textvariable=self.status_var).pack(anchor=tk.W)

        self.root.after(self.TIMER_MS, self.tick)
        if initial_file:
            self.root.after(0, lambda: self.load_file(initial_file))

    def _prepare_axes(self) -> None:
        self.ax.set_axis_off()

    def choose_file(self) -> None:
        from tkinter import filedialog
        selected = filedialog.askopenfilename(
            title="选择 HTV2 result.dat", filetypes=(("HTV2 trajectory", "*.dat"), ("All files", "*.*"))
        )
        if selected:
            self.load_file(selected)

    def load_file(self, path: str) -> None:
        from tkinter import messagebox
        self.pause()
        self.status_var.set("正在加载，请稍候……")
        self.root.update_idletasks()
        try:
            data = load_result_file(path)
        except ValueError as exc:
            self.status_var.set("加载失败，可重新选择文件。")
            messagebox.showerror("加载失败", str(exc), parent=self.root)
            return

        self.data = data
        self.display_indices = plot_indices(data.size)
        self.current_time = float(data.time[0])
        duration = max(float(data.time[-1] - data.time[0]), 0.0)
        resolution = max(duration / 10000.0, 0.001)
        self.slider.configure(from_=float(data.time[0]), to=float(data.time[-1]), resolution=resolution, state=self.tk.NORMAL)
        self.play_button.configure(state=self.tk.NORMAL)
        self.reset_button.configure(state=self.tk.NORMAL)
        self.file_label.configure(text=str(Path(path)))
        self._draw_loaded_trajectory()
        self.update_frame(self.current_time)
        self.status_var.set(f"已加载 {data.size:,} 行；轨迹显示 {self.display_indices.size:,} 点。")

    def _draw_loaded_trajectory(self) -> None:
        assert self.data is not None
        self.ax.clear()
        self._prepare_axes()
        enu = self.data.enu
        shown = enu[self.display_indices]
        valid = np.all(np.isfinite(shown), axis=1)
        self.ax.plot(shown[valid, 0], shown[valid, 1], shown[valid, 2], color="0.72", linewidth=1.0, label="Full trajectory")
        (self.progress_line,) = self.ax.plot([], [], [], color="#f59e0b", linewidth=2.0, label="Flown trajectory")
        (self.position_marker,) = self.ax.plot([], [], [], marker="o", color="black", markersize=5)
        if SHOW_EARTH:
            earth_east, earth_north, earth_up = earth_sphere()
            self.ax.plot_surface(
                earth_east, earth_north, earth_up,
                color="#9fd3e8", alpha=0.88, linewidth=0.0,
                antialiased=True, shade=True,
            )
        self.ax.legend(loc="upper right")
        self._set_equal_limits(shown[valid])
        self.arrow_length = max(float(np.ptp(shown[valid], axis=0).max()) * 0.05, 10.0)
        self.model_length = self.arrow_length * 1.5
        self.chase_view_radius = self.model_length * 2.2
        self.arrow_artists = []
        if self.model_triangles is not None:
            self.model_artist = self.Poly3DCollection(
                [], facecolor="#d9dce1", edgecolor="#596273", linewidth=0.06,
                alpha=1.0, antialiased=True
            )
            self.ax.add_collection3d(self.model_artist)
        else:
            self.model_artist = None
        self.canvas.draw_idle()

    def _set_equal_limits(self, points: np.ndarray) -> None:
        minima, maxima = np.min(points, axis=0), np.max(points, axis=0)
        centers = (minima + maxima) / 2.0
        radius = max(float(np.max(maxima - minima)) / 2.0, 1.0)
        self.ax.set_xlim(centers[0] - radius, centers[0] + radius)
        self.ax.set_ylim(centers[1] - radius, centers[1] + radius)
        self.ax.set_zlim(centers[2] - radius, centers[2] + radius)
        self.ax.set_box_aspect((1.0, 1.0, 1.0))

    def update_frame(self, requested_time: float) -> None:
        if self.data is None:
            return
        end_time = float(self.data.time[-1])
        self.current_time = min(max(requested_time, float(self.data.time[0])), end_time)
        row = min(int(np.searchsorted(self.data.time, self.current_time, side="right") - 1), self.data.size - 1)
        row = max(row, 0)

        shown_rows = self.display_indices[self.display_indices <= row]
        progress = self.data.enu[shown_rows]
        progress = progress[np.all(np.isfinite(progress), axis=1)]
        self.progress_line.set_data_3d(progress[:, 0], progress[:, 1], progress[:, 2])

        for artist in self.arrow_artists:
            artist.remove()
        self.arrow_artists.clear()

        position = self.data.enu[row]
        angles = (self.data.yaw[row], self.data.pitch[row], self.data.roll[row])
        if np.all(np.isfinite(position)):
            self.position_marker.set_data_3d([position[0]], [position[1]], [position[2]])
            if np.all(np.isfinite(angles)):
                axes = body_axes_enu(*angles)
                try:
                    view_frame = trajectory_frame_enu(self.data.enu, row)
                except ValueError:
                    view_frame = axes
                elevation, azimuth = chase_view_angles(view_frame)
                self.ax.view_init(elev=elevation, azim=azimuth, roll=0.0)
                self.ax.set_xlim(position[0] - self.chase_view_radius, position[0] + self.chase_view_radius)
                self.ax.set_ylim(position[1] - self.chase_view_radius, position[1] + self.chase_view_radius)
                self.ax.set_zlim(position[2] - self.chase_view_radius, position[2] + self.chase_view_radius)
                if self.model_artist is not None and self.model_triangles is not None:
                    model_enu = self.model_triangles @ axes.T
                    model_enu = model_enu * self.model_length + position
                    self.model_artist.set_verts(model_enu)
                for axis, color, name in zip(
                    axes.T,
                    ("red", "green", "blue"),
                    ("Front (Xb)", "Up (Yb)", "Right (Zb)"),
                ):
                    arrow = self.ax.quiver(*position, *axis, length=self.arrow_length, normalize=True, color=color, label=name)
                    self.arrow_artists.append(arrow)
        else:
            self.position_marker.set_data_3d([], [], [])
            if self.model_artist is not None:
                self.model_artist.set_verts([])

        self.info_vars["time"].set(_fmt(self.data.time[row], " s"))
        self.info_vars["speed"].set(_fmt(self.data.speed[row], " m/s"))
        self.info_vars["altitude"].set(_fmt(self.data.altitude[row], " m"))
        self.info_vars["yaw"].set(_fmt(self.data.yaw[row], "°"))
        self.info_vars["pitch"].set(_fmt(self.data.pitch[row], "°"))
        self.info_vars["roll"].set(_fmt(self.data.roll[row], "°"))
        self.info_vars["alpha"].set(_fmt(self.data.alpha[row], "°"))
        self.info_vars["beta"].set(_fmt(self.data.beta[row], "°"))
        self.info_vars["velocity_theta"].set(_fmt(self.data.velocity_theta[row], "°"))
        self.info_vars["velocity_psi"].set(_fmt(self.data.velocity_psi[row], "°"))
        self.info_vars["phase"].set(str(int(self.data.phase[row])) if np.isfinite(self.data.phase[row]) else "--")

        self._updating_slider = True
        self.slider.set(self.current_time)
        self._updating_slider = False
        self.canvas.draw_idle()

    def on_slider(self, value: str) -> None:
        if self.data is None or self._updating_slider or not self._slider_dragging:
            return
        self.update_frame(float(value))

    def _begin_slider_drag(self, _event: object) -> None:
        self._slider_dragging = True
        self.pause()

    def _end_slider_drag(self, _event: object) -> None:
        if self.data is not None:
            self.update_frame(float(self.slider.get()))
        self._slider_dragging = False

    def toggle_play(self) -> None:
        if self.data is None:
            return
        if self.playing:
            self.pause()
        else:
            if self.current_time >= float(self.data.time[-1]):
                self.current_time = float(self.data.time[0])
            self.playing = True
            self.last_tick = time.perf_counter()
            self.play_button.configure(text="暂停")

    def pause(self) -> None:
        self.playing = False
        if hasattr(self, "play_button"):
            self.play_button.configure(text="播放")

    def reset(self) -> None:
        if self.data is not None:
            self.pause()
            self.update_frame(float(self.data.time[0]))

    def tick(self) -> None:
        now = time.perf_counter()
        if self.playing and self.data is not None:
            multiplier = float(self.speed_var.get().removesuffix("×"))
            next_time = self.current_time + (now - self.last_tick) * multiplier
            if next_time >= float(self.data.time[-1]):
                next_time = float(self.data.time[-1])
                self.pause()
            self.update_frame(next_time)
        self.last_tick = now
        self.root.after(self.TIMER_MS, self.tick)

    def run(self) -> None:
        self.root.mainloop()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Replay a HTV2 result.dat trajectory.")
    parser.add_argument("result", nargs="?", help="Optional result.dat loaded at startup")
    parser.add_argument("--check", action="store_true", help="Validate the file without opening the GUI")
    args = parser.parse_args(argv)
    if args.check:
        if not args.result:
            parser.error("--check requires a result.dat path")
        data = load_result_file(args.result)
        print(f"OK: {data.size} rows, t={data.time[0]:.3f}..{data.time[-1]:.3f} s")
        return 0
    ReplayApp(args.result).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
