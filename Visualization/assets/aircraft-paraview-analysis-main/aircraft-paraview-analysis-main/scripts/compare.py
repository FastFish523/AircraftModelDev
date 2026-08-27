"""
Aircraft Metric Comparison Charts
Reads analysis_results.json and generates side-by-side comparison figures.
"""

import json
import os
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
REPO_DIR    = os.path.dirname(SCRIPT_DIR)
JSON_PATH   = os.path.join(REPO_DIR, "results", "analysis_results.json")
OUT_DIR     = os.path.join(REPO_DIR, "results", "charts")
os.makedirs(OUT_DIR, exist_ok=True)

# ── Style ─────────────────────────────────────────────────────────────────────
COLORS  = {"ATR_x": "#5599DD", "Fighter_Jet": "#E8943A"}
LABELS  = {"ATR_x": "ATR_x", "Fighter_Jet": "Fighter Jet"}
BG      = "#0E0F14"
GRID_C  = "#2A2B35"
TEXT_C  = "#D8DCE8"

plt.rcParams.update({
    "figure.facecolor":  BG,
    "axes.facecolor":    "#16171F",
    "axes.edgecolor":    GRID_C,
    "axes.labelcolor":   TEXT_C,
    "axes.titlecolor":   TEXT_C,
    "xtick.color":       TEXT_C,
    "ytick.color":       TEXT_C,
    "grid.color":        GRID_C,
    "grid.linewidth":    0.6,
    "text.color":        TEXT_C,
    "font.family":       "DejaVu Sans",
    "font.size":         11,
    "axes.titlesize":    13,
    "axes.labelsize":    11,
})


def load():
    with open(JSON_PATH) as f:
        return json.load(f)


def bar_pair(ax, labels, values, title, unit="", log=False):
    x    = np.arange(len(labels))
    cols = [COLORS[k] for k in labels]
    bars = ax.bar(x, values, color=cols, width=0.5, edgecolor="none", zorder=3)
    ax.set_xticks(x)
    ax.set_xticklabels([LABELS[k] for k in labels], fontsize=12)
    ax.set_title(title, pad=8)
    ax.yaxis.grid(True, zorder=0)
    ax.set_axisbelow(True)
    if log:
        ax.set_yscale("log")
    if unit:
        ax.set_ylabel(unit)
    for bar, val in zip(bars, values):
        fmt = f"{val:,.0f}" if val >= 10 else f"{val:.4f}"
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() * 1.02,
                fmt, ha="center", va="bottom", fontsize=10, color=TEXT_C)
    return ax


# ── Figure 1: Mesh statistics ─────────────────────────────────────────────────
def fig_mesh(data):
    keys   = list(data.keys())
    tris   = [data[k]["mesh"]["triangles"] for k in keys]
    verts  = [data[k]["mesh"]["vertices"]  for k in keys]
    ratio  = [data[k]["mesh"]["tri_to_vert_ratio"] for k in keys]

    fig, axes = plt.subplots(1, 3, figsize=(14, 5))
    fig.suptitle("Mesh Statistics", fontsize=15, y=1.01)

    bar_pair(axes[0], keys, tris,  "Triangles",  "count")
    bar_pair(axes[1], keys, verts, "Vertices",   "count")
    bar_pair(axes[2], keys, ratio, "Tri / Vert ratio")

    fig.tight_layout()
    path = os.path.join(OUT_DIR, "comparison_mesh.png")
    fig.savefig(path, dpi=150, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Figure 2: Geometry ────────────────────────────────────────────────────────
def fig_geometry(data):
    keys   = list(data.keys())
    areas  = [data[k]["geometry"]["surface_area"] for k in keys]
    vols   = [data[k]["geometry"]["volume"]        for k in keys]

    fig, axes = plt.subplots(1, 2, figsize=(10, 5))
    fig.suptitle("Geometry Measurements", fontsize=15, y=1.01)

    bar_pair(axes[0], keys, areas, "Surface Area",  "mm²")
    bar_pair(axes[1], keys, vols,  "Volume (est.)", "mm³")

    fig.tight_layout()
    path = os.path.join(OUT_DIR, "comparison_geometry.png")
    fig.savefig(path, dpi=150, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Figure 3: Bounding box dimensions ─────────────────────────────────────────
def fig_bbox(data):
    keys = list(data.keys())
    dims = {ax_: [data[k]["bounding_box"][ax_]["length"] for k in keys]
            for ax_ in ("x", "y", "z")}
    diag = [data[k]["bounding_box"]["diagonal"] for k in keys]

    fig, axes = plt.subplots(1, 4, figsize=(16, 5))
    fig.suptitle("Bounding Box Dimensions", fontsize=15, y=1.01)

    bar_pair(axes[0], keys, dims["x"], "Length X (nose→tail)", "mm")
    bar_pair(axes[1], keys, dims["y"], "Width Y (wingspan)",   "mm")
    bar_pair(axes[2], keys, dims["z"], "Height Z",             "mm")
    bar_pair(axes[3], keys, diag,      "Bounding Diagonal",    "mm")

    fig.tight_layout()
    path = os.path.join(OUT_DIR, "comparison_bbox.png")
    fig.savefig(path, dpi=150, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Figure 4: Radar / spider chart ───────────────────────────────────────────
def fig_radar(data):
    """Normalised radar chart across 6 key metrics."""
    keys    = list(data.keys())
    metrics = ["Triangles", "Vertices", "Surface\nArea", "Volume", "Length X", "Wingspan Y"]

    raw = {
        k: [
            data[k]["mesh"]["triangles"],
            data[k]["mesh"]["vertices"],
            data[k]["geometry"]["surface_area"],
            data[k]["geometry"]["volume"],
            data[k]["bounding_box"]["x"]["length"],
            data[k]["bounding_box"]["y"]["length"],
        ]
        for k in keys
    }

    # Normalise each metric to [0, 1] across the two models
    cols_T   = list(zip(*[raw[k] for k in keys]))
    normed   = {k: [] for k in keys}
    for col in cols_T:
        mn, mx = min(col), max(col)
        span   = mx - mn if mx != mn else 1
        for k, v in zip(keys, col):
            normed[k].append((v - mn) / span)

    n      = len(metrics)
    angles = [i * 2 * math.pi / n for i in range(n)] + [0]

    fig, ax = plt.subplots(figsize=(7, 7), subplot_kw=dict(polar=True))
    fig.patch.set_facecolor(BG)
    ax.set_facecolor("#16171F")

    for k in keys:
        vals = normed[k] + [normed[k][0]]
        ax.plot(angles, vals, color=COLORS[k], linewidth=2, label=LABELS[k])
        ax.fill(angles, vals, color=COLORS[k], alpha=0.18)

    ax.set_thetagrids([a * 180 / math.pi for a in angles[:-1]], metrics,
                      fontsize=11, color=TEXT_C)
    ax.set_yticklabels([])
    ax.grid(color=GRID_C, linewidth=0.6)
    ax.spines["polar"].set_color(GRID_C)
    ax.set_title("Normalised Metric Radar", pad=18, fontsize=14, color=TEXT_C)
    ax.legend(loc="upper right", bbox_to_anchor=(1.28, 1.12),
              facecolor="#22232E", edgecolor=GRID_C, labelcolor=TEXT_C)

    path = os.path.join(OUT_DIR, "comparison_radar.png")
    fig.savefig(path, dpi=150, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Figure 5: Combined overview panel ────────────────────────────────────────
def fig_overview(data):
    """Single figure with all key comparisons side by side."""
    keys  = list(data.keys())
    clrs  = [COLORS[k] for k in keys]
    xlbls = [LABELS[k] for k in keys]

    panels = [
        ("Triangles",           [data[k]["mesh"]["triangles"]                  for k in keys], "count"),
        ("Vertices",            [data[k]["mesh"]["vertices"]                   for k in keys], "count"),
        ("Surface Area (mm²)",  [data[k]["geometry"]["surface_area"]           for k in keys], "mm²"),
        ("Volume est. (mm³)",   [data[k]["geometry"]["volume"]                 for k in keys], "mm³"),
        ("Length X (mm)",       [data[k]["bounding_box"]["x"]["length"]        for k in keys], "mm"),
        ("Wingspan Y (mm)",     [data[k]["bounding_box"]["y"]["length"]        for k in keys], "mm"),
        ("Height Z (mm)",       [data[k]["bounding_box"]["z"]["length"]        for k in keys], "mm"),
        ("Bounding Diag (mm)",  [data[k]["bounding_box"]["diagonal"]           for k in keys], "mm"),
    ]

    fig, axes = plt.subplots(2, 4, figsize=(20, 10))
    fig.suptitle("Aircraft Model — Full Comparison", fontsize=17, y=1.01)
    axes = axes.flatten()

    for ax, (title, vals, unit) in zip(axes, panels):
        x    = np.arange(len(keys))
        bars = ax.bar(x, vals, color=clrs, width=0.5, edgecolor="none", zorder=3)
        ax.set_xticks(x)
        ax.set_xticklabels(xlbls, fontsize=11)
        ax.set_title(title, pad=6)
        ax.yaxis.grid(True, zorder=0)
        ax.set_axisbelow(True)
        for bar, val in zip(bars, vals):
            fmt = f"{val:,.0f}" if val >= 100 else f"{val:.2f}"
            ax.text(bar.get_x() + bar.get_width() / 2,
                    bar.get_height() * 1.02, fmt,
                    ha="center", va="bottom", fontsize=10, color=TEXT_C)

    legend_patches = [
        mpatches.Patch(color=COLORS[k], label=LABELS[k]) for k in keys
    ]
    fig.legend(handles=legend_patches, loc="lower center", ncol=2,
               facecolor="#22232E", edgecolor=GRID_C, labelcolor=TEXT_C,
               fontsize=12, bbox_to_anchor=(0.5, -0.03))

    fig.tight_layout()
    path = os.path.join(OUT_DIR, "comparison_overview.png")
    fig.savefig(path, dpi=150, bbox_inches="tight", facecolor=BG)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    data = load()
    print("Generating comparison charts...")
    fig_mesh(data)
    fig_geometry(data)
    fig_bbox(data)
    fig_radar(data)
    fig_overview(data)
    print("Done.")


if __name__ == "__main__":
    main()
