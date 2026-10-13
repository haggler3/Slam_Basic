"""Static figures for the README, from viewer/data/run.json and docs/results_seeds.json.

    python tools/make_plots.py            # writes docs/plots/*.png
"""
from __future__ import annotations

import base64
import json
import zlib
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "plots"
BG, FG, GRID = "#0b101c", "#e6ecff", "#26304a"
C = dict(odom="#ff4d6d", front="#9a86c9", online="#3dd9ff", opt="#9dffb0", drone="#ffb347", gt="#ffffff", loop="#ff4fd8")
plt.rcParams.update({"figure.facecolor": BG, "axes.facecolor": BG, "savefig.facecolor": BG, "text.color": FG,
                     "axes.labelcolor": FG, "xtick.color": FG, "ytick.color": FG, "axes.edgecolor": GRID,
                     "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "font.size": 10,
                     "axes.titlesize": 11, "legend.frameon": True, "legend.facecolor": BG, "legend.edgecolor": GRID, "legend.framealpha": 0.85})


def box_outline(s):
    hx, hy = s["size"][0], s["size"][1]
    a = np.radians(s.get("yaw", 0.0))
    R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    corners = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]]) @ R.T + np.array(s["pos"][:2])
    return corners


def draw_world(ax, D, alpha=0.5):
    for s in D["world"]["statics"]:
        if s["kind"] == "box":
            ax.add_patch(Polygon(box_outline(s), closed=True, fill=True, fc="#24304a", ec="#5b6b92", lw=0.6, alpha=alpha))
        else:
            ax.add_patch(plt.Circle(s["pos"][:2], s["size"][0], fc="#24304a", ec="#5b6b92", lw=0.6, alpha=alpha))
    a = D["world"]["arena"]
    ax.set_xlim(-a - 1, a + 1); ax.set_ylim(-a - 1, a + 1); ax.set_aspect("equal")


def error_vs_time(D):
    m, t = D["metrics"], np.array(D["t"])
    fig, axs = plt.subplots(2, 1, figsize=(9, 6.4), sharex=True)
    for ax, who, col in zip(axs, ("rover", "drone"), (C["online"], C["drone"])):
        ax.plot(t, m[f"{who}_odom_err"], color=C["odom"], lw=1.4, label=f"dead reckoning (RMSE {m[who + '_odom_rmse']:.2f} m)")
        ax.plot(t, m[f"{who}_front_err"], color=C["front"], lw=1.4, label=f"front-end odometry ({m[who + '_front_rmse']:.2f} m)")
        ax.plot(t, m[f"{who}_online_err"], color=col, lw=1.8, label=f"graph SLAM, online ({m[who + '_online_rmse']:.2f} m)")
        ax.plot(t, m[f"{who}_opt_err"], color=C["opt"], lw=1.2, ls="--", label=f"after final optimisation ({m[who + '_opt_rmse']:.2f} m)")
        for e in D[who]["graph"][:-1]:
            ax.axvline(t[min(e["f"], len(t) - 1)], color=C["loop"], lw=0.7, alpha=0.45)
        ax.set_yscale("log"); ax.set_ylim(0.01, 400); ax.set_ylabel("position error [m]")
        ax.set_title(f"{who.capitalize()}: pose error against ground truth (pink lines = loop-closure optimisations)", loc="left")
        ax.legend(loc="upper left", fontsize=8.5, ncol=2)
    axs[1].set_xlabel("time [s]")
    fig.tight_layout(); fig.savefig(OUT / "error_vs_time.png", dpi=150); plt.close(fig)


def trajectories(D):
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))
    r = {k: np.array(D["rover"][k]) for k in ("gt", "dr", "fe", "est", "opt")}
    ax = axs[0]; draw_world(ax, D)
    ax.plot(r["dr"][:, 0], r["dr"][:, 1], color=C["odom"], lw=1, label="dead reckoning")
    ax.plot(r["fe"][:, 0], r["fe"][:, 1], color=C["front"], lw=1.2, label="front-end odometry")
    ax.plot(r["gt"][:, 0], r["gt"][:, 1], color=C["gt"], lw=1.6, alpha=0.8, label="ground truth")
    ax.plot(r["opt"][:, 0], r["opt"][:, 1], color=C["opt"], lw=1.2, ls="--", label="graph SLAM (optimised)")
    g = D["rover"]["graph"][-1]; nodes = np.array(g["nodes"])
    for i, j in g["loops"]:
        ax.plot(*zip(nodes[i], nodes[j]), color=C["loop"], lw=1.2)
    ax.scatter(nodes[:, 0], nodes[:, 1], s=4, color="#d77bff", zorder=3, label=f"keyframe nodes ({len(g['loops'])} loop closures)")
    ax.set_title("Rover (top-down)", loc="left"); ax.legend(loc="lower left", fontsize=8)
    ax = axs[1]; draw_world(ax, D)
    d = {k: np.array(D["drone"][k]) for k in ("gt", "dr", "fe", "est", "opt")}
    ax.plot(d["dr"][:, 0], d["dr"][:, 1], color=C["odom"], lw=1, label="dead reckoning")
    ax.plot(d["fe"][:, 0], d["fe"][:, 1], color=C["front"], lw=1.2, label="front-end odometry")
    ax.plot(d["gt"][:, 0], d["gt"][:, 1], color=C["gt"], lw=1.6, alpha=0.8, label="ground truth")
    ax.plot(d["opt"][:, 0], d["opt"][:, 1], color=C["opt"], lw=1.2, ls="--", label="graph SLAM (optimised)")
    g = D["drone"]["graph"][-1]; nodes = np.array(g["nodes"])
    for i, j in g["loops"]:
        ax.plot(*zip(nodes[i][:2], nodes[j][:2]), color=C["loop"], lw=1.2)
    ax.scatter(nodes[:, 0], nodes[:, 1], s=4, color="#d77bff", zorder=3, label=f"keyframe nodes ({len(g['loops'])} loop closures)")
    ax.set_title("Drone (top-down)", loc="left"); ax.legend(loc="lower left", fontsize=8)
    fig.tight_layout(); fig.savefig(OUT / "trajectories.png", dpi=150); plt.close(fig)


def grid_at(D, frame):
    best = None
    for g in D["grids"]:
        if g["f"] <= frame:
            best = g
    n = D["meta"]["grid"]
    return np.frombuffer(zlib.decompress(base64.b64decode(best["z"])), np.uint8).reshape(n, n)


def map_before_after(D):
    half, n = D["meta"]["half"], D["meta"]["grid"]
    ev = [e for e in D["rover"]["graph"] if e["rebuilt"]]
    f0 = ev[0]["f"]
    frames = [("just before the first loop closure", f0 - 1), ("just after (map rebuilt from optimised poses)", f0),
              ("end of run", D["meta"]["n"] - 1)]
    fig, axs = plt.subplots(1, 3, figsize=(15, 5.2))
    for ax, (title, f) in zip(axs, frames):
        g = grid_at(D, f).astype(float)
        img = np.zeros((n, n, 3)); img[:] = np.array([0.043, 0.063, 0.11])
        free = (g > 0) & ((g - 1) / 254 < 0.4); occ = (g > 0) & ((g - 1) / 254 >= 0.62)
        img[free] = (0.06, 0.2, 0.27); img[occ] = (0.35, 0.92, 1.0)
        ax.imshow(img, origin="lower", extent=[-half, half, -half, half], interpolation="nearest")
        for s in D["world"]["statics"]:  # ground truth outlines on top: gaps = map error
            if s["kind"] == "box":
                ax.add_patch(Polygon(box_outline(s), closed=True, fill=False, ec="#ff4fd8", lw=0.5, alpha=0.8))
            else:
                ax.add_patch(plt.Circle(s["pos"][:2], s["size"][0], fill=False, ec="#ff4fd8", lw=0.5, alpha=0.8))
        a = D["world"]["arena"]; ax.set_xlim(-a - 1, a + 1); ax.set_ylim(-a - 1, a + 1); ax.set_aspect("equal"); ax.grid(False)
        ax.set_title(f"t = {D['t'][f]:.0f} s: {title}", fontsize=9.5, loc="left")
    fig.suptitle("Rover occupancy map (cyan) with true obstacle outlines (pink): loop closure removes the double walls", y=0.99)
    fig.tight_layout(); fig.savefig(OUT / "map_before_after.png", dpi=150); plt.close(fig)


def seeds_bar():
    R = json.loads((ROOT / "docs" / "results_seeds.json").read_text())
    seeds = sorted(R)
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
    for ax, who in zip(axs, ("rover", "drone")):
        keys = [("odom", "dead reckoning", C["odom"]), ("front", "front-end", C["front"]), ("online", "graph SLAM (online)", C["online"]),
                ("opt", "optimised", C["opt"])]
        w = 0.2
        for k, (key, lab, col) in enumerate(keys):
            v = [R[s][f"{who}_{key}_rmse"] for s in seeds]
            ax.bar(np.arange(len(seeds)) + (k - 1.5) * w, v, w, color=col, label=lab)
            for x, y in zip(np.arange(len(seeds)) + (k - 1.5) * w, v):
                ax.text(x, y * 1.08, f"{y:.2f}", ha="center", fontsize=7, rotation=90, va="bottom")
        ax.set_yscale("log"); ax.set_ylim(0.1, 40); ax.set_xticks(range(len(seeds))); ax.set_xticklabels([f"seed {s}" for s in seeds])
        ax.set_ylabel("position RMSE [m]"); ax.set_title(who.capitalize(), loc="left")
    axs[0].legend(fontsize=8, loc="upper right", ncol=2)
    fig.tight_layout(); fig.savefig(OUT / "seeds_rmse.png", dpi=150); plt.close(fig)


def loop_scores(D):
    """Graph size / loop closures over time."""
    fig, ax = plt.subplots(figsize=(9, 3.4))
    t = np.array(D["t"])
    for who, col in (("rover", C["online"]), ("drone", C["drone"])):
        ev = D[who]["graph"]
        ax.step([t[min(e["f"], len(t) - 1)] for e in ev], [len(e["loops"]) for e in ev], where="post", color=col, lw=2, label=who)
    ax.set_xlabel("time [s]"); ax.set_ylabel("loop-closure edges in graph"); ax.set_title("Loop closures accepted over time", loc="left")
    ax.legend(); fig.tight_layout(); fig.savefig(OUT / "loop_closures.png", dpi=150); plt.close(fig)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    D = json.loads((ROOT / "viewer" / "data" / "run.json").read_text())
    error_vs_time(D); trajectories(D); map_before_after(D); seeds_bar(); loop_scores(D)
    print("wrote", *sorted(p.name for p in OUT.glob("*.png")))


if __name__ == "__main__":
    main()
