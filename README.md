# SLAM Lab

**Graph SLAM** for a MuJoCo ground rover and quadrotor exploring a custom, dynamic world — with an interactive
Three.js viewer and a scripted film of the whole run.

![preview](docs/preview.png)

[![Highlight: the rover closes a loop and the map snaps into place](docs/slam_lab_highlight.gif)](docs/slam_lab.mp4)

▶ **Click the animation for the full 100 s film** ([`docs/slam_lab.mp4`](docs/slam_lab.mp4), 960×540, 6 MB; GitHub opens it in a player).
The animation above is a 12 s excerpt: the rover recognises a place it has seen before (pink ring), the pose graph is
re-optimised, and the error drops from decimetres to centimetres. Rendered by `tools/make_video.py`, then compressed with ffmpeg.

Both robots fly/drive through **The Foundry**, a 28 × 28 m industrial hall (octagonal rotunda, shelving aisles,
container yard with cluttered lanes, a gantry crane, a pillar row) shared with four moving obstacles. Each builds its
own map from simulated LiDAR and noisy odometry; the viewer shows the maps, the pose graphs, every loop closure, and
how the error changes when the back-end optimises.

## How the SLAM works

| | Rover (2D, SE(2)) | Drone (3D, SE(3)) |
|---|---|---|
| **Sensors** | 360° LiDAR (`mj_multiRay`) + wheel odometry (3.5 % scale error, yaw bias, noise) | 96×9-beam 3D LiDAR + body velocity / gyro (4 % scale error, biases, noise) |
| **Front-end** | particle filter + correlative scan matching against a *local, decaying* occupancy sub-map | point-to-plane ICP against a *local* voxel map (voxels seen in the last 12 s) |
| **Graph** | keyframe every 0.4 s, odometry edges | keyframe every 0.3 s, odometry edges |
| **Loop closure** | revisit candidates by distance in the current graph + ≥ 15 m of travel; multi-start 2D ICP between the two scans; accepted on inlier ratio, mutual overlap, RMS | revisit candidates by distance + ≥ 20 m of travel; multi-start 3D ICP (x/y/z/yaw offsets) against a sub-map around the old keyframe |
| **Back-end** | sparse Levenberg–Marquardt on the pose graph (Huber loss) | same, with SE(3) and a local rotation parametrisation |
| **Map** | log-odds occupancy grid, **rebuilt from the optimised poses** after a big correction | voxel map (voxels need repeat hits, are cleared when later rays pass through), rebuilt likewise |

The front-end is deliberately a *local* odometry: it is accurate over seconds and drifts over minutes. That is what
makes the graph back-end matter — when the rover returns to a place it has seen (~60 s in), ICP between the two scans
adds a constraint, the whole trajectory is pulled taut at once, and the map visibly snaps shut. The solver
(`slamviz/posegraph.py`) is implemented from scratch; its Jacobians are numerical, so it is slow-ish but short.

## Plots

All figures are generated from the committed run by `python tools/make_plots.py`.

**Pose error against ground truth over time.** Pink lines mark the moments the pose graph was re-optimised after a loop closure.
Dead reckoning drifts to metres; the front-end alone stays near a metre; graph SLAM drops to centimetres once loops close.

![error vs time](docs/plots/error_vs_time.png)

**Trajectories and the pose graph** (violet dots = keyframe nodes, pink = loop-closure edges):

![trajectories](docs/plots/trajectories.png)

**The rover's map before and after the first loop closure.** Pink outlines are the true obstacles, so any gap between cyan and pink is map error.

![map before and after](docs/plots/map_before_after.png)

**Three random seeds** (log scale) and **loop closures accepted over time**:

![seeds](docs/plots/seeds_rmse.png)
![loop closures](docs/plots/loop_closures.png)

## The math, at a high level

No heavy derivations here, just the ideas each part of the code is built on.

**1. Pose graph (the "graph" in graph SLAM).** Every keyframe is a node (a pose). Every measurement is an edge that says
"node j is *this* far from node i". Odometry gives edges between neighbouring keyframes; loop closures give edges between
places that were visited twice. The back-end finds the poses that disagree least with all edges at once, i.e. it minimises

```
sum over edges of   ‖ predicted relative pose − measured relative pose ‖²  /  σ²
```

(a robust Huber version, so one bad edge cannot wreck everything). It is a nonlinear least-squares problem, solved with
Levenberg–Marquardt on a sparse system. Rotations are updated by composing small rotations, so there is no angle wrap-around.

**2. Rover front-end: scan matching on an occupancy grid.**
- The map is a grid of log-odds (a number per cell: positive = probably occupied, negative = probably free). A LiDAR ray lowers the
  cells it passes through and raises the cell it hits.
- A pose is scored by how close the scan's end points land to occupied cells (a distance transform gives that distance), with a
  Gaussian penalty for straying from the odometry prediction.
- A particle filter proposes poses; an exhaustive local search around the best one polishes them.

**3. Drone front-end: ICP.** Iterative Closest Point aligns the new 3D scan to the local voxel map. Each map point has a surface
normal, and the cost is the squared distance of scan points *along those normals* ("point-to-plane"), minimised by repeatedly
solving a small 6×6 linear system (3 rotation + 3 translation unknowns). A light pull toward the inertial prediction keeps it stable.

**4. Loop closure.** When the robot is near an earlier keyframe (and has travelled far enough to make that meaningful), try ICP
between the two scans from many starting offsets, keep the best, and accept it only if enough points agree, the two scans overlap
in both directions, and the residual is small. The result becomes a new graph edge.

**5. Dead reckoning vs SLAM.** Dead reckoning adds up noisy velocity measurements, so error grows without bound. SLAM re-uses the
map to correct it. Quality is measured as the position RMSE against ground truth,

```
RMSE = sqrt( mean over time of  ‖ estimated position − true position ‖² )
```

**6. Rebuilding the map.** After the graph is corrected, every stored scan is replayed with its corrected pose into a fresh map,
which is why walls that looked doubled snap into one.

## Run it

```bash
pip install -r requirements.txt            # (matplotlib is only needed for tools/make_plots.py)
python -m slamviz.run                  # simulate + graph SLAM -> viewer/data/run.json  (~2 min)
python serve.py                        # open http://localhost:8000
python -m pytest tests                 # ~1 min
```

A pre-computed run (`--seed 0`) is committed, so `python serve.py` works straight away.
Options: `--duration 115 --seed N --map maps/your_map.json --out path.json`.

### Make the film

```bash
pip install -r requirements-video.txt && playwright install chromium     # or: export CHROMIUM=/path/to/chrome
python tools/make_video.py --out docs/slam_lab.mp4         # ~100 s film, 1280x720 @ 20 fps (needs ffmpeg on PATH)
# the committed film was then shrunk 27 MB -> 6 MB:
#   ffmpeg -i docs/slam_lab.mp4 -vf scale=960:540 -c:v libx264 -preset slow -crf 33 -pix_fmt yuv420p -movflags +faststart small.mp4
python tools/make_video.py --preview --out /tmp/p.mp4      # two frames per shot, to check the framing
```

Frames are rendered one by one (software WebGL, ~1.2 s per frame, so ~40 min for the full film) with scripted camera
paths and captions; the shot timing is derived from the actual loop-closure events in `run.json`.

## Your own map

Maps are plain JSON (`maps/foundry.json` is generated by `maps/make_foundry.py`, which also documents the schema):
boxes, cylinders and arbitrary-angle walls; movers following line / circle / spin motions; a rover route and a drone
route. Drop in a file and run `python -m slamviz.run --map maps/mine.json --out viewer/data/mine.json`, then open
`http://localhost:8000/?data=data/mine.json` — the viewer picks up everything (statics, movers, zone labels, arena
size) from the exported data. `tests/test_slam.py` builds a small map inline as an
example and runs the whole pipeline on it.

Design tip learned the hard way: long featureless parallel walls (corridors) make LiDAR odometry slide. The Foundry's
container lanes have crates in them for that reason; without them the drone front-end drifted by 2 m or more.

## Viewer

* **Space** play/pause · **← →** step · drag the timeline to scrub (maps and graphs rewind correctly)
* **1–5** camera: orbit / rover chase / drone chase / top-down / cinematic
* Layers: ground truth, occupancy map, voxel map (coloured by height), pose graph + loop closures (violet / pink; a ring
  flashes where a loop is closed), LiDAR rays, particles, truth + online estimate, front-end odometry, raw dead
  reckoning, the final optimised trajectory, zone labels.
* The panel plots pose error (log scale) of raw odometry, front-end, online graph SLAM and the optimised trajectory.

Everything drawn is the **estimated** map, so a mismatch with the translucent ground-truth world is real SLAM error.

## Results

Position RMSE in metres over 115 s (`python -m slamviz.run --seed N`):

| seed | rover: odometry → front-end → graph (online) → optimised | loops | drone: odometry → front-end → graph (online) → optimised | loops |
|---|---|---|---|---|
| 0 | 8.21 → 0.89 → 0.66 → 0.30 | 20 | 7.31 → 0.26 → 0.21 → 0.22 | 7 |
| 1 | 9.11 → 0.95 → 0.54 → 0.23 | 20 | 6.84 → 0.68 → 0.35 → 0.51 | 7 |
| 2 | 8.27 → 0.94 → 0.65 → 0.24 | 16 | 6.99 → 0.26 → 0.21 → 0.21 | 7 |

*Online* is what a robot would know at each moment; *optimised* is every pose in the history after the final
optimisation. Accepted loop closures were compared with ground truth for seeds 0–2: none was wrong (rover: worst
0.12 m / 0.9°; drone: all within 0.3 m / 3°).

### Honest limitations

* The rover benefits a lot from graph optimisation. The drone's front-end is already good, so the gain is small;
  on seed 1 the *optimised* history is **worse** than the online estimate (0.51 vs 0.35 m): a height slip happened in
  one lane, and the optimiser spreads the correction evenly over all odometry edges (they share one covariance), which
  slightly damages the accurate early part. Per-edge covariances from scan-match quality would fix this; not done.
* The drone finds loop closures only late in the run (~83 s) because its route revisits few places.
* Loop-closure verification is geometric only (no place-recognition descriptor), so very repetitive scenes could fool it.
  The pillar row is irregular on purpose.
* "Wheel odometry" and the drone's velocity are derived from simulator ground truth plus an error model; there is no
  wheel/IMU physics model.
* Not tested on a real GPU or in a non-headless browser; the film is software-rendered.

## Layout

```
maps/                  foundry.json (+ the script that writes it and documents the schema)
slamviz/world.py       map loader, MJCF generation
slamviz/sim.py         MuJoCo stepping, sensors, noisy odometry
slamviz/control.py     rover reactive navigation, drone geometric controller
slamviz/posegraph.py   SE(2)/SE(3) pose-graph optimiser
slamviz/occgrid.py     log-odds occupancy grid
slamviz/rover_slam.py  front-end, keyframes, loop closure, graph back-end (2D)
slamviz/drone_slam.py  same in 3D, voxel map
slamviz/run.py         runs everything, exports JSON
viewer/                static Three.js app (three.js vendored, no build step)
tools/make_video.py    frame-exact video renderer
tools/make_plots.py    README figures from the run data
tests/                 solvers, scan matching, custom map, end-to-end graph SLAM
```
