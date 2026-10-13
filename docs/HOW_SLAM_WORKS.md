# How graph SLAM works (ASCII walkthrough)

Notation: `x_k` = robot pose at keyframe k, `u_k` = odometry measurement between `x_k` and `x_{k+1}`,
`l_n` = loop-closure measurement (a revisit), `z` = LiDAR scan taken at a pose.

## 1. Odometry builds a chain

Each keyframe is a node. Each odometry measurement is an edge saying "the next pose is this far from the last one".

```
  x0 ──u0──▶ x1 ──u1──▶ x2 ──u2──▶ x3 ──u3──▶ x4 ──u4──▶ x5 ──u5──▶ x6
  │          │          │          │          │          │          │
  z0         z1         z2         z3         z4         z5         z6      (LiDAR scans)
```

## 2. The chain drifts

Every `u_k` has a small error (wheel slip, scale error, gyro bias). Errors add up, so the *estimated* path (dashed)
walks away from the *true* path (solid). Here the robot drives a square and should end where it started:

```
   true path                         estimated path (from odometry only)

   x0 ─────▶ x1                      x0 ─────▶ x1
   ▲          │                      ▲          ╲
   │          ▼                      ╎           ╲
   x6 ◀───── x2                      ╎            x2 ╌╌╌╌▶ ...
   (x6 = x0: same place!)            ╎
                                     x6 ╌╌╌╌╌╌╌ ends up HERE, away from x0

   error grows with distance travelled: after a lap, metres off
```

## 3. A loop closure adds one extra edge

Back at the start, the LiDAR scan `z6` looks like the scan `z0` from the beginning. ICP aligns the two scans and
returns *how far apart the two poses really are* (almost zero). That is a new edge `l`, linking the present to the past:

```
          ┌───────────────────────── l ─────────────────────────┐
          │      "x6 is at x0 (± 3 cm), I recognise this place"  │
          ▼                                                      ▼
  x0 ──u0──▶ x1 ──u1──▶ x2 ──u2──▶ x3 ──u3──▶ x4 ──u4──▶ x5 ──u5──▶ x6
  │                                                                 │
  z0 ════════════════ scans overlap ══════════════════════════════ z6
```

Now the graph has a cycle: two paths from `x0` to `x6` (the long chain, and the loop edge) that disagree.

## 4. Optimisation squeezes the error out of the whole chain

The back-end finds all poses at once that best satisfy *every* edge, each weighted by its trust (σ). The
disagreement is spread along the chain instead of being left at the end:

```
  before:   x0 ─u0─ x1 ─u1─ x2 ─u2─ x3 ─u3─ x4 ─u4─ x5 ─u5─ x6       gap between x6 and x0 = 8 m
                                                                         ╲ l says gap should be ~0
  after:    x0 ─u0'─ x1' ─u1'─ x2' ─u2'─ x3' ─u3'─ x4' ─u4'─ x5' ─u5'─ x6' ≈ x0
            (every u_k stretched/rotated a little, so the sum closes)
```

Cost being minimised (sum over all edges, with a robust Huber loss):

```
  Σ_k ‖ pose(x_{k+1}) ⊖ pose(x_k) − u_k ‖²_σ   +   Σ_n ‖ pose(x_j) ⊖ pose(x_i) − l_n ‖²_σ
        └─────── odometry edges ───────┘          └───────── loop-closure edges ─────────┘
```

## 5. Many loop closures

After the first revisit, the robot keeps passing near old keyframes, so more `l_n` edges appear and each tightens the graph:

```
  x0 ─u0─ x1 ─u1─ x2 ─u2─ x3 ─u3─ x4 ─u4─ x5 ─u5─ x6 ─u6─ x7 ─u7─ x8 ─u8─ x9 ...
  └───────────────────────────────── l1 ───────────┘     (x6 ~ x0)
       └──────────────────────────────────── l2 ──────────────┘   (x7 ~ x1)
            └──────────────────────────────────────── l3 ────────────────┘ (x8 ~ x2)
```

## 6. One full cycle, as this repo does it

```
  LiDAR scan z_k + odometry u_k
          │
          ▼
  ┌─────────────────┐   local, drifts over minutes
  │   front-end     │   rover: scan matching on a decaying occupancy grid
  │ (odometry)      │   drone: ICP against a local voxel map
  └────────┬────────┘
           │ relative pose between keyframes  ──▶  odometry edge u_k
           ▼
  ┌─────────────────┐
  │ keyframe graph  │   x0 ─u0─ x1 ─u1─ x2 ...
  └────────┬────────┘
           │ near an old keyframe and far enough along the path?
           ▼
  ┌─────────────────┐   many starting offsets, ICP between the two scans
  │ loop detection  │   accept only if: enough points agree, overlap both
  │ + verification  │   ways, small residual   ──▶  loop edge l_n
  └────────┬────────┘
           ▼
  ┌─────────────────┐   sparse Levenberg–Marquardt, Huber loss
  │  optimise graph │   ──▶ corrected x0..xk
  └────────┬────────┘
           ▼
  ┌─────────────────┐   replay every stored scan with its corrected pose
  │ rebuild the map │   ──▶ doubled walls snap into one
  └─────────────────┘
```

## 7. Real examples from the committed run (seed 0)

**Rover, first loop closure at t = 59.6 s.** The rover finishes its first lap and returns to where it started.

```
  keyframes   x0 ─u0─ x1 ─ ... ─ x120 ─ ... ─ x148          (a keyframe every 0.4 s)
                 │                               │
                 └────────────── l1 ─────────────┘          ICP: inlier 0.8, overlap 0.8
  position error  0.70 m  ───────────────────────▶  0.03 m  (one optimisation)
  map             walls drawn twice, ~0.7 m apart ──▶ single walls
```

By the end of the run the rover has accepted 20 loop closures `l1 … l20`, and the final optimisation brings its
RMSE from 8.2 m (wheel odometry only) to 0.30 m.

**Drone, first loop closure at t = 83 s.** The drone flies back through the rotunda it crossed at about t = 9 s.

```
  x(t=9s) ─── ... ~75 s of flight, 4 m path of drift correction needed ... ─── x(t=83s)
     └──────────────────────────────── l1 ───────────────────────────────────────┘
                      3D ICP against a sub-map around the old keyframe
```

The drone's front-end was already accurate (0.26 m RMSE), so its 7 loop closures improve it only a little
(0.21 m online). Loop closures matter most when the front-end drifts, as the rover's does.

## 8. What can go wrong

```
  wrong loop closure ("this corridor looks like that one")   ──▶  Huber loss + strict verification
  robot stands still, so every pose looks like a revisit      ──▶  require ≥ 15–20 m of travel between the pair
  front-end slides along a featureless corridor               ──▶  crates in the lanes of the map
  drift too big for ICP to find the match                     ──▶  wider multi-start search (± 2 m, ± 0.2 rad)
```
