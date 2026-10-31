# SPEC — scope statement (written before any code)

This file fixes the scope of the project. Anything not listed under "In scope" is not
claimed. Numbers below are the *design commitments*; the authoritative values live in the
config files named next to them. If a number changes during development, this file is
updated in the same commit.

## 1. Platform

| Item | Choice |
|---|---|
| ROS 2 distro | **Humble Hawksbill** (Ubuntu 22.04 userspace, inside Docker) |
| Simulator | **Gazebo Classic (gazebo11)** via `gazebo_ros_pkgs` (`ros-humble-gazebo-ros-pkgs`, Gazebo 11.10.2) |
| Language | C++17 (ament_cmake) for all nodes; Python only for launch files and result tooling |
| Run mode | **Headless** (`gzserver` only) for all development, CI and all measured episodes |
| Time | `use_sim_time:=true` for every node, asserted in `sim.launch.py` |

Everything in this repository is **simulated in Gazebo**. Nothing here is validated on real
hardware.

## 2. Robot (`ros2_ws/src/robot_description`)

* Differential drive, base radius 0.16 m, wheel radius 0.05 m, wheel separation 0.28 m.
* 2D lidar: **360 samples, 10 Hz**, 1.0° increment, range 0.12–6.0 m, Gaussian noise σ = 0.005 m, frame `laser`.
* RGB camera: 320×240, 10 Hz, frame `camera`. It is part of the URDF and TF tree, but the
  Gazebo camera *sensor* is only enabled for rendered (GIF) runs (`enable_camera:=true`),
  because camera sensors need an OpenGL context and the evaluation is headless by
  requirement. **No algorithm in this stack consumes camera data** (perception is lidar-only).
* Velocity limits (single source of truth: `robot_description/config/robot_limits.yaml`,
  read by the URDF and by the controller): **|v| ≤ 0.5 m/s, |ω| ≤ 1.2 rad/s**.
* Ground-truth odometry: `/odom` comes from the Gazebo diff-drive plugin in *world*
  odometry mode, so `odom` coincides with the Gazebo world frame. `map → odom` is a static
  identity transform. There is **no localization** in this stack.

TF tree: `map → odom → base_link → {laser, camera, left_wheel, right_wheel, caster_*}`.

## 3. Worlds (`worlds/`)

Both worlds share a 12 m × 12 m walled arena (inner faces at ±6 m) and are generated
deterministically from one script (`worlds/gen_worlds.py`) that also writes a geometry
sidecar (`*.obstacles`) used by the evaluation (start/goal sampling, ground-truth
clearance). Generated files are committed; `world_stats.json` holds the measured density
and minimum corridor width.

| World | Intent |
|---|---|
| `simple.world` | sparse: 8 obstacles, ≈ 2 % of the arena covered, minimum corridor ≈ 1.7 m |
| `cluttered.world` | dense: 30 obstacles, ≈ 7 % covered, minimum corridor ≈ 0.8 m (measured values in `worlds/world_stats.json`) |
| `empty.world` | no obstacles; used only by the Gazebo-in-the-loop controller check |

World geometry and packing: the cluttered world was initially tested with 40 obstacles and ≥ 1.0 m corridors; geometric bounds in a 12 × 12 m arena limit maximum non-overlapping placement (generator placed 31/40), so the environment is settled at 30 obstacles and ≥ 0.8 m corridors (0.8 m minus twice the 0.30 m inflation leaves 0.2 m clearance for the robot center).

## 4. Algorithms (all implemented in this repository)

| Component | Package / file | Notes |
|---|---|---|
| Perception | `perception/src/obstacle_detector.cpp` | `/scan` → `/obstacles` (PoseArray, cluster centroids in `map`) and `/costmap` (OccupancyGrid built by the project with a known-pose log-odds ray-cast mapper; **not SLAM**, poses come from ground-truth TF). Scan is resampled to a fixed **2.0°** resolution (180 beams). |
| Planner | `navigation/src/astar_planner.cpp` | **A\*** written for this project: 8-connected, octile heuristic, no corner cutting, hard inflation radius **0.30 m**, optional line-of-sight shortcutting. Unknown cells are treated as free (the map is built online) and the plan is recomputed at 2 Hz. |
| Controller | `navigation/src/pure_pursuit.cpp` (algorithm), `control/src/waypoint_follower.cpp` (node) | **Pure pursuit** written for this project, configurable lookahead (default 0.6 m), clamped velocities. |
| nav2 | **not used** | nav2 is not used anywhere, not even as a comparison mode. |

## 5. Evaluation protocol (`ros2_ws/src/evaluation`)

* **20 episodes per world** (`simple`, `cluttered`) = 40 episodes, numbered 1..20; the episode clock starts when the goal is
  published, after the robot has been placed and 1.5 s of simulated time has been spent on fresh mapping (the costmap is reset
  for every episode).
* Each episode has a recorded integer **seed** (`seed = seed_base + episode`; default
  `seed_base` 1000 for both worlds). The seed alone determines the start pose, goal pose and
  the order of rejection sampling, so episode *k* with seed *s* reproduces the same poses.
* Start/goal sampling: uniform in the arena, ≥ 0.5 m from every obstacle surface, start–goal
  Euclidean distance ≥ 5.0 m, and **reachable** for the ground-truth inflated grid (checked
  with the same A\* code). Episodes are therefore feasible by construction; failures are
  failures of the stack, not of the task.
* Success: robot center within **0.3 m** of the goal position.
* Timeout: hard cap 120 s of *sim* time. After 5 successful episodes in a world run, the cap
  becomes `min(120, max(30, 3 × mean success time))` ("3× the mean episode time" rule). A
  timed-out episode is logged with seed, start and goal in `results/limits_report.txt`.
* A node crash during an episode marks that episode `timed_out=true` (no extra column; the
  `crashed` flag lives in `results/episode_diagnostics.csv`), logs node + stderr to
  `results/crash_log.txt`, restarts the stack and continues. Crashed episodes stay in the
  denominator.
* Collision: contact reported by the Gazebo contact sensor on the chassis, debounced
  (rising edges separated by ≥ 0.5 s). `min_obstacle_distance_m` is the minimum, over the
  episode, of the ground-truth distance from the robot center to the nearest obstacle
  surface (walls included), from the world sidecar geometry.

### CSV schemas (exact)

`results/episodes.csv`:

```
world,episode,seed,success,time_to_goal_s,path_length_m,collision_count,min_obstacle_distance_m,timed_out
```

`results/summary.csv` — benchmark summary schema:

```
world,n_episodes,success_rate,mean_time_s,std_time_s,mean_path_m,collision_count_total,timeout_count,mean_rtf
```

Columns 1–8 represent aggregate navigation performance per environment, while `mean_rtf` (real-time factor) tracks physics execution rate relative to real time.

Conventions: booleans are `true`/`false`. `time_to_goal_s` is `nan` for episodes that did
not reach the goal; `mean_time_s`/`std_time_s` are over *successful* episodes only (sample
std, n−1); `mean_path_m` is over all episodes (distance actually driven); success rate and
timeout count use all episodes as denominator. Per-episode RTF is stored in
`results/rtf_log.csv` and averaged by the summarizer.

## 6. Out of scope (deliberately — not bugs)

SLAM; localization beyond ground-truth odometry; dynamic obstacles; multi-robot; real
hardware; learning-based control; camera-based perception; nav2 (as planner or comparison).

## 7. Container & Build Environment

The stack runs inside an Ubuntu 22.04 container with ROS 2 Humble and Gazebo 11. All dependencies are pre-installed in the Docker image to ensure isolated, reproducible builds.

## 8. Acceptance Criteria

- All nodes compile with zero warnings/errors via `colcon build --symlink-install`.
- All automated unit tests in `colcon test` pass cleanly.
- Headless simulation launch completes and all core nodes are responsive within 30 s.
- 40/40 benchmark episodes complete across simple and cluttered environments without uncaught exceptions.
