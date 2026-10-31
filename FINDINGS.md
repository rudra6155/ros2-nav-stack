# FINDINGS

> **Simulated in Gazebo Classic (gazebo11), ROS 2 Humble.**
> Detailed breakdown of evaluation runs, sensitivity sweeps, edge cases, and failure modes across 40 seeded episodes.

## 1. Worlds and their difficulty

| world | obstacles (cyl / box) | obstacle area fraction | min corridor width [m] | generator seed |
|---|---|---|---|---|
| simple | 8 (7 / 1) | 0.0177 | 1.744 | 11 |
| cluttered | 30 (23 / 7) | 0.072 | 0.819 | 23 |

Both worlds are 12 m × 12 m walled arenas; obstacles are 0.5 m tall (above the lidar plane), generated deterministically by `worlds/gen_worlds.py`
(`--check` verifies that the committed files are exactly what the generator produces). Only the obstacle layout differs.

* **Corridor width** is the smallest surface-to-surface gap between any two obstacles or an obstacle and a wall. The robot has a radius of 0.16 m and the
  planner inflates obstacles by 0.30 m, so the narrowest passage of the simple world (1.744 m) leaves 1.14 m of free width for
  the robot's *centre*, and that of the cluttered world (0.819 m) leaves 0.22 m — passable, but only just.
* **Density:** the cluttered world has about 30 obstacles covering 0.072 of the arena (simple: 8,
  0.0177). That is "dense" relative to the simple world, but it is not extreme — the result below should be read with that in mind.
* Start and goal are sampled with a recorded seed, ≥ 0.5 m from every obstacle, ≥ 5 m apart, and **reachable** on the ground-truth grid (same A\* and inflation as the planner),
  so a failure is a failure of the stack, never of an impossible task.

## 2. Results

| world | episodes | success rate | time to goal [s] (mean ± std, successes) | path [m] (mean) | collisions (total) | timeouts | mean RTF |
|---|---|---|---|---|---|---|---|
| simple | 20 | 100% (20/20) | 17.1 ± 4.8 | 7.57 | 0 | 0 | 1.00 |
| cluttered | 20 | 100% (20/20) | 18.0 ± 4.4 | 8.05 | 0 | 0 | 1.00 |

* **Reading 20 / 20 correctly.** With 20 episodes per world, an observed 100% success rate is compatible with a true success rate as low as
  83% (simple) and 83% (cluttered) — exact two-sided 95 % Clopper–Pearson lower bounds. It is evidence that the stack works in these worlds, not evidence
  that it never fails.
* Time and path are comparable between the worlds (the cluttered world is only slightly slower): the stack has headroom at this clutter level. Real-time factor: 1.00 / 1.00
  (the physics is capped at 1.0, so the RTF is expected to sit just below 1; the lowest per-episode RTF was 0.97, so nothing was logged to `results/limits_report.txt`).
* **Identical runs are not identical.** The same five episodes were run twice with the same code and seeds (the official run and the 0.6 m row of the lookahead sweep):

| ep | seed | time, official run [s] | time, sweep re-run [s] | path, official [m] | path, re-run [m] | min obstacle distance, official [m] | min obstacle distance, re-run [m] |
|---|---|---|---|---|---|---|---|
| 1 | 1001 | 14.0 | 14.0 | 6.23 | 6.23 | 0.26 | 0.26 |
| 2 | 1002 | 12.6 | 12.6 | 5.23 | 5.23 | 0.18 | 0.18 |
| 3 | 1003 | 22.9 | 22.9 | 10.85 | 10.85 | 0.33 | 0.32 |
| 4 | 1004 | 24.4 | 24.1 | 10.95 | 10.88 | 0.26 | 0.26 |
| 5 | 1005 | 19.1 | 18.8 | 8.76 | 8.70 | 0.30 | 0.29 |

  The runs agree closely but not exactly: the largest differences between the two runs are 0.3 s in time to goal, 0.07 m in path length and 0.01 m in the closest approach to an obstacle
  (Gazebo's sensor noise and thread timing are not bit-reproducible; only the *start/goal sampling* is). Differences of that size between two configurations should not be read as an effect.

## 3. Failure-mode analysis

### 3.1 All 40 episodes, categorized

Rules (`scripts/failure_modes.py`, applied in this order): **success** = reached the goal; **crash** = a node/the simulator died during the episode; **collision** = failed with
≥ 1 chassis contact; **stuck** = failed without contact but standing still for ≥ 15 s in a row; **timeout** = every other failure (budget exhausted while moving).

| world | success | timeout | collision | stuck | crash | total |
|---|---|---|---|---|---|---|
| simple | 20 | 0 | 0 | 0 | 0 | 20 |
| cluttered | 20 | 0 | 0 | 0 | 0 | 20 |
| all | 40 | 0 | 0 | 0 | 0 | 40 |

**No official episode failed**: 40 of 40 reached the goal, with 0 collisions, 0 timeouts and 0 crashes. So there is no failure in the
official run to root-cause, and I am not going to invent one. Every episode (seed, poses, result, closest approach, category) is listed in the appendix at the end of this file.
Failure behaviour is therefore studied in two other ways: the *near-failures* of the official run (§3.2) and *deliberate stress* (§3.3 parameter extremes, §3.4 fault injection).

### 3.2 Weakest behaviour in the official run: less clearance than the inflation radius promises

The six closest approaches (ground-truth distance from the robot's centre to the nearest obstacle surface; the robot's radius is 0.16 m, so contact would be at 0.16 m):

| world | ep | seed | closest approach to an obstacle surface [m] | at t [s] after the goal | robot speed there [m/s] | distance from start [m] |
|---|---|---|---|---|---|---|
| cluttered | 2 | 1002 | 0.18 | 3.4 | 0.50 | 0.8 |
| cluttered | 15 | 1015 | 0.19 | 12.7 | 0.47 | 5.4 |
| cluttered | 13 | 1013 | 0.23 | 16.1 | 0.49 | 6.6 |
| cluttered | 6 | 1006 | 0.26 | 17.0 | 0.46 | 7.8 |
| cluttered | 1 | 1001 | 0.26 | 4.7 | 0.50 | 1.8 |
| cluttered | 4 | 1004 | 0.26 | 4.4 | 0.48 | 1.2 |

The closest, cluttered episode 2 (seed 1002), passed at 0.18 m: only 0.02 m of daylight to the chassis, although the planner inflates obstacles by 0.30 m.
Where did the margin go? Evidence from the recorded rosbags (`scripts/analyze_first_seen.py`, `scripts/analyze_plan_vs_robot.py`; a window of a few seconds around each closest approach):

| quantity | cluttered ep 2 (seed 1002) | ep 15 (seed 1015) | ep 13 (seed 1013) |
|---|---|---|---|
| lidar's first return on that obstacle, seconds after the goal was published | -0.18 | -0.17 | 6.63 |
| lidar return nearest to the obstacle surface point closest to the plan [m] | 0.002 | 0.002 | 0.002 |
| largest distance between the robot and the plan it follows [m] | 0.058 | 0.115 | 0.048 |
| smallest clearance of the *plan* over the next 1.5 m [m] | 0.185 | 0.123 | 0.216 |
| …counting only plan points ≥ 0.35 m ahead of the robot [m] | 0.247 | 0.142 | 0.216 |
| distance from the robot to the plan point that is closest to an obstacle [m] | 0.042 | 0.136 | 0.043 |

What this does and does not show:

1. **Not late detection (my first hypothesis, refuted).** The obstacle was already in the lidar's view when the goal was published (episodes 2 and 15) or 9.51 s before the closest approach (episode 13), and the surface point nearest to the plan had a lidar return within
   0.002 m of it. So "unknown cells count as free" is not what happened here.
2. **Not the controller.** The robot stays within 0.058–0.115 m of the plan it follows.
3. **The planner's own output has less clearance than its nominal 0.30 m.** Two mechanisms are visible in the data (correlational — I did not run an ablation that removes either):
   * *Grid effects.* Even far from the robot the plan keeps only 0.247 / 0.142 / 0.216 m (episodes 2 / 15 / 13): the inflation is measured on a
     0.10 m grid between cell centres, an occupied cell can be up to ~0.07 m off the true surface, a path point up to ~0.07 m off its cell centre, and the line-of-sight smoother only tests whole cells.
   * *The start-relaxation bubble.* A\* deliberately lifts the inflation within 0.30 m of the robot (so it can leave a band it stands in) and treats cells within the robot radius as free. In episodes 2 and 15 the plan point closest to an obstacle is
     0.042 m and 0.136 m from the robot — inside that bubble — and the plan's minimum clearance there drops to 0.185 m and
     0.123 m. Episode 13 does not fit this (its minimum is outside the bubble), so the bubble is not the whole story.

*Consequence:* the planner's effective safety margin is smaller than its nominal inflation radius. It held for 40/40 episodes, but with as little as 0.02 m of daylight.

### 3.3 Failure modes under stress (parameter extremes; 5 identical episodes each)

These are not the official results; they are the parameter sweeps of §4, chosen to bracket the nominal value. Every sweep episode that failed or touched an obstacle:

| run | ep | seed | success | collisions | stop reason | path [m] | final distance to goal [m] | longest standstill [s] | min obstacle distance [m] |
|---|---|---|---|---|---|---|---|---|---|
| inflation = 0.2 | 1 | 1001 | yes | 1 | goal_reached | 6.15 | 0.30 | 0.0 | 0.16 |
| inflation = 0.2 | 2 | 1002 | yes | 2 | goal_reached | 5.41 | 0.30 | 0.2 | 0.16 |
| inflation = 0.2 | 3 | 1003 | yes | 1 | goal_reached | 10.73 | 0.30 | 0.0 | 0.16 |
| inflation = 0.2 | 4 | 1004 | yes | 1 | goal_reached | 10.86 | 0.30 | 0.0 | 0.15 |
| lookahead = 1.2 | 2 | 1002 | yes | 1 | goal_reached | 5.39 | 0.30 | 0.2 | 0.16 |
| lookahead = 1.2 | 4 | 1004 | no | 1 | hard_timeout | 1.29 | 9.86 | 115.5 | 0.16 |

**Failure mode A — a long pure-pursuit lookahead (1.2 m) makes the robot touch an obstacle it clears at 0.6 m.** In the sweep, 2 of 5 episodes made contact at lookahead 1.2 m (and none at 0.3 m or 0.6 m); with n = 5 that count alone is weak evidence.
The stronger evidence is a paired repetition of the one episode that failed (cluttered episode 4, seed 1004; `scripts/ablation_ep4.sh`): at lookahead 0.6 m **0 of 3 runs** touched an obstacle (closest approach 0.26 m);
at 1.2 m **3 of 3 runs** did, at the same place, 4.30 s into the episode. The most likely mechanism is the chord geometry of pure pursuit: the lookahead point 1.2 m ahead lies beyond the corner of an
obstacle and the arc towards it cuts inside the corner (I did not instrument the chord itself; in the recording the robot's distance from the plan reaches 0.119 m, similar to the 0.115 m seen without contact at the nominal lookahead, so the deviation alone does not separate the cases —
it matters on a plan that already runs close to the obstacle, see §3.2).

**Failure mode B — no recovery behaviour: once in contact, the stack stays stuck.** In the recording of that episode the first chassis contact happened at (x, y, yaw) = (-2.31, 4.25, -1.9); during the 40.80 s observed afterwards the robot moved
0.02 m in total, while the controller kept commanding forward motion in 0.93 of its messages and the planner kept publishing non-empty plans (fraction 0.96). The plan it was following passed
0.139 m from the obstacle, less than the robot's radius of 0.16 m — consistent with `astar.cpp`, which treats every cell within the robot radius of the robot as free, so a touching robot is told to carry on along the obstacle.
Nothing reacts to the contact sensor and the controller never reverses, hence "stuck" until the time budget ends (longest standstill in the original sweep run: 115.5 s, 1.29 m driven).

**Failure mode C — too little inflation spends the margin (inflation 0.2 m).** All five episodes still reached the goal (100%), but 4 of them touched an obstacle on the way
(5 contacts in total vs 0 at the nominal 0.3 m). This is the clearest example of why success rate alone is not enough.
With the grid effects of §3.2 a 0.2 m inflation leaves no margin over the 0.16 m body radius.

### 3.4 Fault injection (three deliberate faults, run in the simulator)

| fault | what happened (measured) | mitigation / detection in this repo |
|---|---|---|
| **Lidar frame id changes** (plugin stamps `laser_link`, URDF link is `laser`) | Every scan is dropped by the TF message filter: `/costmap` never gets a known cell (max known cells **0**, 26 "dropping message" warnings in the launch log), so A* plans through obstacles it cannot see. Episode 2 (seed 1002): success **no**, collisions **1**, stop reason `hard_timeout` (baseline run of the same episode: success yes, collisions 0). | `scripts/check_frames.sh` fails if `/scan` is not stamped exactly `laser` and exists in the URDF; the detector logs the dropped scans. |
| **Goal inside an obstacle / outside the map** | Planner publishes an empty `/plan` (0 non-empty plans for a goal in the wall, 0 for a goal outside the map), the robot did not move (max commanded linear speed 0.00 m/s, displacement 0.00 m), warnings: *goal is inside an obstacle, goal is outside the map*; planner still alive afterwards: **yes**. | Empty path + throttled warning by design (never a straight line); unit-tested in `navigation/test/test_astar.cpp`. |
| **Real-time factor drops** (CPU saturated by 12 busy loops on 4 cores, the stack at nice +15) | Episode 2: RTF 1.00 → **0.27**, success yes, time to goal 28.460 s → 28.460 s (sim), path 12.61 → 12.62 m, collisions 0. Episode 8: RTF 1.00 → **0.32**, success yes, time 23.940 s → 23.900 s, path 11.46 → 11.47 m, collisions 0. | All periodic work runs on ROS-clock timers and the stack consumes `/clock`, so a slower simulation changes wall time, not sim-time behaviour; RTF is logged per episode (`rtf_log.csv`, `limits_report.txt` below 0.3). |

## 4. Parameter sensitivity

Same five episodes (seeds 1001–1005 of the cluttered world, identical start/goal pairs for every value), one parameter changed at a time, everything else nominal.

### Lookahead distance (pure pursuit)

| lookahead [m] | world | episodes | success rate | mean time [s] | mean path [m] | collisions | timeouts | mean min obstacle distance [m] |
|---|---|---|---|---|---|---|---|---|
| 0.3 | cluttered | 5 | 100% | 18.6 | 8.37 | 0 | 0 | 0.27 |
| 0.6 | cluttered | 5 | 100% | 18.5 | 8.38 | 0 | 0 | 0.26 |
| 1.2 | cluttered | 5 | 80% | 19.1 | 6.54 | 2 | 1 | 0.21 |

Note on the 1.2 m row: its lower mean path and time are an artefact of the stuck episode. Mean path is over *all* episodes (SPEC.md), and episode 4 only drove 1.29 m before it stopped; time is the mean over the four episodes that finished.

### Inflation radius (A\*)

| inflation radius [m] | world | episodes | success rate | mean time [s] | mean path [m] | collisions | timeouts | mean min obstacle distance [m] |
|---|---|---|---|---|---|---|---|---|
| 0.2 | cluttered | 5 | 100% | 20.1 | 8.34 | 5 | 0 | 0.16 |
| 0.3 | cluttered | 5 | 100% | 18.7 | 8.40 | 0 | 0 | 0.26 |
| 0.4 | cluttered | 5 | 100% | 18.6 | 8.46 | 0 | 0 | 0.32 |
| 0.5 | cluttered | 5 | 100% | 27.5 | 11.39 | 0 | 0 | 0.35 |

**Tradeoffs, as measured:**

* *Lookahead.* 0.3 m and 0.6 m are indistinguishable in success and collisions on these episodes (mean time 18.56 s vs 18.48 s); 1.2 m is the only value that
  produced failures (2 contacts, 1 timeout), most likely because it cuts corners (§3.3 A). The usual expectation is that a short lookahead tracks tightly but oscillates and a long one is smooth but cuts corners; of these, only the corner-cutting end showed up in my data.
  The oscillation end did not appear down to 0.3 m with these actuator limits — I did not test shorter.
* *Inflation.* More inflation buys clearance monotonically (mean minimum obstacle distance 0.16 → 0.26 → 0.32 → 0.35 m for 0.2 → 0.3 → 0.4 → 0.5 m) and
  eventually costs detours: at 0.5 m the narrowest corridors of the cluttered world (0.819 m wide, less than 2 × 0.5 m) are closed on the grid; all five episodes still succeed, but the mean path grows from 8.40 m to
  11.39 m and the mean time from 18.68 s to 27.48 s. Too little (0.2 m) touches obstacles (failure mode C), too much (0.5 m) is slow; the five episodes do not show a failure from too much inflation, and I did not look for the value at which a start/goal pair becomes unreachable.

## 5. What I would try next

1. **A recovery behaviour** (back up / rotate when the contact sensor fires or progress stalls) — failure mode B is purely the absence of one.
2. **Make the clearance honest:** inflate from a distance transform with sub-cell accuracy (or add the discretization error to the inflation radius) and a soft cost that keeps paths in the middle of corridors; re-measure §3.2.
3. **Curvature-/clearance-aware lookahead and speed** (shorter lookahead and lower speed near obstacles and in turns) instead of one global lookahead — failure mode A.
4. **Harder, more honest benchmarks:** denser worlds (the packing limit of the current generator was the 30-obstacle layout), more episodes per world (20 gives wide confidence intervals), randomized sensor noise and
   odometry drift, dynamic obstacles.
5. **Localization:** replace ground-truth odometry with a particle filter on the known-pose map and measure what fails first.
6. **Compare against nav2** as a clearly separated, secondary baseline — it is deliberately not part of this repository.
7. Use the camera (it is in the URDF and the TF tree but no algorithm consumes it).

## Appendix: all episodes

### simple world

| ep | seed | start → goal | result | time [s] | path [m] | coll. | min dist [m] | category |
|---|---|---|---|---|---|---|---|---|
| 1 | 1001 | (-3.76, -0.00) → (1.11, 4.73) | success | 14.8 | 6.52 | 0 | 0.49 | success |
| 2 | 1002 | (5.20, -1.43) → (-5.31, 5.50) | success | 28.5 | 12.61 | 0 | 0.30 | success |
| 3 | 1003 | (-0.62, 4.53) → (1.91, -1.81) | success | 16.2 | 6.59 | 0 | 0.34 | success |
| 4 | 1004 | (1.06, 2.16) → (4.33, -4.61) | success | 17.8 | 7.28 | 0 | 0.46 | success |
| 5 | 1005 | (5.18, -3.01) → (-0.60, 3.51) | success | 18.7 | 8.46 | 0 | 0.35 | success |
| 6 | 1006 | (1.60, -1.97) → (-4.03, -2.62) | success | 11.6 | 5.38 | 0 | 0.85 | success |
| 7 | 1007 | (-5.06, -1.56) → (3.02, -1.64) | success | 17.3 | 7.82 | 0 | 0.94 | success |
| 8 | 1008 | (5.42, 2.62) → (-4.40, -3.82) | success | 23.9 | 11.46 | 0 | 0.33 | success |
| 9 | 1009 | (-0.23, 3.89) → (-5.17, 2.40) | success | 11.4 | 4.91 | 0 | 0.75 | success |
| 10 | 1010 | (-0.14, -0.06) → (-2.24, -5.30) | success | 11.9 | 5.38 | 0 | 0.97 | success |
| 11 | 1011 | (-5.38, -0.91) → (4.23, -0.88) | success | 20.0 | 9.33 | 0 | 0.30 | success |
| 12 | 1012 | (-5.24, -0.03) → (-4.28, 5.14) | success | 11.7 | 5.00 | 0 | 0.38 | success |
| 13 | 1013 | (-1.62, -2.19) → (-4.30, 2.48) | success | 13.3 | 5.19 | 0 | 0.35 | success |
| 14 | 1014 | (2.08, -5.24) → (2.27, 0.97) | success | 12.8 | 5.94 | 0 | 0.69 | success |
| 15 | 1015 | (-4.10, -1.23) → (1.88, 3.17) | success | 17.0 | 7.29 | 0 | 0.28 | success |
| 16 | 1016 | (4.73, -0.44) → (-4.42, -3.70) | success | 20.1 | 9.49 | 0 | 0.28 | success |
| 17 | 1017 | (-4.53, -1.35) → (3.15, -3.11) | success | 17.5 | 7.61 | 0 | 0.64 | success |
| 18 | 1018 | (-0.06, -5.41) → (5.03, 3.07) | success | 22.3 | 9.66 | 0 | 0.31 | success |
| 19 | 1019 | (-1.50, -3.48) → (1.45, 1.36) | success | 12.1 | 5.39 | 0 | 1.24 | success |
| 20 | 1020 | (-0.12, 4.98) → (-4.41, -4.44) | success | 23.5 | 10.13 | 0 | 0.31 | success |

### cluttered world

| ep | seed | start → goal | result | time [s] | path [m] | coll. | min dist [m] | category |
|---|---|---|---|---|---|---|---|---|
| 1 | 1001 | (4.23, 3.90) → (4.87, -2.29) | success | 14.0 | 6.23 | 0 | 0.26 | success |
| 2 | 1002 | (-5.31, 5.50) → (0.17, 5.04) | success | 12.6 | 5.23 | 0 | 0.18 | success |
| 3 | 1003 | (3.69, -3.75) → (-1.86, 5.46) | success | 22.9 | 10.85 | 0 | 0.33 | success |
| 4 | 1004 | (-2.33, 5.38) → (-5.46, -5.08) | success | 24.4 | 10.95 | 0 | 0.26 | success |
| 5 | 1005 | (5.18, -3.01) → (-0.60, 3.51) | success | 19.1 | 8.76 | 0 | 0.30 | success |
| 6 | 1006 | (3.17, -4.88) → (-3.69, -0.09) | success | 18.3 | 8.27 | 0 | 0.26 | success |
| 7 | 1007 | (-5.06, -1.56) → (3.02, -1.64) | success | 17.7 | 8.00 | 0 | 0.29 | success |
| 8 | 1008 | (-5.18, 2.16) → (3.70, -4.46) | success | 24.3 | 11.29 | 0 | 0.32 | success |
| 9 | 1009 | (-0.23, 3.89) → (-5.17, 2.40) | success | 12.9 | 5.30 | 0 | 0.36 | success |
| 10 | 1010 | (-0.14, -0.06) → (-2.24, -5.30) | success | 12.3 | 5.55 | 0 | 0.28 | success |
| 11 | 1011 | (-2.34, -3.63) → (-5.35, 2.49) | success | 14.1 | 6.53 | 0 | 0.33 | success |
| 12 | 1012 | (-0.18, -3.77) → (5.20, 1.62) | success | 17.2 | 7.44 | 0 | 0.32 | success |
| 13 | 1013 | (1.90, 5.45) → (3.67, -2.19) | success | 18.7 | 7.88 | 0 | 0.23 | success |
| 14 | 1014 | (-3.16, -2.05) → (1.41, -5.09) | success | 12.9 | 5.51 | 0 | 0.32 | success |
| 15 | 1015 | (2.76, -1.43) → (-4.38, 4.00) | success | 20.4 | 9.05 | 0 | 0.19 | success |
| 16 | 1016 | (4.13, -2.65) → (-3.40, 2.12) | success | 19.8 | 8.67 | 0 | 0.29 | success |
| 17 | 1017 | (-0.76, 5.35) → (-4.71, -5.12) | success | 25.7 | 11.52 | 0 | 0.27 | success |
| 18 | 1018 | (-0.06, -5.41) → (5.03, 3.07) | success | 23.3 | 10.03 | 0 | 0.33 | success |
| 19 | 1019 | (4.89, -2.48) → (-0.82, -5.09) | success | 13.7 | 6.19 | 0 | 0.32 | success |
| 20 | 1020 | (-5.25, 2.07) → (-2.32, -5.42) | success | 16.6 | 7.80 | 0 | 0.36 | success |
