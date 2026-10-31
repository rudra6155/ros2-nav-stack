# ARCHITECTURE

> Simulated in Gazebo Classic (gazebo11) with ROS 2 Humble.
> Design and specification of the computation graph, TF tree, topics, and timing model.

## 1. Node / topic graph

```
┌─ gzserver (headless Gazebo Classic 11) ────────────────────────────────────────────────────────────┐
│ lidar_plugin     ──► /scan            sensor_msgs/LaserScan   (frame "laser")                      │
│ diff_drive       ──► /odom, /tf       nav_msgs/Odometry, odom→base_link  (ground truth)            │
│                  ◄── /cmd_vel         geometry_msgs/Twist                                          │
│ bumper_plugin    ──► /bumper_states   gazebo_msgs/ContactsState  (chassis only)                    │
│ gazebo_ros_state ──► /set_entity_state, /get_entity_state  (services)                              │
│ gazebo_ros_init  ──► /clock           simulation time for every node                               │
└────────────────────────────────────────────────────────────────────────────────────────────────────┘

                /scan (lidar_plugin)                                    /cmd_vel (to diff_drive)
                     │                                                          ▲
                     ▼                                                          │
 ┌──────────────────────────┐  /costmap  ┌─────────────────────┐  /plan  ┌───────────────────┐
 │ obstacle_detector        │──────────► │ astar_planner       │───────► │ waypoint_follower │
 │ 2° resampling, clusters, │            │ 8-connected A*,     │         │ pure pursuit,     │
 │ log-odds grid (TF poses) │            │ replans on new maps │         │ clamped Twist     │
 └──────────────────────────┘            └─────────────────────┘         └───────────────────┘
        │ /obstacles                              ▲ /goal_pose, /goal_cancel         ▲ /odom
        ▼ (PoseArray, diagnostics)                │ (from run_episodes)              │ (from diff_drive)

┌─ run_episodes (evaluation harness, only while measuring) ──────────────────────────────────────────┐
│ starts the stack as a child process; places the robot (/set_entity_state); calls /reset_costmap;   │
│ publishes /goal_pose, /goal_cancel; listens to /odom /plan /costmap /bumper_states; measures.      │
│ robot_state_publisher ──► /tf_static  base_link→{laser, camera, caster_*}, /robot_description      │
│ map_odom_static_tf    ──► /tf_static  map→odom  (identity: odom IS the Gazebo world frame)         │
└────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

| Topic | Type | Rate | Publisher → subscriber |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | physics rate | gazebo_ros_init → every node (`use_sim_time`) |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | lidar_plugin → obstacle_detector |
| `/odom` | `nav_msgs/Odometry` | 50 Hz | diff_drive → waypoint_follower, run_episodes |
| `/tf` | `tf2_msgs/TFMessage` | 50 Hz | diff_drive (`odom→base_link`) |
| `/tf_static` | `tf2_msgs/TFMessage` | latched | robot_state_publisher, map_odom_static_tf |
| `/obstacles` | `geometry_msgs/PoseArray` | 10 Hz (one per scan) | obstacle_detector → (diagnostics) |
| `/costmap` | `nav_msgs/OccupancyGrid` | 2 Hz | obstacle_detector → astar_planner, run_episodes |
| `/goal_pose` | `geometry_msgs/PoseStamped` | on demand | run_episodes → astar_planner |
| `/goal_cancel` | `std_msgs/Empty` | on demand | run_episodes → astar_planner |
| `/plan` | `nav_msgs/Path` | 2 Hz (empty = "no plan / unreachable") | astar_planner → waypoint_follower |
| `/cmd_vel` | `geometry_msgs/Twist` | 20 Hz | waypoint_follower → diff_drive |
| `/bumper_states` | `gazebo_msgs/ContactsState` | contact sensor rate | bumper_plugin → run_episodes |
| `/reset_costmap` | `std_srvs/Empty` (service) | per episode | run_episodes → obstacle_detector |

## 2. TF tree

```
 map ──(static identity)──► odom ──(diff_drive, ground truth)──► base_link ─┬─► laser     (z = +0.12 m, fixed)
                                                                           ├─► camera    (x = +0.13 m, fixed)
                                                                           ├─► left_wheel, right_wheel
                                                                           └─► caster_front, caster_rear
```

`map` and `odom` coincide because `/odom` is Gazebo's ground-truth world pose (`odometry_source = 1`).
There is no localization in this project. The frame names are asserted by `scripts/check_frames.sh`.

## 3. Rates and why

| Loop | Rate | Why |
|---|---|---|
| Physics | 500 Hz (`max_step_size` 0.002 s), real-time factor capped at 1.0 | a small fixed step (I did not tune it); the cap at real time makes the timing of the CPU-bound nodes comparable across runs |
| Lidar / perception | 10 Hz | the sensor's rate; one map update and one `/obstacles` message per scan |
| Costmap publication | 2 Hz | the grid is 14 m × 14 m at 0.10 m (about 20 000 cells); the planner consumes it at its own 2 Hz, so publishing faster would only add traffic |
| Planner | 2 Hz | the map only changes meaningfully when the robot has moved; at the speed limit the robot moves 0.25 m between replans, below the inflation radius. I did not sweep this rate |
| Controller | 20 Hz | a conventional tracking-loop rate, ≫ the plan rate, so the tracker always works on a plan that is at most one replan old (a plan older than 3 s stops the robot). I did not sweep this rate |
| Odometry / TF | 50 Hz | diff-drive plugin update rate; scans are looked up in TF at their own timestamp |

## 4. Perception (`perception/`)

* Input `/scan`, 360 samples. It is **downsampled to a fixed angular resolution of 2.0°**
  (180 beams over 360°, bins on the grid θₖ = −π + k·res). Each bin keeps its closest valid return, which is
  conservative for obstacle avoidance. The result does not depend on the sensor's native resolution.
* `/obstacles`: neighbouring returns closer than `0.15 m + 1.5·res·range` form a cluster (wrap-around at ±π handled); clusters with
  ≥ 3 points are published as their centroid, in `map`.
* `/costmap`: a log-odds occupancy grid (14 m × 14 m, 0.10 m cells, Bresenham ray casting from the
  sensor pose looked up in TF **at the scan's timestamp**). It uses given (ground-truth) poses: this is mapping, **not SLAM**.
  Values: 0 free, 100 occupied, −1 unknown. The grid is **not** inflated here.
* Unit-tested without a ROS graph (`perception/test`): synthetic wall at 1.0 m → centroid within 0.1 m, seam wrap-around, mapper cells.

## 5. Planner (`navigation/src/astar*.cpp`)

* 8-connected A\* with the octile heuristic (admissible and consistent), no corner cutting, written for this project.
* Runs on the **inflated** grid, **inflation radius 0.30 m** (robot radius 0.16 m + margin), hard (binary) inflation.
* Unknown cells are free (the map is built online), so early plans can be optimistic; the 2 Hz replanning corrects them.
* Start/goal relaxation: the inflation band around the start and the goal is lifted for non-occupied cells, so the robot can leave a
  band it is standing in and a free goal near a wall stays reachable.
* **Goal inside an obstacle or unreachable → an empty `/plan` and a warning; never a straight line, never a crash** (unit-tested and
  exercised in the fault-injection run, section 10).
* Optional line-of-sight shortcutting; the path is resampled every 0.1 m.

## 6. Controller (`navigation/src/pure_pursuit.cpp`, `control/src/waypoint_follower.cpp`)

* Pure pursuit: lookahead point at distance L (default 0.6 m, parameter `lookahead_distance`) along the plan from the
  closest path point; curvature κ = 2·yₗ / L²; v = clamp, ω = v·κ.
* **Limits** (`robot_description/config/robot_limits.yaml`, also the wheel joint limits in the URDF): |v| ≤ 0.50 m/s,
  |ω| ≤ 1.20 rad/s. If the angular limit binds, v is reduced so that κ is preserved. v is also scaled down for large
  heading error and near the goal; a heading error above 0.9 rad turns the robot on the spot.
* Stops (publishes zero) when there is no plan, the plan is empty, the plan is older than 3 s, or no odometry has arrived.

## 7. Time

Everything runs on `/clock` (`use_sim_time:=true`, **asserted** in `sim.launch.py`, which refuses to start otherwise; checked per node by
`scripts/check_tf.sh`). Wall time is used by `run_episodes` for watchdogs and for the real-time-factor measurement only.

## 8. Evaluation harness (`evaluation/`)

`run_episodes` = sampler + supervisor + recorder. Seeds are `1000 + episode`; the sampler (SplitMix64, rejection sampling with a
ground-truth reachability check) is bit-reproducible. Per episode: teleport → reset costmap → 1.5 s of sim time of fresh mapping → goal →
measure until success (within 0.3 m), timeout (120 s sim, or 3× the mean success time once 5 successes exist) or crash. Crash
detection parses the `process has died` lines of the child `ros2 launch`; a stalled clock (30 s wall without sim progress) counts as a crash too.

## 9. How to add a new node / behavior (5 steps)

1. **Create the package** (or add to an existing one): `ros2_ws/src/<pkg>/{package.xml,CMakeLists.txt,src/<node>.cpp,include/<pkg>/}`; keep the
   algorithm in a ROS-free header/library (as `navigation_core` does) so it can be unit-tested without a ROS graph.
2. **CMake**: `add_executable(<node> src/<node>.cpp)`, `ament_target_dependencies(<node> rclcpp ...)`, link `navigation::navigation_core` if needed,
   `install(TARGETS <node> DESTINATION lib/${PROJECT_NAME})`; tests with `ament_add_gtest(test_<node> test/test_<node>.cpp)`.
3. **Parameters**: add a block named after the node in `bringup/config/stack_params.yaml`; declare every parameter with a default in the node.
4. **Launch**: add `Node(package='<pkg>', executable='<node>', name='<node>', parameters=[params, {'use_sim_time': True}])` to
   `bringup/launch/stack.launch.py`.
5. **Contracts**: add the node to `scripts/expected_nodes.txt` (and its topics to `scripts/expected_topics.txt`), publish in the `map`/`odom`/
   URDF frames only, then run `colcon test`, `scripts/check_tf.sh`, `scripts/check_frames.sh`.

## 10. Known fragility (what breaks, and what I observed)

| fault | what happened (measured) | mitigation / detection in this repo |
|---|---|---|
| **Lidar frame id changes** (plugin stamps `laser_link`, URDF link is `laser`) | Every scan is dropped by the TF message filter: `/costmap` never gets a known cell (max known cells **0**, 26 "dropping message" warnings in the launch log), so A* plans through obstacles it cannot see. Episode 2 (seed 1002): success **no**, collisions **1**, stop reason `hard_timeout` (baseline run of the same episode: success yes, collisions 0). | `scripts/check_frames.sh` fails if `/scan` is not stamped exactly `laser` and exists in the URDF; the detector logs the dropped scans. |
| **Goal inside an obstacle / outside the map** | Planner publishes an empty `/plan` (0 non-empty plans for a goal in the wall, 0 for a goal outside the map), the robot did not move (max commanded linear speed 0.00 m/s, displacement 0.00 m), warnings: *goal is inside an obstacle, goal is outside the map*; planner still alive afterwards: **yes**. | Empty path + throttled warning by design (never a straight line); unit-tested in `navigation/test/test_astar.cpp`. |
| **Real-time factor drops** (CPU saturated by 12 busy loops on 4 cores, the stack at nice +15) | Episode 2: RTF 1.00 → **0.27**, success yes, time to goal 28.460 s → 28.460 s (sim), path 12.61 → 12.62 m, collisions 0. Episode 8: RTF 1.00 → **0.32**, success yes, time 23.940 s → 23.900 s, path 11.46 → 11.47 m, collisions 0. | All periodic work runs on ROS-clock timers and the stack consumes `/clock`, so a slower simulation changes wall time, not sim-time behaviour; RTF is logged per episode (`rtf_log.csv`, `limits_report.txt` below 0.3). |
