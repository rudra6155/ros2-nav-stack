#!/usr/bin/env python3
"""Deterministic generator for simple.world and cluttered.world.

One definition produces, per world:
  <name>.world      Gazebo Classic (SDF 1.6) world, headless-friendly (no model:// downloads)
  <name>.obstacles  plain-text geometry sidecar used by the evaluation (start/goal sampling,
                    ground-truth clearance):
                        arena <half_size>
                        box <name> <cx> <cy> <yaw> <size_x> <size_y>
                        cylinder <name> <cx> <cy> <radius>
and one shared  world_stats.json  with the measured density and corridor width.

Usage:  python3 worlds/gen_worlds.py            (rewrites the files next to this script)
        python3 worlds/gen_worlds.py --check    (exit 1 if committed files differ from the generator)
"""
import argparse
import json
import math
import os
import random
import sys

ARENA_HALF = 6.0          # inner faces of the walls at +-6 m  -> 12 m x 12 m arena
WALL_T = 0.2
OBST_H = 0.5              # all obstacles are taller than the lidar plane (0.17 m above ground)
KEEP_OUT_ORIGIN = 1.0     # default robot spawn is the origin: keep it clear

WORLDS = {
    # name: (seed, n_obstacles, min_surface_gap_m, min_surface_gap_to_wall_m, size ranges)
    'simple': dict(seed=11, n=8, gap=2.0, wall_gap=1.5, r=(0.25, 0.45), box=(0.5, 1.0)),
    'empty': dict(seed=0, n=0, gap=2.0, wall_gap=1.5, r=(0.25, 0.45), box=(0.5, 1.0)),  # controller check only
    'cluttered': dict(seed=23, n=30, gap=0.9, wall_gap=0.8, r=(0.2, 0.35), box=(0.4, 1.2)),
}


# ---------------------------------------------------------------- geometry
class Shape:
    def __init__(self, kind, name, cx, cy, yaw=0.0, sx=0.0, sy=0.0, r=0.0):
        self.kind, self.name = kind, name
        self.cx, self.cy, self.yaw, self.sx, self.sy, self.r = cx, cy, yaw, sx, sy, r

    def sdf(self, px, py):
        """Signed distance from point to the shape surface (negative inside)."""
        if self.kind == 'cylinder':
            return math.hypot(px - self.cx, py - self.cy) - self.r
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = px - self.cx, py - self.cy
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        qx, qy = abs(lx) - self.sx / 2.0, abs(ly) - self.sy / 2.0
        return math.hypot(max(qx, 0.0), max(qy, 0.0)) + min(max(qx, qy), 0.0)

    def boundary_points(self, step=0.02):
        pts = []
        if self.kind == 'cylinder':
            n = max(16, int(2 * math.pi * self.r / step))
            for i in range(n):
                a = 2 * math.pi * i / n
                pts.append((self.cx + self.r * math.cos(a), self.cy + self.r * math.sin(a)))
        else:
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            hx, hy = self.sx / 2.0, self.sy / 2.0
            edges = [((-hx, -hy), (hx, -hy)), ((hx, -hy), (hx, hy)),
                     ((hx, hy), (-hx, hy)), ((-hx, hy), (-hx, -hy))]
            for (x0, y0), (x1, y1) in edges:
                n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
                for i in range(n):
                    t = i / n
                    lx, ly = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
                    pts.append((self.cx + c * lx - s * ly, self.cy + s * lx + c * ly))
        return pts

    def bounding_radius(self):
        return self.r if self.kind == 'cylinder' else math.hypot(self.sx, self.sy) / 2.0


def surface_gap(a, b, cutoff=3.0):
    """Distance between the surfaces of two shapes (approximate to the boundary sampling step).
    Returns 99 when the shapes are certainly farther apart than `cutoff`."""
    if math.hypot(a.cx - b.cx, a.cy - b.cy) > a.bounding_radius() + b.bounding_radius() + cutoff:
        return 99.0  # far apart: skip the expensive computation
    best = 99.0
    for (px, py) in a.boundary_points():
        d = b.sdf(px, py)
        best = min(best, d)
    for (px, py) in b.boundary_points():
        d = a.sdf(px, py)
        best = min(best, d)
    return best


def walls():
    h = ARENA_HALF + WALL_T / 2.0
    span = 2 * ARENA_HALF + 2 * WALL_T
    return [
        Shape('box', 'wall_north', 0.0, h, 0.0, span, WALL_T),
        Shape('box', 'wall_south', 0.0, -h, 0.0, span, WALL_T),
        Shape('box', 'wall_east', h, 0.0, 0.0, WALL_T, span),
        Shape('box', 'wall_west', -h, 0.0, 0.0, WALL_T, span),
    ]


def generate(name, cfg):
    rng = random.Random(cfg['seed'])
    placed = []
    attempts = 0
    while len(placed) < cfg['n'] and attempts < 60000:
        attempts += 1
        if rng.random() < 0.55:
            r = rng.uniform(*cfg['r'])
            sh = Shape('cylinder', f'{name}_cyl_{len(placed)}', 0, 0, r=r)
        else:
            sx = rng.uniform(*cfg['box'])
            sy = rng.uniform(cfg['box'][0], cfg['box'][1] * 1.5 if name == 'cluttered' else cfg['box'][1])
            sh = Shape('box', f'{name}_box_{len(placed)}', 0, 0, rng.uniform(-math.pi, math.pi), sx, sy)
        margin = cfg['wall_gap'] + sh.bounding_radius()
        sh.cx = rng.uniform(-ARENA_HALF + margin, ARENA_HALF - margin)
        sh.cy = rng.uniform(-ARENA_HALF + margin, ARENA_HALF - margin)
        # keep the spawn area free
        if sh.sdf(0.0, 0.0) < KEEP_OUT_ORIGIN:
            continue
        # wall clearance (exact enough: wall faces are axis-aligned)
        if min(ARENA_HALF - abs(p[0]) for p in sh.boundary_points()) < cfg['wall_gap'] - 1e-6:
            continue
        if min(ARENA_HALF - abs(p[1]) for p in sh.boundary_points()) < cfg['wall_gap'] - 1e-6:
            continue
        if any(surface_gap(sh, o, cfg['gap']) < cfg['gap'] for o in placed):
            continue
        placed.append(sh)
    if len(placed) != cfg['n']:
        sys.exit(f'{name}: could only place {len(placed)}/{cfg["n"]} obstacles')
    return placed


def stats(obstacles):
    ws = walls()
    free_area = (2 * ARENA_HALF) ** 2
    occ = 0.0
    for o in obstacles:
        occ += math.pi * o.r ** 2 if o.kind == 'cylinder' else o.sx * o.sy
    min_gap = 99.0
    if not obstacles:
        min_gap = None
    for i, a in enumerate(obstacles):
        for b in obstacles[i + 1:]:
            min_gap = min(min_gap, surface_gap(a, b))
        for w in ws:
            min_gap = min(min_gap, surface_gap(a, w))
    return dict(
        n_obstacles=len(obstacles),
        n_cylinders=sum(o.kind == 'cylinder' for o in obstacles),
        n_boxes=sum(o.kind == 'box' for o in obstacles),
        arena_m=2 * ARENA_HALF,
        obstacle_area_fraction=round(occ / free_area, 4),
        min_corridor_width_m=None if min_gap is None else round(min_gap, 3),
    )


# ---------------------------------------------------------------- writers
def f(v):
    return f'{v:.4f}'


def sdf_box(sh, color):
    return f'''    <model name="{sh.name}">
      <static>true</static>
      <pose>{f(sh.cx)} {f(sh.cy)} {f(OBST_H / 2)} 0 0 {f(sh.yaw)}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{f(sh.sx)} {f(sh.sy)} {f(OBST_H)}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{f(sh.sx)} {f(sh.sy)} {f(OBST_H)}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      </link>
    </model>
'''


def sdf_cyl(sh, color):
    return f'''    <model name="{sh.name}">
      <static>true</static>
      <pose>{f(sh.cx)} {f(sh.cy)} {f(OBST_H / 2)} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>{f(sh.r)}</radius><length>{f(OBST_H)}</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>{f(sh.r)}</radius><length>{f(OBST_H)}</length></cylinder></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      </link>
    </model>
'''


def world_sdf(name, obstacles):
    body = ''
    for w in walls():
        body += sdf_box(w, '0.55 0.55 0.6 1')
    for o in obstacles:
        body += sdf_box(o, '0.85 0.45 0.15 1') if o.kind == 'box' else sdf_cyl(o, '0.2 0.6 0.3 1')
    return f'''<?xml version="1.0" ?>
<!-- GENERATED by worlds/gen_worlds.py - do not edit by hand. -->
<sdf version="1.6">
  <world name="{name}">
    <gravity>0 0 -9.8</gravity>
    <physics name="ode_500hz" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>

    <!-- ROS interface: exposes /get_entity_state and /set_entity_state (used by the episode runner) -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/</namespace></ros>
      <update_rate>10.0</update_rate>
    </plugin>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.85 0.9 0.95 1</background>
      <shadows>false</shadows>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -1.0</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
        </visual>
      </link>
    </model>

{body}
    <!-- top-down camera for rendered (GUI) runs; ignored by gzserver -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0 0 13 0 1.5708 0</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>
'''


def obstacles_txt(obstacles):
    lines = ['# GENERATED by worlds/gen_worlds.py - geometry sidecar for the evaluation',
             f'arena {f(ARENA_HALF)}']
    for w in walls():
        lines.append(f'box {w.name} {f(w.cx)} {f(w.cy)} {f(w.yaw)} {f(w.sx)} {f(w.sy)}')
    for o in obstacles:
        if o.kind == 'box':
            lines.append(f'box {o.name} {f(o.cx)} {f(o.cy)} {f(o.yaw)} {f(o.sx)} {f(o.sy)}')
        else:
            lines.append(f'cylinder {o.name} {f(o.cx)} {f(o.cy)} {f(o.r)}')
    return '\n'.join(lines) + '\n'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true')
    args = ap.parse_args()
    here = os.path.dirname(os.path.abspath(__file__))
    outputs, all_stats = {}, {}
    for name, cfg in WORLDS.items():
        obs = generate(name, cfg)
        outputs[f'{name}.world'] = world_sdf(name, obs)
        outputs[f'{name}.obstacles'] = obstacles_txt(obs)
        all_stats[name] = dict(seed=cfg['seed'], **stats(obs))
    outputs['world_stats.json'] = json.dumps(all_stats, indent=2) + '\n'

    bad = 0
    for fn, text in outputs.items():
        path = os.path.join(here, fn)
        if args.check:
            if not os.path.exists(path) or open(path).read() != text:
                print(f'DIFFERS: {fn}')
                bad += 1
        else:
            with open(path, 'w') as fh:
                fh.write(text)
            print(f'wrote {fn}')
    for name, s in all_stats.items():
        print(name, s)
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
