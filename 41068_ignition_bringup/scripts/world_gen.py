#!/usr/bin/env python3
"""
Generate a random Gazebo SDF world and overwrite:
  <share>/<package>/worlds/world_gen.sdf

Usage (random new world every time):
  ros2 run 41068_ignition_bringup world_gen.py

Reproducible:
  ros2 run 41068_ignition_bringup world_gen.py --seed 123 --oaks 12 --pines 10 --rocks 6

Tip: call this from your launch *before* starting Gazebo, then load the file:
  PathJoinSubstitution([FindPackageShare('<pkg>'),'worlds','world_gen.sdf'])
"""

import argparse, os, sys, math, random
from pathlib import Path
from model_list import *

# import files from directory
try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None  # fallback handled below


# ---------- helpers ----------
def resolve_worlds_dir(package: str) -> Path:
    """Return the worlds directory in the source tree."""
    here = Path(__file__).resolve()
    src_worlds = here.parent.parent / "worlds"
    src_worlds.mkdir(parents=True, exist_ok=True)
    return src_worlds


def sample_points(n, half_size, min_dist, keepouts=None, max_tries=10000):
    """ Rejection sampling with minimum spacing (approx Poisson disk). """
    pts = []
    tries = 0
    keepouts = keepouts or []
    while len(pts) < n and tries < max_tries:
        tries += 1
        x = random.uniform(-half_size, half_size)
        y = random.uniform(-half_size, half_size)

        # keep-out rectangles: (xmin,xmax,ymin,ymax)
        skip = False
        for (xmin, xmax, ymin, ymax) in keepouts:
            if xmin <= x <= xmax and ymin <= y <= ymax:
                skip = True
                break
        if skip:
            continue

        ok = True
        for (px, py) in pts:
            if (px - x) ** 2 + (py - y) ** 2 < (min_dist ** 2):
                ok = False
                break
        if ok:
            pts.append((x, y))
    return pts


def include_block(uri, name, x, y, z=0.0, yaw=0.0, static=True):
    return f"""
    <include>
      <uri>{uri}</uri>
      <name>{name}</name>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.6f}</pose>
      {'<static>true</static>' if static else '<static>false</static>'}
      <self_collide>false</self_collide>
      <allow_auto_disable>true</allow_auto_disable>
    </include>"""


def create_ground(size):
  n = int(math.ceil(size / PLANE_SIZE))

  if n % 2 == 0:
    n += 1

  half_plane_itterations = n // 2
  pieces = []

  for horiz in range((-half_plane_itterations), half_plane_itterations + 1): 
    for verical in range((-half_plane_itterations), half_plane_itterations + 1): 
      x = PLANE_SIZE * horiz 
      y = PLANE_SIZE * verical 
      yaw = (math.pi / 2.0) if ((horiz + verical) % 2) else 0.0
      pieces.append(f""" <include> 
                          <uri>{FOREST_PLANE_URI}</uri> 
                          <name>forest_plane{horiz}_{verical}</name>
                          <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.6f}</pose> 
                          </include>""")
    
  return "".join(pieces)


def build_world_xml(size, oaks, pines, rocks, md_oak, md_pine, md_rock,
                    walls, clearing_half, seed):
    half = size / 2.0
    if seed is None:
        seed = random.SystemRandom().randint(0, 2**31-1)
    random.seed(seed)

    keepouts = []
    if clearing_half > 0:
        keepouts.append((-clearing_half, clearing_half, -clearing_half, clearing_half))

    oak_pts  = sample_points(oaks,  half, md_oak,  keepouts)
    pine_pts = sample_points(pines, half, md_pine, keepouts)
    rock_pts = sample_points(rocks, half, md_rock, keepouts)

    # --- world header ---
    xml = f"""<?xml version="1.0"?>
    <sdf version="1.8">
    <world name="world_gen">
    <!-- Core systems -->
    <plugin name="ignition::gazebo::systems::Physics" filename="ignition-gazebo-physics-system"/>
    <plugin name="ignition::gazebo::systems::UserCommands" filename="ignition-gazebo-user-commands-system"/>
    <plugin name="ignition::gazebo::systems::SceneBroadcaster" filename="ignition-gazebo-scene-broadcaster-system"/>
    <plugin name="ignition::gazebo::systems::Contact" filename="ignition-gazebo-contact-system"/>

    <light name="sun" type="directional">
      <cast_shadows>0</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.8 0.8 0.8 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
      <intensity>5</intensity>
    </light>

    <gravity>0 0 -9.81</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <physics name="default_physics" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
    </physics>

    <scene>
      <ambient>1 1 1 1</ambient>
      <background>0.6 0.8 1.0 1</background>
      <shadows>0</shadows>
      <grid>0</grid>
    </scene>

    <spherical_coordinates>
      <latitude_deg>0.0</latitude_deg>
      <longitude_deg>0.0</longitude_deg>
      <elevation>10.0</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>


""" 
    # Ground creation
    xml += create_ground(size)

    # Boundary walls (optional ring like your demo)
    if walls:
        wall_poses = [
            ( size/2,    0.0,   2, 1.570796, 0, -1.570796),
            ( size/2, -0.75*size/2, 2, 1.570796, 0, -1.570796),
            ( size/2,  0.75*size/2, 2, 1.570796, 0, -1.570796),
            (-size/2,    0.0,   2, 1.570796, 0,  1.570796),
            (-size/2, -0.75*size/2, 2, 1.570796, 0,  1.570796),
            (-size/2,  0.75*size/2, 2, 1.570796, 0,  1.570796),
            ( 0.0, -size/2,      2, 1.570796, 0,  0),
            (-0.75*size/2, -size/2, 2, 1.570796, 0,  0),
            ( 0.75*size/2, -size/2, 2, 1.570796, 0,  0),
            ( 0.0,  size/2,      2, 1.570796, 0,  0),
            (-0.75*size/2, size/2, 2, 1.570796, 0,  0),
            ( 0.75*size/2, size/2, 2, 1.570796, 0,  0),
        ]
        for i, (x,y,z,rr,pp,yy) in enumerate(wall_poses, 1):
            xml += f"""
    <include>
      <uri>{FOREST_WALL_URI}</uri>
      <name>forest_wall{i}</name>
      <pose>{x:.3f} {y:.3f} {z:.3f} {rr} {pp} {yy}</pose>
    </include>"""

    # Trees
    for i, (x,y) in enumerate(oak_pts, 1):
        yaw = random.uniform(-math.pi, math.pi)
        xml += include_block(OAK_URI, f"oak_{i}", x, y, z=-0.30, yaw=yaw)

    for i, (x,y) in enumerate(pine_pts, 1):
        yaw = random.uniform(-math.pi, math.pi)
        xml += include_block(PINE_URI, f"pine_{i}", x, y, yaw=yaw)

    # Rocks
    for i, (x,y) in enumerate(rock_pts, 1):
        yaw = random.uniform(-math.pi, math.pi)
        uri = ROCK_URIS[(i-1) % len(ROCK_URIS)]
        xml += include_block(uri, f"rock_{i}", x, y, yaw=yaw)

    # footer
    xml += f"""
    <!-- Debug info -->
    <model name="seed_marker" static="true">
      <pose>0 0 0 0 0 0</pose>
      <link name="seed">
        <visual name="txt"><geometry><box><size>0.001 0.001 0.001</size></box></geometry></visual>
      </link>
    </model>
  </world>
</sdf>
"""
    return xml, seed

# ---------- main ----------
def main():
    ap = argparse.ArgumentParser(description="Random world generator -> worlds/world_gen.sdf")
    ap.add_argument("--package", default="41068_ignition_bringup", help="Target ROS 2 package")
    ap.add_argument("--outfile", default=None, help="Override output path")
    ap.add_argument("--size", type=float, default=25.0, help="World side length (meters)")
    ap.add_argument("--oaks", type=int, default=8)
    ap.add_argument("--pines", type=int, default=8)
    ap.add_argument("--rocks", type=int, default=6)
    ap.add_argument("--mdist_oak", type=float, default=2.5, help="Min spacing between oaks (m)")
    ap.add_argument("--mdist_pine", type=float, default=2.5, help="Min spacing between pines (m)")
    ap.add_argument("--mdist_rock", type=float, default=1.5, help="Min spacing between rocks (m)")
    ap.add_argument("--clearing_half", type=float, default=2.0, help="Half-size of central clearing (m, 0=off)")
    ap.add_argument("--walls", action="store_true", help="Add boundary walls (requires model://forest_wall)")
    ap.add_argument("--seed", type=int, default=None, help="Random seed (None=random each run)")
    args = ap.parse_args()

    worlds_dir = resolve_worlds_dir(args.package)
    outpath = Path(args.outfile) if args.outfile else (worlds_dir / "world_gen.sdf")

    xml, seed_used = build_world_xml(
        size=args.size,
        oaks=args.oaks, pines=args.pines, rocks=args.rocks,
        md_oak=args.mdist_oak, md_pine=args.mdist_pine, md_rock=args.mdist_rock,
        walls=args.walls, clearing_half=args.clearing_half, seed=args.seed
    )

    with open(outpath, "w") as f:
        f.write(xml)

    print(f"[world_gen] Seed: {seed_used}")
    print(f"[world_gen] Wrote: {outpath}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
