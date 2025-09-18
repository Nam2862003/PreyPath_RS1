#!/usr/bin/env python3
import argparse, math, random, sys
from pathlib import Path
from model_list import *   # must define PLANE_SIZE, FOREST_PLANE_URI
                           # and (optionally) OAK_URI/PINE_URI/ROCK_URIS if reused

TILE_SIZE = 50.0  # must match how tiles were generated


def resolve_worlds_dir(package: str) -> Path:
    """
    Return <pkg_source_root>/worlds, creating it if needed.
    Works whether you run from source or from an installed env.
    """
    here = Path(__file__).resolve()
    # script is typically <pkg>/scripts/world_gen.py  -> parents[1] is <pkg>
    pkg_root = here.parents[1]
    worlds = pkg_root / "worlds"
    worlds.mkdir(parents=True, exist_ok=True)
    return worlds


def tile_indices_to_cover(size_m):
    n = int(math.ceil(size_m / TILE_SIZE))
    if n % 2 == 0:
        n += 1
    k = n // 2
    return range(-k, k + 1)

# ---------- ground (checkerboard) ----------
def create_ground(size):
    n = int(math.ceil(size / PLANE_SIZE))
    if n % 2 == 0:
        n += 1

    half = n // 2
    pieces = []
    for horiz in range(-half, half + 1):
        for verical in range(-half, half + 1):
            x = PLANE_SIZE * horiz
            y = PLANE_SIZE * verical
            yaw = (math.pi / 2.0) if ((horiz + verical) % 2) else 0.0
            pieces.append(
                f"""
    <include>
      <uri>{FOREST_PLANE_URI}</uri>
      <name>forest_plane_{horiz}_{verical}</name>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.6f}</pose>
    </include>"""
            )
    return "".join(pieces)


def create_perimeter_walls(size,
                           seg_len=9.32,   # wall length along local X
                           height=4.0,     # wall height
                           uri=FOREST_WALL_URI,
                           name_prefix="forest_wall"):
    """
    Build a closed rectangle of wall segments centered at (0,0),
    with world half-extent = size/2.
    Each segment is assumed to lie along its local X axis.
    """
    half = size / 2.0
    z = height / 2.0      # place center so the wall sits on z=0
    n = int(math.ceil(size / seg_len))  # segments per side
    step = size / n                      # center-to-center spacing
    centers = [-half + (i + 0.5) * step for i in range(n)]
    angle90 = math.pi / 2.0

    pieces = []

    # Top (+y) and bottom (-y) edges: walls run along X (yaw = 0)
    for i, x in enumerate(centers):
        # top
        pieces.append(f"""
    <include>
      <uri>{uri}</uri>
      <name>{name_prefix}_top_{i}</name>
      <pose>{x:.3f} {half:.3f} {z:.3f} {angle90:.6f} 0 0</pose>
      <static>true</static>
    </include>""")
        # bottom
        pieces.append(f"""
    <include>
      <uri>{uri}</uri>
      <name>{name_prefix}_bot_{i}</name>
      <pose>{x:.3f} {-half:.3f} {z:.3f} {angle90:.6f} 0 0</pose>
      <static>true</static>
    </include>""")

    # Right (+x) and left (-x) edges: rotate 90° so walls run along Y (yaw = π/2)

    for i, y in enumerate(centers):
        # right
        pieces.append(f"""
    <include>
      <uri>{uri}</uri>
      <name>{name_prefix}_right_{i}</name>
      <pose>{half:.3f} {y:.3f} {z:.3f} {angle90:.6f} 0 {angle90:.6f}</pose>
      <static>true</static>
    </include>""")
        # left
        pieces.append(f"""
    <include>
      <uri>{uri}</uri>
      <name>{name_prefix}_left_{i}</name>
      <pose>{-half:.3f} {y:.3f} {z:.3f} {angle90:.6f} 0 {angle90:.6f}</pose>
      <static>true</static>
    </include>""")

    return "".join(pieces)


# ---------- tiles ----------
def make_tiles(variant_names, size):
    xs = tile_indices_to_cover(size)
    body = []
    idx = 0
    for ix in xs:
        for iy in xs:
            uri = f"model://{variant_names[idx % len(variant_names)]}"
            x = ix * TILE_SIZE
            y = iy * TILE_SIZE
            body.append(
                f"""
    <include>
      <uri>{uri}</uri>
      <name>tile_{ix}_{iy}</name>
      <pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
    </include>"""
            )
            idx += 1
    return "".join(body)

def make_world_xml(variants, size, add_sun=True):
    header = f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="tile_gen">
    <scene>
      <ambient>1 1 1 1</ambient>
      <background>0.6 0.8 1.0 1</background>
      <shadows>0</shadows><grid>0</grid>
    </scene>
    { "<light name='sun' type='directional'><cast_shadows>0</cast_shadows><pose>0 0 100 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.8 0.8 0.8 1</specular><direction>-0.5 0.1 -0.9</direction><intensity>5</intensity></light>" if add_sun else "" }
"""
    ground = create_ground(size)
    tiles  = make_tiles(variants, size)
    walls = create_perimeter_walls(size)
    footer = """
  </world>
</sdf>
"""
    return header + ground + tiles + walls + footer

def main():
    ap = argparse.ArgumentParser(description="Assemble a world with ground planes + forest tiles")
    ap.add_argument("--size", type=float, default=150.0,
                    help="Approx world width in meters (centered at 0,0)")
    ap.add_argument("--variants", nargs="*",
                    help="Tile model names (without model://). Defaults to forest_tile_0..2")
    ap.add_argument("--outfile", default=None,
                    help="Output SDF (default: <pkg>/worlds/world_gen.sdf)")
    # NEW: optional package name (not strictly used by resolver, but handy for clarity)
    ap.add_argument("--package", default="41068_ignition_bringup",
                    help="ROS 2 package (source) to place worlds/ into")
    args = ap.parse_args()

    variants = args.variants or ["forest_tile_0", "forest_tile_1", "forest_tile_2"]
    xml = make_world_xml(variants, args.size)

    # >>> CHANGED: write into <pkg>/worlds via the helper
    worlds_dir = resolve_worlds_dir(args.package)
    out = Path(args.outfile) if args.outfile else (worlds_dir / "world_gen.sdf")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(xml)
    print(f"[world] wrote {out} using variants: {variants}")


if __name__ == "__main__":
    sys.exit(main())
