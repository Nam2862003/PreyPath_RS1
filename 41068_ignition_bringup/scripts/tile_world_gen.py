#!/usr/bin/env python3
import argparse, math, random, sys
from pathlib import Path
from model_list import *   # must define PLANE_SIZE, FOREST_PLANE_URI
                           # and (optionally) OAK_URI/PINE_URI/ROCK_URIS if reused

TILE_SIZE = 50.0  # must match how tiles were generated

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
    footer = """
  </world>
</sdf>
"""
    return header + ground + tiles + footer

def main():
    ap = argparse.ArgumentParser(description="Assemble a world with ground planes + forest tiles")
    ap.add_argument("--size", type=float, default=150.0,
                    help="Approx world width in meters (centered at 0,0)")
    ap.add_argument("--variants", nargs="*",
                    help="Tile model names (without model://). Defaults to forest_tile_0..2")
    ap.add_argument("--outfile", default=None,
                    help="Output SDF (default: <pkg>/worlds/world_gen.sdf)")
    args = ap.parse_args()

    variants = args.variants or ["forest_tile_0", "forest_tile_1", "forest_tile_2"]
    xml = make_world_xml(variants, args.size)

    pkg_root = Path(__file__).resolve().parents[1]
    out = Path(args.outfile) if args.outfile else (pkg_root / "worlds" / "world_gen.sdf")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(xml)
    print(f"[world] wrote {out} using variants: {variants}")

if __name__ == "__main__":
    sys.exit(main())
