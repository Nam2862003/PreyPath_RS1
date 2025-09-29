#!/usr/bin/env python3
import argparse, math, random, sys
from pathlib import Path
from model_list import *

# ---- Configure your asset URIs here (Gazebo model:// or file://) ----

TILE_SIZE = 50.0  # meters
Z_GROUND  = 0.0

def sample_points(n, half, min_dist, seed=None):
    if seed is not None:
        random.seed(seed)
    pts = []
    tries = 0
    while len(pts) < n and tries < 20000:
        tries += 1
        x = random.uniform(-half, half)
        y = random.uniform(-half, half)
        if all((x-px)**2 + (y-py)**2 >= (min_dist**2) for px,py in pts):
            pts.append((x,y))
    return pts

def write_model(tile_dir: Path, model_name: str, xml: str):
    tile_dir.mkdir(parents=True, exist_ok=True)
    (tile_dir / "model.sdf").write_text(xml)
    (tile_dir / "model.config").write_text(f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author><name>PreyPath</name></author>
  <description>Prebaked forest tile</description>
</model>
""")

def build_tile_sdf(model_name: str, oaks: int, pines: int, rocks: int,
                   md_oak: float, md_pine: float, md_rock: float, seed: int|None):
    half = TILE_SIZE/2.0
    if seed is None:
        seed = random.SystemRandom().randint(0, 2**31-1)
    random.seed(seed)

    oak_pts  = sample_points(oaks,  half, md_oak,  seed+11)
    pine_pts = sample_points(pines, half, md_pine, seed+22)
    rock_pts = sample_points(rocks, half, md_rock, seed+33)

    def inc(uri, name, x,y, yaw=0.0):
        return f"""
      <include>
        <uri>{uri}</uri>
        <name>{name}</name>
        <pose>{x:.3f} {y:.3f} {Z_GROUND:.3f} 0 0 {yaw:.6f}</pose>
        <static>true</static>
        <self_collide>false</self_collide>
        <allow_auto_disable>true</allow_auto_disable>
      </include>"""

    pieces = []
    for i,(x,y) in enumerate(oak_pts,1):
        pieces.append(inc(OAK_URI, f"oak_{i}", x,y, yaw=random.uniform(-math.pi, math.pi)))
    for i,(x,y) in enumerate(pine_pts,1):
        pieces.append(inc(PINE_URI, f"pine_{i}", x,y, yaw=random.uniform(-math.pi, math.pi)))
    for i,(x,y) in enumerate(rock_pts,1):
        pieces.append(inc(ROCK_URIS[(i-1)%len(ROCK_URIS)], f"rock_{i}", x,y,
                          yaw=random.uniform(-math.pi, math.pi)))

    # One model, single link; all children static via <include>
    includes = "\n".join(pieces)   # each piece is an <include>…</include>
    sdf = f"""<?xml version="1.0"?>
    <sdf version="1.8">
      <model name="{model_name}" static="true">
        <pose>0 0 0 0 0 0</pose>
        {includes}
      </model>
    </sdf>
    """

    return sdf, seed


def main():
    ap = argparse.ArgumentParser(description="Generate 50x50 forest tile models")
    ap.add_argument("--out", default=None, help="Output models directory (defaults to <pkg>/models)")
    ap.add_argument("--variants", type=int, default=3, help="How many tile variants to generate")
    ap.add_argument("--oaks",  type=int, default=10)
    ap.add_argument("--pines", type=int, default=10)
    ap.add_argument("--rocks", type=int, default=6)
    ap.add_argument("--mdist_oak",  type=float, default=3.0)
    ap.add_argument("--mdist_pine", type=float, default=3.0)
    ap.add_argument("--mdist_rock", type=float, default=2.0)
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()

    # Resolve package root (…/41068_IGNITION_BRINGUP)
    pkg_root = Path(__file__).resolve().parents[1]

    # Default output = <pkg_root>/models unless overridden
    outroot = Path(args.out) if args.out is not None else (pkg_root / "models")

    for i in range(args.variants):
        name = f"forest_tile_{i}"
        tile_dir = outroot / name
        xml, seed_used = build_tile_sdf(
            name, args.oaks, args.pines, args.rocks,
            args.mdist_oak, args.mdist_pine, args.mdist_rock,
            None if args.seed is None else args.seed + i
        )
        write_model(tile_dir, name, xml)
        print(f"[tile] wrote {tile_dir}/model.sdf  (seed={seed_used})")

if __name__ == "__main__":
    sys.exit(main())