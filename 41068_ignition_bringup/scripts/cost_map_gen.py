#!/usr/bin/env python3
"""
Generate an 80x80 m empty occupancy map (.pgm + .yaml)
Run from inside the ROS2 workspace (source your env first)
"""

from PIL import Image
import numpy as np
import yaml
from pathlib import Path

def main():
    resolution = 0.05       # meters/pixel
    width_m = 80.0
    height_m = 80.0
    width_px = int(width_m / resolution)
    height_px = int(height_m / resolution)

    # Create directory for outputs
    out_dir = Path(__file__).resolve().parent.parent / "maps"
    out_dir.mkdir(exist_ok=True)
    pgm_path = out_dir / "forest_empty_map.pgm"
    yaml_path = out_dir / "forest_empty_map.yaml"

    # Create empty map (255 = free, 0 = occupied)
    map_array = np.ones((height_px, width_px), dtype=np.uint8) * 255
    border = 10
    map_array[:border, :] = 0
    map_array[-border:, :] = 0
    map_array[:, :border] = 0
    map_array[:, -border:] = 0

    # Save .pgm
    Image.fromarray(map_array).save(pgm_path)

    # Save .yaml
    yaml_data = {
        'image': str(pgm_path.name),
        'mode': 'trinary',
        'resolution': resolution,
        'origin': [-width_m / 2, -height_m / 2, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25
    }

    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_data, f)

    print(f"Map saved to: {out_dir}")
    print(f"  - {pgm_path.name}")
    print(f"  - {yaml_path.name}")

if __name__ == "__main__":
    main()
