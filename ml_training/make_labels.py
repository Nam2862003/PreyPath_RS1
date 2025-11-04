from pathlib import Path
img_dir = Path('sim/images/raw')
lbl_dir = Path('sim/labels/raw'); lbl_dir.mkdir(parents=True, exist_ok=True)

exts = {'.jpg','.jpeg','.png'}
for p in img_dir.iterdir():
    if p.suffix.lower() in exts:
        (lbl_dir / (p.stem + '.txt')).write_text('')  # empty = negative
