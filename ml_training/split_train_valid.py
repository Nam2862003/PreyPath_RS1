import os
import random
import shutil
from pathlib import Path

# --- SETTINGS ---
BASE_DIR = Path("sim")    # your dataset root
SPLIT_RATIO = 0.15           # 15% of training data -> valid set
IMG_EXT = ".jpg"             # or ".png" if needed

train_img_dir = BASE_DIR / "train" / "images"
train_lbl_dir = BASE_DIR / "train" / "labels"
valid_img_dir = BASE_DIR / "valid" / "images"
valid_lbl_dir = BASE_DIR / "valid" / "labels"

# make sure target dirs exist
for d in [valid_img_dir, valid_lbl_dir]:
    d.mkdir(parents=True, exist_ok=True)

# get list of all training images
images = [f for f in train_img_dir.glob(f"*{IMG_EXT}")]

# pick random subset
num_to_move = int(len(images) * SPLIT_RATIO)
valid_subset = random.sample(images, num_to_move)

print(f"📸 Found {len(images)} images in train/")
print(f"➡️ Moving {num_to_move} images to valid/")

# move images + matching labels
for img_path in valid_subset:
    label_path = train_lbl_dir / (img_path.stem + ".txt")

    # move image
    shutil.move(str(img_path), valid_img_dir / img_path.name)

    # move label if exists
    if label_path.exists():
        shutil.move(str(label_path), valid_lbl_dir / label_path.name)

print("✅ Done! Train/Valid split complete.")
