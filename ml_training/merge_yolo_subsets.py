import shutil
from pathlib import Path

# --- SETTINGS ---
# List all subset folders you want to merge
SUBSETS = [
    "animal",
    "animal2",
    "person",
    "rifle",
    "rifle2",
    # add more...
]

# Final dataset root (will have train/ and valid/ like the subsets)
DATASET_ROOT = Path("dataset")

def copy_dir_flat(src: Path, dst: Path):
    """
    Copy all files from src into dst (non-recursive).
    Creates dst if needed.
    Returns number of files copied.
    """
    dst.mkdir(parents=True, exist_ok=True)
    count = 0
    for item in src.iterdir():
        if item.is_file():
            shutil.copy(item, dst / item.name)
            count += 1
    return count

def merge_one_subset(subset_path: Path):
    """
    subset_path/
      train/images
      train/labels
      valid/images
      valid/labels
    """
    for split in ["train", "valid"]:
        src_imgs = subset_path / split / "images"
        src_lbls = subset_path / split / "labels"

        dst_imgs = DATASET_ROOT / split / "images"
        dst_lbls = DATASET_ROOT / split / "labels"

        if src_imgs.exists():
            n_img = copy_dir_flat(src_imgs, dst_imgs)
        else:
            n_img = 0

        if src_lbls.exists():
            n_lbl = copy_dir_flat(src_lbls, dst_lbls)
        else:
            n_lbl = 0

        print(f"✅ {subset_path.name}/{split}: copied {n_img} images, {n_lbl} labels")

def main():
    for subset in SUBSETS:
        subset_path = Path(subset)
        if not subset_path.exists():
            print(f"⚠️ subset not found: {subset_path}")
            continue
        merge_one_subset(subset_path)

    print("\n🎯 Done. Final dataset at:", DATASET_ROOT.resolve())

if __name__ == "__main__":
    main()
