import os
from pathlib import Path

# --- SETTINGS ---
LABELS_DIR = "animal2/valid/labels"   # path to your label folder
NEW_CLASS_ID = 1                      # e.g. set to 2 for 'gun'

def change_class_ids(label_dir, new_id):
    label_dir = Path(label_dir)
    txt_files = list(label_dir.rglob("*.txt"))

    print(f"Found {len(txt_files)} label files in {label_dir}")

    for f in txt_files:
        with open(f, "r") as file:
            lines = file.readlines()

        new_lines = []
        for line in lines:
            parts = line.strip().split()
            if not parts:
                continue
            parts[0] = str(new_id)  # replace first number (class id)
            new_lines.append(" ".join(parts) + "\n")

        with open(f, "w") as file:
            file.writelines(new_lines)

    print("✅ Done! Updated all class IDs.")

if __name__ == "__main__":
    change_class_ids(LABELS_DIR, NEW_CLASS_ID)
