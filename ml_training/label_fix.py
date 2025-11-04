from pathlib import Path

# --- SETTINGS ---
LABELS_DIR = Path("/home/greese/git/PreyPath_RS1/ml_training/sim/label_studio/labels")  # change this!
OLD_ID = "1"
NEW_ID = "0"

for txt_path in LABELS_DIR.glob("*.txt"):
    lines_out = []
    with open(txt_path, "r") as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            if parts[0] == OLD_ID:
                parts[0] = NEW_ID
            lines_out.append(" ".join(parts) + "\n")

    with open(txt_path, "w") as f:
        f.writelines(lines_out)

    print(f"✅ Updated {txt_path.name}")
