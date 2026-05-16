import yaml
from pathlib import Path
from lerobot.utils.constants import HF_LEROBOT_HOME
import re
import shutil
from datetime import datetime


def clean_dataset_info():
    # ====== [LOAD CONFIG] ======
    parent_path = Path(__file__).resolve().parent
    cfg_path = parent_path.parent / "config" / "cfg.yaml"
    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f)

    repo_id = cfg["record"]["repo_id"]
    user_name = repo_id.split("/", 1)[0]

    # ====== [DEFINE PATHS] ======
    base_path = Path(HF_LEROBOT_HOME) / user_name       # e.g. ~/.cache/lerobot/scylearning
    info_file = base_path / "dataset_info.txt"

    if not info_file.exists():
        print(f"====== [ERROR] dataset_info.txt not found at {info_file} ======")
        return

    # ====== [CREATE BACKUP BEFORE MODIFICATION] ======
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_file = base_path / "dataset_info_backup" / f"dataset_info_backup_{timestamp}.txt"
    backup_file.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(info_file, backup_file)
    print(f"====== [BACKUP] Created backup file: {backup_file} ======")

    # ====== [READ EXISTING FOLDERS] ======
    existing_folders = {p.name for p in base_path.iterdir() if p.is_dir()}
    print(f"====== [INFO] Found {len(existing_folders)} existing dataset folders ======")

    # ====== [READ INFO FILE LINES] ======
    with open(info_file, "r") as f:
        lines = f.readlines()

    kept_lines = []
    removed_lines = []

    # filter lines: only keep folders that exist
    for line in lines:
        match = re.search(r'name="([^"]+)"', line)
        if match:
            full_name = match.group(1)
            folder_name = full_name.split("/", 1)[1] if "/" in full_name else full_name
            if folder_name in existing_folders:
                kept_lines.append(line)
            else:
                removed_lines.append(line)
        else:
            kept_lines.append(line)

    # ====== [UPDATE record_id] ======
    # ====== [UPDATE record_id as string] ======
    updated_lines = []
    for idx, line in enumerate(kept_lines, start=1):
        line = re.sub(r'record_id="[^"]*"', f'record_id="{idx}"', line)
        updated_lines.append(line)


    # ====== [WRITE CLEAN FILE BACK] ======
    with open(info_file, "w") as f:
        f.writelines(updated_lines)

    # ====== [REPORT RESULTS] ======
    print("====== [CLEANUP COMPLETE] ======")
    print(f"Kept {len(updated_lines)} lines, removed {len(removed_lines)} invalid entries.")
    print(f"Backup saved at: {backup_file}")
    if removed_lines:
        print("Removed entries:")
        for rl in removed_lines:
            print(" -", rl.strip())



def main():
    clean_dataset_info()
