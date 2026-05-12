#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Download selected Objaverse assets into the project raw_assets folder.

Input:
  asset_candidates/selected_objaverse_uids.csv

Output:
  src/waste_sorting_gazebo/raw_assets/<class>/<raw_asset_folder>/

Each downloaded asset folder contains:
  source.<original_ext>
  metadata.json

Important:
  This script downloads raw 3D assets only.
  It does not convert them to Gazebo SDF yet.
"""

import csv
import json
import os
import shutil
import sys
import traceback
from typing import Dict, List, Tuple


VALID_CLASSES = {"glass", "metal", "paper", "plastic"}


def project_root_from_script() -> str:
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..")
    )


def clean(value) -> str:
    return str(value or "").replace("\n", " ").replace("\r", " ").strip()


def read_selected_csv(path: str) -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []

    with open(path, newline="") as f:
        reader = csv.DictReader(f)

        for row in reader:
            class_name = clean(row.get("class_name", "")).lower()
            uid = clean(row.get("uid", ""))

            if class_name not in VALID_CLASSES:
                continue

            if not uid:
                continue

            rows.append(row)

    return rows


def safe_folder_name(value: str) -> str:
    text = clean(value).lower()
    allowed = []

    for ch in text:
        if ch.isalnum():
            allowed.append(ch)
        elif ch in ["_", "-", "."]:
            allowed.append(ch)
        elif ch.isspace():
            allowed.append("_")

    result = "".join(allowed).strip("_")

    if not result:
        result = "asset"

    return result[:80]


def get_extension(path: str) -> str:
    _, ext = os.path.splitext(path)

    if ext:
        return ext.lower()

    return ".glb"


def copy_downloaded_file(src_path: str, dst_dir: str) -> str:
    os.makedirs(dst_dir, exist_ok=True)

    ext = get_extension(src_path)
    dst_path = os.path.join(dst_dir, "source{}".format(ext))

    shutil.copy2(src_path, dst_path)

    return dst_path


def download_objects_objaverse(uids: List[str], download_processes: int) -> Dict[str, str]:
    """
    Uses Objaverse 1.0 API.

    Expected return:
      {uid: local_file_path}
    """

    import objaverse  # type: ignore

    try:
        objects = objaverse.load_objects(
            uids=uids,
            download_processes=download_processes,
        )
    except TypeError:
        # Compatibility fallback for slightly different API versions.
        objects = objaverse.load_objects(uids=uids)

    if not isinstance(objects, dict):
        raise RuntimeError("objaverse.load_objects returned unsupported type: {}".format(type(objects)))

    return objects


def write_metadata(
    metadata_path: str,
    row: Dict[str, str],
    downloaded_source_path: str,
    copied_source_path: str,
) -> None:
    metadata = {
        "class_name": clean(row.get("class_name", "")).lower(),
        "uid": clean(row.get("uid", "")),
        "asset_name": clean(row.get("asset_name", "")),
        "license": clean(row.get("license", "")),
        "matched_phrase": clean(row.get("matched_phrase", "")),
        "score": clean(row.get("score", "")),
        "source_dataset": clean(row.get("source_dataset", "")),
        "source": clean(row.get("source", "")),
        "file_identifier_or_url": clean(row.get("file_identifier_or_url", "")),
        "raw_asset_folder": clean(row.get("raw_asset_folder", "")),
        "downloaded_source_path": downloaded_source_path,
        "copied_source_path": copied_source_path,
        "notes": clean(row.get("notes", "")),
    }

    with open(metadata_path, "w") as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)


def write_asset_credits(project_root: str, rows: List[Dict[str, str]]) -> None:
    credits_path = os.path.join(project_root, "ASSET_CREDITS.md")

    with open(credits_path, "w") as f:
        f.write("# Asset Credits\n\n")
        f.write("This file records selected Objaverse assets used for realistic 3D waste simulation.\n\n")
        f.write("| Class | Asset Name | UID | License | Matched Phrase | Raw Folder |\n")
        f.write("|---|---|---|---|---|---|\n")

        for row in rows:
            f.write(
                "| {} | {} | {} | {} | {} | {} |\n".format(
                    clean(row.get("class_name", "")),
                    clean(row.get("asset_name", "")).replace("|", "/"),
                    clean(row.get("uid", "")),
                    clean(row.get("license", "")),
                    clean(row.get("matched_phrase", "")),
                    clean(row.get("raw_asset_folder", "")),
                )
            )


def main() -> None:
    project_root = project_root_from_script()

    selected_csv = os.path.join(
        project_root,
        "asset_candidates",
        "selected_objaverse_uids.csv",
    )

    raw_assets_root = os.path.join(
        project_root,
        "src",
        "waste_sorting_gazebo",
        "raw_assets",
    )

    if not os.path.exists(selected_csv):
        print("[ERROR] selected CSV not found:", selected_csv)
        sys.exit(1)

    rows = read_selected_csv(selected_csv)

    if not rows:
        print("[ERROR] no selected rows found in:", selected_csv)
        sys.exit(1)

    uid_to_row: Dict[str, Dict[str, str]] = {}
    uids: List[str] = []

    for row in rows:
        uid = clean(row.get("uid", ""))
        uid_to_row[uid] = row
        uids.append(uid)

    print("=" * 100)
    print("DOWNLOAD SELECTED OBJAVERSE ASSETS")
    print("=" * 100)
    print("[INFO] selected_csv:", selected_csv)
    print("[INFO] raw_assets_root:", raw_assets_root)
    print("[INFO] total selected:", len(uids))
    print("=" * 100)

    try:
        objects = download_objects_objaverse(uids, download_processes=4)
    except Exception as exc:
        print("[ERROR] Objaverse download failed:", str(exc))
        print(traceback.format_exc())
        sys.exit(1)

    copied_count = 0
    failed_count = 0

    for uid in uids:
        row = uid_to_row[uid]

        class_name = clean(row.get("class_name", "")).lower()
        raw_folder = clean(row.get("raw_asset_folder", ""))

        if not raw_folder:
            raw_folder = "{}_{}".format(class_name, uid[:8])

        raw_folder = safe_folder_name(raw_folder)

        dst_dir = os.path.join(raw_assets_root, class_name, raw_folder)

        downloaded_path = objects.get(uid)

        if not downloaded_path or not os.path.exists(downloaded_path):
            print("[WARN] missing downloaded file for uid:", uid)
            failed_count += 1
            continue

        try:
            copied_path = copy_downloaded_file(downloaded_path, dst_dir)

            metadata_path = os.path.join(dst_dir, "metadata.json")
            write_metadata(
                metadata_path=metadata_path,
                row=row,
                downloaded_source_path=downloaded_path,
                copied_source_path=copied_path,
            )

            print("[OK] {} -> {}".format(uid, copied_path))
            copied_count += 1

        except Exception as exc:
            print("[WARN] failed to copy uid={} error={}".format(uid, str(exc)))
            failed_count += 1

    write_asset_credits(project_root, rows)

    print("=" * 100)
    print("[OK] Download completed.")
    print("[OK] copied:", copied_count)
    print("[OK] failed:", failed_count)
    print("[OK] ASSET_CREDITS.md updated.")
    print("=" * 100)


if __name__ == "__main__":
    main()
