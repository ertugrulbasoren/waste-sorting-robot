#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Create human-readable review files from Objaverse waste candidate CSV.

Input:
  asset_candidates/objaverse_waste_candidates.csv

Outputs:
  asset_candidates/review_glass.txt
  asset_candidates/review_metal.txt
  asset_candidates/review_paper.txt
  asset_candidates/review_plastic.txt

Purpose:
  The automatic Objaverse keyword search creates candidate assets.
  This script makes manual review easier before downloading selected assets.
"""

import csv
import os
from collections import defaultdict
from typing import Dict, List


VALID_CLASSES = ["glass", "metal", "paper", "plastic"]


def project_root_from_script() -> str:
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..")
    )


def clean(value: str) -> str:
    return str(value or "").replace("\n", " ").replace("\r", " ").strip()


def main() -> None:
    project_root = project_root_from_script()

    input_csv = os.path.join(
        project_root,
        "asset_candidates",
        "objaverse_waste_candidates.csv",
    )

    output_dir = os.path.join(project_root, "asset_candidates")

    if not os.path.exists(input_csv):
        raise FileNotFoundError("Candidate CSV not found: {}".format(input_csv))

    rows_by_class: Dict[str, List[dict]] = defaultdict(list)

    with open(input_csv, newline="") as f:
        reader = csv.DictReader(f)

        for row in reader:
            class_guess = clean(row.get("class_guess", "")).lower()

            if class_guess not in VALID_CLASSES:
                continue

            rows_by_class[class_guess].append(row)

    os.makedirs(output_dir, exist_ok=True)

    for class_name in VALID_CLASSES:
        rows = rows_by_class.get(class_name, [])

        out_path = os.path.join(output_dir, "review_{}.txt".format(class_name))

        with open(out_path, "w") as f:
            f.write("=" * 100 + "\n")
            f.write("OBJAVERSE CANDIDATE REVIEW - {}\n".format(class_name.upper()))
            f.write("=" * 100 + "\n\n")
            f.write("Review rule:\n")
            f.write("  Keep assets that are visually recognizable as {} waste.\n".format(class_name))
            f.write("  Avoid irrelevant models, buildings, characters, logos, masks, machines, etc.\n")
            f.write("  Prefer licenses: by, cc0, public domain. Be careful with by-nc.\n\n")

            for i, row in enumerate(rows, start=1):
                f.write("-" * 100 + "\n")
                f.write("INDEX          : {}\n".format(i))
                f.write("CLASS          : {}\n".format(clean(row.get("class_guess", ""))))
                f.write("MATCHED PHRASE : {}\n".format(clean(row.get("matched_phrase", ""))))
                f.write("UID            : {}\n".format(clean(row.get("uid", ""))))
                f.write("NAME           : {}\n".format(clean(row.get("name", ""))))
                f.write("LICENSE        : {}\n".format(clean(row.get("license", ""))))
                f.write("SOURCE         : {}\n".format(clean(row.get("source", ""))))
                f.write("URL/FILE ID    : {}\n".format(clean(row.get("file_identifier_or_url", ""))))
                f.write("PREVIEW        : {}\n".format(clean(row.get("search_text_preview", ""))))
                f.write("\n")

        print("[OK] wrote:", out_path, "count:", len(rows))

    print("[OK] Review files generated.")


if __name__ == "__main__":
    main()
