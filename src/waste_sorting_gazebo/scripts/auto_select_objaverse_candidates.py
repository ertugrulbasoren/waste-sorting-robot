#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Auto-select promising Objaverse waste asset candidates.

Input:
  asset_candidates/objaverse_waste_candidates.csv

Output:
  asset_candidates/selected_objaverse_uids.csv

Purpose:
  The keyword search creates noisy candidates.
  This script scores candidates and selects the most promising ones per class.

Important:
  This is still a pre-selection step.
  Final visual quality should be checked after downloading/rendering.
"""

import argparse
import csv
import os
import re
from collections import defaultdict
from typing import Dict, List


VALID_CLASSES = ["glass", "metal", "paper", "plastic"]


FIELDNAMES = [
    "class_name",
    "uid",
    "asset_name",
    "license",
    "matched_phrase",
    "score",
    "source_dataset",
    "source",
    "file_identifier_or_url",
    "raw_asset_folder",
    "notes",
]


POSITIVE_TERMS: Dict[str, List[str]] = {
    "plastic": [
        "plastic bottle",
        "water bottle",
        "soda bottle",
        "pet bottle",
        "plastic container",
        "plastic cup",
        "plastic bag",
        "detergent bottle",
        "shampoo bottle",
        "bottle",
        "bag",
        "container",
    ],
    "metal": [
        "aluminium can",
        "aluminum can",
        "soda can",
        "tin can",
        "coke can",
        "cola can",
        "beer can",
        "food can",
        "metal can",
        "crushed can",
        "dented can",
        "can",
    ],
    "paper": [
        "cardboard box",
        "cardboard",
        "paper bag",
        "paper cup",
        "crumpled paper",
        "torn paper",
        "paper ball",
        "newspaper",
        "carton",
        "box",
    ],
    "glass": [
        "glass bottle",
        "wine bottle",
        "beer bottle",
        "glass jar",
        "glass container",
        "broken bottle",
        "broken glass",
        "glass shard",
        "bottle",
        "jar",
    ],
}


STRONG_POSITIVE_TERMS: Dict[str, List[str]] = {
    "plastic": [
        "plastic water bottle",
        "plastic bottle",
        "water bottle",
        "pet bottle",
        "plastic bag",
    ],
    "metal": [
        "aluminium can",
        "aluminum can",
        "soda can",
        "tin can",
        "coke can",
        "beer can",
        "crushed can",
        "dented can",
    ],
    "paper": [
        "cardboard box",
        "cardboard",
        "crumpled paper",
        "paper bag",
        "paper cup",
    ],
    "glass": [
        "beer bottle",
        "wine bottle",
        "glass bottle",
        "glass jar",
        "broken glass",
        "glass shard",
    ],
}


NEGATIVE_TERMS = [
    "person",
    "human",
    "face mask",
    "mask",
    "character",
    "anime",
    "cartoon",
    "weapon",
    "gun",
    "knife",
    "sword",
    "building",
    "warehouse",
    "city",
    "street",
    "architecture",
    "church",
    "bridge",
    "abutment",
    "room",
    "house",
    "chair",
    "table",
    "sofa",
    "car",
    "vehicle",
    "truck",
    "airplane",
    "animal",
    "tree",
    "plant",
    "toy",
    "game",
    "logo",
    "font",
    "sign",
    "statue",
    "sculpture",
    "terrain",
    "machine vision",
    "zivid",
    "geometry metal complexes",
    "arknights",
    "vr cromo",
    "cannonball",
]


BAD_LICENSE_TERMS = [
    "by-nc",
    "by-nc-sa",
    "noncommercial",
    "non-commercial",
]


GOOD_LICENSE_TERMS = [
    "cc0",
    "public domain",
    "by",
]


def project_root_from_script() -> str:
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..")
    )


def clean(value) -> str:
    return str(value or "").replace("\n", " ").replace("\r", " ").strip()


def normalize(value) -> str:
    text = clean(value).lower()
    text = re.sub(r"\s+", " ", text)
    return text


def contains_phrase(text: str, phrase: str) -> bool:
    phrase = phrase.lower().strip()
    pattern = r"\b{}\b".format(re.escape(phrase))
    return re.search(pattern, text) is not None


def score_candidate(row: Dict[str, str]) -> int:
    class_name = normalize(row.get("class_guess", ""))
    name = normalize(row.get("name", ""))
    matched_phrase = normalize(row.get("matched_phrase", ""))
    license_name = normalize(row.get("license", ""))
    preview = normalize(row.get("search_text_preview", ""))

    full_text = " ".join([name, matched_phrase, preview])

    score = 0

    if class_name not in VALID_CLASSES:
        return -9999

    # Hard negative terms.
    for term in NEGATIVE_TERMS:
        if contains_phrase(full_text, term):
            score -= 100

    # Positive phrase in matched_phrase is useful.
    if matched_phrase:
        score += 10

    # Strong positive terms in name are very valuable.
    for term in STRONG_POSITIVE_TERMS[class_name]:
        if contains_phrase(name, term):
            score += 35

    # Positive terms anywhere in searchable text.
    for term in POSITIVE_TERMS[class_name]:
        if contains_phrase(full_text, term):
            score += 12

    # Prefer direct object-like names.
    if len(name) > 0:
        score += 5

    # Penalize very generic or suspicious names.
    if name in ["", "a trnio model", "3d scan", "model"]:
        score -= 20

    if len(name) > 80:
        score -= 10

    # License scoring.
    for term in BAD_LICENSE_TERMS:
        if term in license_name:
            score -= 25

    for term in GOOD_LICENSE_TERMS:
        if license_name == term:
            score += 10

    # Some by-nc is not impossible for internal thesis/demo,
    # but we do not prefer it for repo/public redistribution.
    if "by-nc" in license_name:
        score -= 20

    return score


def read_candidates(input_csv: str) -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []

    with open(input_csv, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    return rows


def write_selected(output_csv: str, selected_rows: List[Dict[str, str]]) -> None:
    os.makedirs(os.path.dirname(output_csv), exist_ok=True)

    with open(output_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()

        for row in selected_rows:
            class_name = clean(row.get("class_guess", "")).lower()
            uid = clean(row.get("uid", ""))
            name = clean(row.get("name", ""))
            license_name = clean(row.get("license", ""))
            matched_phrase = clean(row.get("matched_phrase", ""))
            score = clean(row.get("_score", ""))
            source_dataset = clean(row.get("source_dataset", ""))
            source = clean(row.get("source", ""))
            file_id = clean(row.get("file_identifier_or_url", ""))

            raw_asset_folder = "{}_{}".format(
                class_name,
                uid[:8] if uid else "unknown",
            )

            writer.writerow(
                {
                    "class_name": class_name,
                    "uid": uid,
                    "asset_name": name,
                    "license": license_name,
                    "matched_phrase": matched_phrase,
                    "score": score,
                    "source_dataset": source_dataset,
                    "source": source,
                    "file_identifier_or_url": file_id,
                    "raw_asset_folder": raw_asset_folder,
                    "notes": "",
                }
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--per-class", type=int, default=10)
    parser.add_argument("--min-score", type=int, default=15)
    parser.add_argument("--input", type=str, default="")
    parser.add_argument("--output", type=str, default="")

    args = parser.parse_args()

    project_root = project_root_from_script()

    input_csv = args.input or os.path.join(
        project_root,
        "asset_candidates",
        "objaverse_waste_candidates.csv",
    )

    output_csv = args.output or os.path.join(
        project_root,
        "asset_candidates",
        "selected_objaverse_uids.csv",
    )

    if not os.path.exists(input_csv):
        raise FileNotFoundError("Input candidate CSV not found: {}".format(input_csv))

    rows = read_candidates(input_csv)

    grouped = defaultdict(list)

    for row in rows:
        class_name = clean(row.get("class_guess", "")).lower()

        if class_name not in VALID_CLASSES:
            continue

        score = score_candidate(row)
        row["_score"] = str(score)

        if score < args.min_score:
            continue

        grouped[class_name].append(row)

    selected: List[Dict[str, str]] = []

    print("=" * 100)
    print("AUTO SELECT OBJAVERSE WASTE CANDIDATES")
    print("=" * 100)

    for class_name in VALID_CLASSES:
        candidates = grouped[class_name]
        candidates.sort(key=lambda r: int(r.get("_score", "0")), reverse=True)

        chosen = candidates[: args.per_class]
        selected.extend(chosen)

        print()
        print("CLASS:", class_name)
        print("selected:", len(chosen), "from scored candidates:", len(candidates))

        for i, row in enumerate(chosen, start=1):
            print(
                "{:02d}. score={} | {} | {} | {}".format(
                    i,
                    row.get("_score", ""),
                    clean(row.get("matched_phrase", "")),
                    clean(row.get("name", "")),
                    clean(row.get("license", "")),
                )
            )

    write_selected(output_csv, selected)

    print()
    print("=" * 100)
    print("[OK] selected CSV written:")
    print(output_csv)
    print("[OK] total selected:", len(selected))
    print("=" * 100)


if __name__ == "__main__":
    main()
