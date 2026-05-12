#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Low-memory Objaverse waste candidate search.

This script searches Objaverse annotations in small chunks to avoid RAM crashes.

Output:
  asset_candidates/objaverse_waste_candidates.csv

It does NOT download 3D meshes yet.
It only creates a candidate CSV for manual review.

Important fix:
  This version avoids weak substring matches such as:
    "can" inside "Cannonball"
    "can" inside "Scaniverse"
    "bottle" without plastic/glass context
    "paper" without trash/material context
"""

import argparse
import csv
import gc
import json
import os
import re
import sys
import time
from collections import Counter
from typing import Any, Dict, List


CLASS_PHRASES: Dict[str, List[str]] = {
    "plastic": [
        "plastic bottle",
        "crushed plastic bottle",
        "twisted plastic bottle",
        "plastic waste",
        "plastic container",
        "plastic cup",
        "plastic bag",
        "water bottle",
        "soda bottle",
        "detergent bottle",
        "shampoo bottle",
        "milk bottle",
        "bottle plastic",
        "pet bottle",
    ],
    "metal": [
        "crushed can",
        "dented can",
        "soda can",
        "tin can",
        "aluminum can",
        "aluminium can",
        "metal can",
        "food can",
        "rusty can",
        "beer can",
        "coke can",
        "cola can",
        "can soda",
        "can aluminium",
        "can aluminum",
    ],
    "paper": [
        "crumpled paper",
        "crushed paper",
        "torn paper",
        "paper trash",
        "paper waste",
        "paper ball",
        "paper cup",
        "paper bag",
        "cardboard",
        "cardboard box",
        "newspaper",
        "carton box",
        "corrugated cardboard",
    ],
    "glass": [
        "glass bottle",
        "broken glass",
        "glass shard",
        "broken bottle",
        "glass jar",
        "wine bottle",
        "beer bottle",
        "green glass bottle",
        "brown glass bottle",
        "bottle glass",
        "jar glass",
        "glass container",
    ],
}


NEGATIVE_KEYWORDS = [
    "person",
    "human",
    "character",
    "anime",
    "cartoon",
    "weapon",
    "gun",
    "knife",
    "sword",
    "building",
    "house",
    "room",
    "city",
    "street",
    "car",
    "vehicle",
    "animal",
    "tree",
    "plant",
    "toy",
    "game",
    "chair",
    "table",
    "sofa",
    "logo",
    "font",
    "sign",
    "statue",
    "sculpture",
    "terrain",
    "rock",
    "church",
    "bridge",
    "architecture",
    "scaniverse",
    "trnio",
    "abutment",
    "cannonball",
]


FIELDNAMES = [
    "class_guess",
    "matched_phrase",
    "uid",
    "name",
    "license",
    "source_dataset",
    "source",
    "file_identifier_or_url",
    "selected",
    "quality_notes",
    "raw_asset_folder",
    "search_text_preview",
]


def project_root_from_script() -> str:
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..")
    )


def clean_space(text: str) -> str:
    return re.sub(r"\s+", " ", str(text)).strip()


def normalize_text(value: Any) -> str:
    if value is None:
        return ""

    if isinstance(value, float):
        return ""

    if isinstance(value, (dict, list, tuple)):
        try:
            return json.dumps(value, ensure_ascii=False)
        except Exception:
            return str(value)

    return str(value)


def lower_text(text: str) -> str:
    return clean_space(text).lower()


def extract_field(row: Dict[str, Any], names: List[str]) -> str:
    for name in names:
        if name in row and row.get(name) is not None:
            value = clean_space(normalize_text(row.get(name)))
            if value:
                return value
    return ""


def row_to_search_text(row: Dict[str, Any]) -> str:
    preferred_fields = [
        "name",
        "title",
        "description",
        "tags",
        "categories",
        "caption",
        "metadata",
        "license",
        "uid",
    ]

    parts = []

    for field in preferred_fields:
        if field in row:
            parts.append(normalize_text(row.get(field)))

    if not parts:
        for value in row.values():
            parts.append(normalize_text(value))

    return lower_text(" ".join(parts))


def contains_negative_keyword(text: str) -> bool:
    for keyword in NEGATIVE_KEYWORDS:
        pattern = r"\b{}\b".format(re.escape(keyword.lower()))
        if re.search(pattern, text):
            return True
    return False


def phrase_matches(text: str, phrase: str) -> bool:
    """
    Strict phrase matching with word boundaries.
    Prevents weak matches like:
      can -> cannonball
      can -> scaniverse
    """

    phrase = phrase.lower().strip()
    pattern = r"\b{}\b".format(re.escape(phrase))
    return re.search(pattern, text) is not None


def make_candidate(row: Dict[str, Any], class_name: str, matched_phrase: str) -> Dict[str, str]:
    uid = extract_field(row, ["uid", "object_id", "sha256", "id"])
    name = extract_field(row, ["name", "title", "displayName"])
    license_name = extract_field(row, ["license", "license_name", "licenseName"])
    source = extract_field(row, ["source", "repo", "provider"])
    file_identifier = extract_field(row, ["fileIdentifier", "file_identifier", "url", "download_url"])

    if not uid:
        uid = extract_field(row, ["sha256", "fileIdentifier"])

    search_text = row_to_search_text(row)

    return {
        "class_guess": class_name,
        "matched_phrase": matched_phrase,
        "uid": uid,
        "name": name,
        "license": license_name,
        "source_dataset": "objaverse-1.0",
        "source": source,
        "file_identifier_or_url": file_identifier,
        "selected": "False",
        "quality_notes": "",
        "raw_asset_folder": "",
        "search_text_preview": search_text[:300],
    }


def append_candidates(csv_path: str, candidates: List[Dict[str, str]]) -> None:
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    file_exists = os.path.exists(csv_path)
    file_empty = (not file_exists) or os.path.getsize(csv_path) == 0

    with open(csv_path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)

        if file_empty:
            writer.writeheader()

        for row in candidates:
            writer.writerow(row)


def load_existing_uids(csv_path: str) -> set:
    existing = set()

    if not os.path.exists(csv_path):
        return existing

    try:
        with open(csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                uid = row.get("uid", "")
                class_guess = row.get("class_guess", "")
                if uid and class_guess:
                    existing.add((class_guess, uid))
    except Exception:
        return existing

    return existing


def count_existing(csv_path: str) -> Counter:
    counter = Counter()

    if not os.path.exists(csv_path):
        return counter

    try:
        with open(csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                cls = row.get("class_guess", "")
                if cls:
                    counter[cls] += 1
    except Exception:
        return counter

    return counter


def scan_annotation_rows(
    annotations: Dict[str, Any],
    counts: Counter,
    existing_pairs: set,
    max_per_class: int,
    include_negative: bool,
) -> List[Dict[str, str]]:
    new_candidates: List[Dict[str, str]] = []

    for uid, value in annotations.items():
        if isinstance(value, dict):
            row = dict(value)
            row.setdefault("uid", uid)
        else:
            row = {"uid": uid, "value": value}

        search_text = row_to_search_text(row)

        if not search_text:
            continue

        if not include_negative and contains_negative_keyword(search_text):
            continue

        for class_name, phrases in CLASS_PHRASES.items():
            if counts[class_name] >= max_per_class:
                continue

            matched_phrase = ""

            for phrase in phrases:
                if phrase_matches(search_text, phrase):
                    matched_phrase = phrase
                    break

            if not matched_phrase:
                continue

            pair = (class_name, str(uid))
            if pair in existing_pairs:
                continue

            candidate = make_candidate(row, class_name, matched_phrase)
            new_candidates.append(candidate)

            existing_pairs.add(pair)
            counts[class_name] += 1
            break

    return new_candidates


def all_classes_done(counts: Counter, max_per_class: int) -> bool:
    for class_name in CLASS_PHRASES.keys():
        if counts[class_name] < max_per_class:
            return False
    return True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-per-class", type=int, default=50)
    parser.add_argument("--chunk-size", type=int, default=500)
    parser.add_argument("--max-uids", type=int, default=200000)
    parser.add_argument("--start-index", type=int, default=0)
    parser.add_argument("--include-negative", action="store_true")
    parser.add_argument("--output", type=str, default="")
    parser.add_argument("--fresh", action="store_true")

    args = parser.parse_args()

    project_root = project_root_from_script()

    output_csv = args.output
    if not output_csv:
        output_csv = os.path.join(project_root, "asset_candidates", "objaverse_waste_candidates.csv")

    if args.fresh and os.path.exists(output_csv):
        os.remove(output_csv)

    print("=" * 90)
    print("LOW-MEMORY STRICT OBJAVERSE WASTE CANDIDATE SEARCH")
    print("=" * 90)
    print("[INFO] project_root:", project_root)
    print("[INFO] output_csv:", output_csv)
    print("[INFO] max_per_class:", args.max_per_class)
    print("[INFO] chunk_size:", args.chunk_size)
    print("[INFO] max_uids:", args.max_uids)
    print("[INFO] start_index:", args.start_index)
    print("=" * 90)

    try:
        import objaverse  # type: ignore
    except Exception as exc:
        print("[ERROR] Could not import objaverse:", str(exc))
        print("[ERROR] Install it first from external_tools/objaverse-xl:")
        print("        python3 -m pip install --user -e .")
        sys.exit(1)

    print("[INFO] Loading Objaverse UID list...")
    uids = objaverse.load_uids()

    total_uids = len(uids)
    print("[INFO] total available uids:", total_uids)

    end_index = min(total_uids, args.max_uids)

    if args.start_index >= end_index:
        print("[ERROR] start_index >= end_index")
        sys.exit(1)

    counts = count_existing(output_csv)
    existing_pairs = load_existing_uids(output_csv)

    print("[INFO] existing counts:", dict(counts))

    processed = 0
    start_time = time.time()

    index = args.start_index

    while index < end_index:
        if all_classes_done(counts, args.max_per_class):
            print("[INFO] target count reached for all classes.")
            break

        chunk_end = min(index + args.chunk_size, end_index)
        uid_chunk = uids[index:chunk_end]

        print(
            "[INFO] chunk {}:{} / {} counts={}".format(
                index,
                chunk_end,
                end_index,
                dict(counts),
            )
        )

        try:
            annotations = objaverse.load_annotations(uid_chunk)
        except Exception as exc:
            print("[WARN] load_annotations failed for chunk {}:{} -> {}".format(index, chunk_end, str(exc)))
            index = chunk_end
            continue

        new_candidates = scan_annotation_rows(
            annotations=annotations,
            counts=counts,
            existing_pairs=existing_pairs,
            max_per_class=args.max_per_class,
            include_negative=args.include_negative,
        )

        if new_candidates:
            append_candidates(output_csv, new_candidates)
            print("[INFO] wrote new candidates:", len(new_candidates))

        processed += len(uid_chunk)

        del annotations
        del new_candidates
        gc.collect()

        index = chunk_end

    elapsed = time.time() - start_time

    print("=" * 90)
    print("[OK] Search finished.")
    print("[OK] output_csv:", output_csv)
    print("[OK] final counts:", dict(counts))
    print("[OK] processed uid count:", processed)
    print("[OK] elapsed sec:", round(elapsed, 2))
    print("=" * 90)


if __name__ == "__main__":
    main()
