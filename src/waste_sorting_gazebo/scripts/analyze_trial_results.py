#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Analyze waste sorting trial results.

Input:
  logs/trials/waste_sorting_trial.csv

Output:
  Terminal KPI summary

Usage:
  rosrun waste_sorting_gazebo analyze_trial_results.py

Optional:
  rosrun waste_sorting_gazebo analyze_trial_results.py _csv_path:=/path/to/file.csv
"""

import csv
import os
import sys
from collections import defaultdict
from typing import Dict, List, Optional

import rospy


VALID_CLASSES = ["glass", "metal", "paper", "plastic"]


def parse_bool(value: str) -> Optional[bool]:
    text = str(value).strip().lower()

    if text == "true":
        return True

    if text == "false":
        return False

    return None


def parse_float(value: str) -> Optional[float]:
    try:
        return float(value)
    except Exception:
        return None


def percent(numerator: int, denominator: int) -> float:
    if denominator <= 0:
        return 0.0

    return 100.0 * float(numerator) / float(denominator)


def read_csv_rows(csv_path: str) -> List[Dict[str, str]]:
    rows = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)

        for row in reader:
            rows.append(row)

    return rows


def print_line() -> None:
    print("-" * 90)


def print_kpi_summary(rows: List[Dict[str, str]]) -> None:
    total = len(rows)

    yolo_valid = 0
    yolo_correct = 0

    sorting_valid = 0
    sorting_correct = 0

    overall_valid = 0
    overall_correct = 0

    confidence_values = []

    per_class = {
        cls: {
            "total": 0,
            "yolo_valid": 0,
            "yolo_correct": 0,
            "sorting_valid": 0,
            "sorting_correct": 0,
            "overall_valid": 0,
            "overall_correct": 0,
            "conf_sum": 0.0,
            "conf_count": 0,
        }
        for cls in VALID_CLASSES
    }

    confusion = defaultdict(lambda: defaultdict(int))

    for row in rows:
        gt = str(row.get("ground_truth_class", "")).strip().lower()
        detected = str(row.get("detected_class", "")).strip().lower()
        pick_class = str(row.get("pick_event_class", "")).strip().lower()

        yolo_ok = parse_bool(row.get("yolo_is_correct", ""))
        sorting_ok = parse_bool(row.get("sorting_is_correct", ""))
        overall_ok = parse_bool(row.get("overall_success", ""))

        confidence = parse_float(row.get("detected_confidence", ""))

        if confidence is not None:
            confidence_values.append(confidence)

        if yolo_ok is not None:
            yolo_valid += 1
            if yolo_ok:
                yolo_correct += 1

        if sorting_ok is not None:
            sorting_valid += 1
            if sorting_ok:
                sorting_correct += 1

        if overall_ok is not None:
            overall_valid += 1
            if overall_ok:
                overall_correct += 1

        if gt in VALID_CLASSES:
            per_class[gt]["total"] += 1

            if confidence is not None:
                per_class[gt]["conf_sum"] += confidence
                per_class[gt]["conf_count"] += 1

            if yolo_ok is not None:
                per_class[gt]["yolo_valid"] += 1
                if yolo_ok:
                    per_class[gt]["yolo_correct"] += 1

            if sorting_ok is not None:
                per_class[gt]["sorting_valid"] += 1
                if sorting_ok:
                    per_class[gt]["sorting_correct"] += 1

            if overall_ok is not None:
                per_class[gt]["overall_valid"] += 1
                if overall_ok:
                    per_class[gt]["overall_correct"] += 1

        if gt and detected:
            confusion[gt][detected] += 1

    avg_confidence = 0.0
    if confidence_values:
        avg_confidence = sum(confidence_values) / float(len(confidence_values))

    print()
    print("=" * 90)
    print("WASTE SORTING ROBOT - TRIAL KPI SUMMARY")
    print("=" * 90)
    print()
    print("Total logged trials        : {}".format(total))
    print("Average YOLO confidence    : {:.3f}".format(avg_confidence))
    print()
    print("YOLO correct count         : {} / {}".format(yolo_correct, yolo_valid))
    print("YOLO accuracy              : {:.2f}%".format(percent(yolo_correct, yolo_valid)))
    print()
    print("Sorting correct count      : {} / {}".format(sorting_correct, sorting_valid))
    print("Sorting accuracy           : {:.2f}%".format(percent(sorting_correct, sorting_valid)))
    print()
    print("Overall success count      : {} / {}".format(overall_correct, overall_valid))
    print("Overall success rate       : {:.2f}%".format(percent(overall_correct, overall_valid)))
    print()

    print_line()
    print("CLASS-BASED KPI")
    print_line()
    print(
        "{:<10} {:>8} {:>14} {:>16} {:>16} {:>14}".format(
            "Class",
            "Trials",
            "YOLO Acc.",
            "Sorting Acc.",
            "Overall Acc.",
            "Avg Conf.",
        )
    )
    print_line()

    for cls in VALID_CLASSES:
        item = per_class[cls]

        cls_total = item["total"]
        cls_yolo_acc = percent(item["yolo_correct"], item["yolo_valid"])
        cls_sort_acc = percent(item["sorting_correct"], item["sorting_valid"])
        cls_overall_acc = percent(item["overall_correct"], item["overall_valid"])

        if item["conf_count"] > 0:
            cls_avg_conf = item["conf_sum"] / float(item["conf_count"])
        else:
            cls_avg_conf = 0.0

        print(
            "{:<10} {:>8} {:>13.2f}% {:>15.2f}% {:>15.2f}% {:>14.3f}".format(
                cls,
                cls_total,
                cls_yolo_acc,
                cls_sort_acc,
                cls_overall_acc,
                cls_avg_conf,
            )
        )

    print_line()
    print()

    print_line()
    print("YOLO CONFUSION MATRIX COUNTS")
    print_line()

    header = "{:<10}".format("GT\\Pred")
    for pred_cls in VALID_CLASSES:
        header += "{:>10}".format(pred_cls)
    print(header)

    print_line()

    for gt_cls in VALID_CLASSES:
        line = "{:<10}".format(gt_cls)
        for pred_cls in VALID_CLASSES:
            line += "{:>10}".format(confusion[gt_cls][pred_cls])
        print(line)

    print_line()
    print()


def main() -> None:
    rospy.init_node("analyze_trial_results", anonymous=True)

    default_csv_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "..",
            "logs",
            "trials",
            "waste_sorting_trial.csv",
        )
    )

    csv_path = rospy.get_param("~csv_path", default_csv_path)

    if not os.path.exists(csv_path):
        rospy.logerr("[ANALYZE TRIAL RESULTS] CSV file not found: %s", csv_path)
        rospy.logerr("[ANALYZE TRIAL RESULTS] Run full_system_dataset.launch first.")
        sys.exit(1)

    rows = read_csv_rows(csv_path)

    if not rows:
        rospy.logerr("[ANALYZE TRIAL RESULTS] CSV file is empty: %s", csv_path)
        sys.exit(1)

    print_kpi_summary(rows)


if __name__ == "__main__":
    main()
