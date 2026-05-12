#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Analyze realistic YOLO full pipeline KPI logs.

Input:
  logs/trials/realistic_yolo_kpi.csv

Output:
  Terminal KPI summary:
    - total events
    - correct count
    - accuracy
    - average confidence
    - class-based accuracy
    - confusion matrix

Usage:
  rosrun waste_sorting_gazebo analyze_realistic_yolo_kpi.py

Optional:
  rosrun waste_sorting_gazebo analyze_realistic_yolo_kpi.py _log_path:=/path/to/file.csv
"""

import csv
import os
from collections import Counter, defaultdict

import rospy


VALID_CLASSES = ["glass", "metal", "paper", "plastic"]


def project_root_from_script() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def safe_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return default


def read_rows(path):
    if not os.path.exists(path):
        raise FileNotFoundError("KPI log file not found: {}".format(path))

    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def print_summary(rows):
    total = len(rows)
    correct = sum(1 for row in rows if row.get("correct") == "True")

    conf_values = [
        safe_float(row.get("confidence", 0.0))
        for row in rows
        if row.get("confidence", "") != ""
    ]

    avg_conf = sum(conf_values) / len(conf_values) if conf_values else 0.0

    by_gt = Counter(row.get("gt_class", "") for row in rows)
    by_correct = Counter(row.get("gt_class", "") for row in rows if row.get("correct") == "True")

    matrix = defaultdict(Counter)

    for row in rows:
        gt_class = row.get("gt_class", "")
        pred_class = row.get("pred_class", "")

        if gt_class in VALID_CLASSES and pred_class in VALID_CLASSES:
            matrix[gt_class][pred_class] += 1

    print("=" * 90)
    print("REALISTIC YOLO FULL PIPELINE KPI")
    print("=" * 90)
    print("Total events   : {}".format(total))
    print("Correct        : {}".format(correct))
    print("Accuracy       : {}".format("{:.2f}%".format(correct / total * 100.0) if total else "0.00%"))
    print("Avg confidence : {:.3f}".format(avg_conf))

    print("-" * 90)
    print("CLASS-BASED ACCURACY")
    print("-" * 90)

    for class_name in VALID_CLASSES:
        n = by_gt[class_name]
        ok = by_correct[class_name]
        rate = (ok / n * 100.0) if n else 0.0
        print("{:<8} {:>4} / {:<4} {:>7.2f}%".format(class_name, ok, n, rate))

    print("-" * 90)
    print("CONFUSION MATRIX")
    print("-" * 90)

    header = "GT/Pred"
    print("{:<10}".format(header) + "".join("{:>10}".format(c) for c in VALID_CLASSES))

    for gt in VALID_CLASSES:
        print("{:<10}".format(gt) + "".join("{:>10}".format(matrix[gt][pred]) for pred in VALID_CLASSES))

    print("=" * 90)

    if total > 0:
        print("SUMMARY SENTENCE")
        print("-" * 90)
        print(
            "The realistic 3D YOLO full pipeline achieved {:.2f}% accuracy "
            "over {} resolved pick events with an average confidence of {:.3f}.".format(
                correct / total * 100.0,
                total,
                avg_conf,
            )
        )
        print("=" * 90)


def main():
    rospy.init_node("analyze_realistic_yolo_kpi", anonymous=True)

    project_root = project_root_from_script()
    default_log_path = os.path.join(project_root, "logs", "trials", "realistic_yolo_kpi.csv")

    log_path = rospy.get_param("~log_path", default_log_path)

    rows = read_rows(log_path)
    print_summary(rows)


if __name__ == "__main__":
    main()
