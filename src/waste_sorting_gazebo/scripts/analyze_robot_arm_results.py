#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Analyze robot arm execution result logs.

Input:
  logs/trials/robot_arm_results.csv

Output:
  Terminal KPI summary:
    - total cycles
    - successful cycles
    - failed cycles
    - success rate
    - average cycle time
    - class-based success rate

Usage:
  rosrun waste_sorting_gazebo analyze_robot_arm_results.py

Optional:
  rosrun waste_sorting_gazebo analyze_robot_arm_results.py _log_path:=/path/to/robot_arm_results.csv
"""

import csv
import os
from collections import Counter

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
        raise FileNotFoundError("Robot arm result log file not found: {}".format(path))

    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def print_summary(rows):
    total = len(rows)
    success = sum(1 for row in rows if row.get("success") == "True")
    failed = total - success

    cycle_values = [
        safe_float(row.get("cycle_time_sec", 0.0))
        for row in rows
        if row.get("cycle_time_sec", "") != ""
    ]

    avg_cycle = sum(cycle_values) / len(cycle_values) if cycle_values else 0.0
    min_cycle = min(cycle_values) if cycle_values else 0.0
    max_cycle = max(cycle_values) if cycle_values else 0.0

    by_class = Counter(row.get("class_name", "") for row in rows)
    by_class_success = Counter(row.get("class_name", "") for row in rows if row.get("success") == "True")

    print("=" * 90)
    print("ROBOT ARM EXECUTION KPI")
    print("=" * 90)
    print("Total cycles      : {}".format(total))
    print("Successful cycles : {}".format(success))
    print("Failed cycles     : {}".format(failed))
    print("Success rate      : {}".format("{:.2f}%".format(success / total * 100.0) if total else "0.00%"))
    print("Avg cycle time    : {:.3f} sec".format(avg_cycle))
    print("Min cycle time    : {:.3f} sec".format(min_cycle))
    print("Max cycle time    : {:.3f} sec".format(max_cycle))

    print("-" * 90)
    print("CLASS-BASED ROBOT ARM SUCCESS")
    print("-" * 90)

    for class_name in VALID_CLASSES:
        n = by_class[class_name]
        ok = by_class_success[class_name]
        rate = (ok / n * 100.0) if n else 0.0
        print("{:<8} {:>4} / {:<4} {:>7.2f}%".format(class_name, ok, n, rate))

    print("-" * 90)
    print("EXECUTOR COUNTS")
    print("-" * 90)

    executor_counts = Counter(row.get("executor", "") for row in rows)
    for executor, count in executor_counts.most_common():
        print("{:<32} {}".format(executor or "unknown", count))

    print("=" * 90)

    if total > 0:
        print("SUMMARY SENTENCE")
        print("-" * 90)
        print(
            "The robot arm executor completed {} out of {} cycles successfully "
            "with a {:.2f}% success rate and an average cycle time of {:.3f} seconds.".format(
                success,
                total,
                success / total * 100.0,
                avg_cycle,
            )
        )
        print("=" * 90)


def main():
    rospy.init_node("analyze_robot_arm_results", anonymous=True)

    project_root = project_root_from_script()
    default_log_path = os.path.join(project_root, "logs", "trials", "robot_arm_results.csv")

    log_path = rospy.get_param("~log_path", default_log_path)

    rows = read_rows(log_path)
    print_summary(rows)


if __name__ == "__main__":
    main()
