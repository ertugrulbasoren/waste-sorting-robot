#!/usr/bin/env python3
import json
import math
import rospy

from std_msgs.msg import String
from waste_sorting_gazebo.msg import Detection2D


class TrackerNode:
    def __init__(self):
        rospy.init_node("tracker_node")

        self.max_distance = int(rospy.get_param("~max_distance", 80))
        self.track_timeout = float(rospy.get_param("~track_timeout", 0.8))

        # FIXED PARAMETERS
        self.pick_line_y = 560
        self.pick_tolerance = 10  # tightened to prevent fake triggers

        # TRACK STATE
        self.next_track_id = 1
        self.tracks = {}

        self.det_sub = rospy.Subscriber(
            "/waste/detection",
            Detection2D,
            self.det_callback,
            queue_size=10
        )

        self.track_pub = rospy.Publisher(
            "/waste/tracked_detection",
            String,
            queue_size=10
        )

        self.pick_pub = rospy.Publisher(
            "/waste/pick_event",
            String,
            queue_size=10
        )

        rospy.Timer(rospy.Duration(0.2), self.cleanup_tracks)

        rospy.loginfo("TRACKER READY")

    def det_callback(self, msg):
        now = rospy.get_time()

        matched_id = None
        best_dist = None

        # 🔥 FIX: prevent duplicate tracks for same object
        for track_id, track in self.tracks.items():
            if track["class_name"].lower() != msg.class_name.lower():
                continue

            dx = msg.center_x - track["center_x"]
            dy = msg.center_y - track["center_y"]
            dist = math.sqrt(dx * dx + dy * dy)

            if dist <= self.max_distance:
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    matched_id = track_id

        # NEW TRACK
        if matched_id is None:
            matched_id = self.next_track_id
            self.next_track_id += 1

            self.tracks[matched_id] = {
                "class_name": msg.class_name.lower(),
                "center_x": msg.center_x,
                "center_y": msg.center_y,
                "x_min": msg.x_min,
                "y_min": msg.y_min,
                "x_max": msg.x_max,
                "y_max": msg.y_max,
                "last_seen": now,
                "picked": False
            }
        else:
            t = self.tracks[matched_id]
            t["class_name"] = msg.class_name.lower()
            t["center_x"] = msg.center_x
            t["center_y"] = msg.center_y
            t["x_min"] = msg.x_min
            t["y_min"] = msg.y_min
            t["x_max"] = msg.x_max
            t["y_max"] = msg.y_max
            t["last_seen"] = now

        track = self.tracks[matched_id]

        # publish tracking info
        self.track_pub.publish(json.dumps({
            "track_id": matched_id,
            "class_name": track["class_name"],
            "center_x": track["center_x"],
            "center_y": track["center_y"],
            "x_min": track["x_min"],
            "y_min": track["y_min"],
            "x_max": track["x_max"],
            "y_max": track["y_max"],
            "picked": track["picked"]
        }))

        # 🔥 SAFE PICK LOGIC
        if not track["picked"]:
            if abs(track["center_y"] - self.pick_line_y) <= self.pick_tolerance:
                track["picked"] = True

                pick_msg = {
                    "track_id": matched_id,
                    "class_name": track["class_name"],
                    "center_x": track["center_x"],
                    "center_y": track["center_y"],
                    "pick_line_y": self.pick_line_y
                }

                self.pick_pub.publish(json.dumps(pick_msg))
                rospy.loginfo("PICK EVENT: %s", track["class_name"])

    def cleanup_tracks(self, _event):
        now = rospy.get_time()
        to_delete = []

        for track_id, track in self.tracks.items():
            if now - track["last_seen"] > self.track_timeout:
                to_delete.append(track_id)

        for tid in to_delete:
            del self.tracks[tid]


if __name__ == "__main__":
    TrackerNode()
    rospy.spin()
