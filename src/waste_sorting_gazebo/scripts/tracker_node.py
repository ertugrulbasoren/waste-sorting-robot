#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import sys
import threading
import traceback
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

import rospy
from std_msgs.msg import String

from waste_sorting_gazebo.msg import Detection2D, Detection2DArray


VALID_CLASSES = {"glass", "metal", "paper", "plastic"}


@dataclass
class Detection:
    class_name: str
    confidence: float
    x_min: int
    y_min: int
    x_max: int
    y_max: int
    center_x: int
    center_y: int
    stamp_sec: float


@dataclass
class Track:
    track_id: int
    class_name: str
    center_x: float
    center_y: float
    confidence: float
    created_sec: float
    last_seen_sec: float

    x_min: int = 0
    y_min: int = 0
    x_max: int = 0
    y_max: int = 0

    hits: int = 1
    misses: int = 0
    state: str = "NEW"

    picked: bool = False
    event_emitted: bool = False
    confirm_logged: bool = False

    previous_center_x: Optional[float] = None
    previous_center_y: Optional[float] = None

    history: List[Tuple[float, float, float]] = field(default_factory=list)

    def update(self, det: Detection) -> None:
        self.previous_center_x = self.center_x
        self.previous_center_y = self.center_y

        alpha = 0.65
        self.center_x = alpha * float(det.center_x) + (1.0 - alpha) * float(self.center_x)
        self.center_y = alpha * float(det.center_y) + (1.0 - alpha) * float(self.center_y)

        self.confidence = float(det.confidence)
        self.x_min = int(det.x_min)
        self.y_min = int(det.y_min)
        self.x_max = int(det.x_max)
        self.y_max = int(det.y_max)
        self.last_seen_sec = float(det.stamp_sec)

        self.hits += 1
        self.misses = 0

        self.history.append((self.last_seen_sec, self.center_x, self.center_y))
        if len(self.history) > 20:
            self.history.pop(0)

    def mark_missed(self) -> None:
        self.misses += 1

    def age_sec(self, now_sec: float) -> float:
        return float(now_sec) - float(self.created_sec)

    def time_since_seen_sec(self, now_sec: float) -> float:
        return float(now_sec) - float(self.last_seen_sec)


class WasteTrackerNode:
    def __init__(self) -> None:
        rospy.init_node("tracker_node", anonymous=False)

        self.lock = threading.RLock()

        self.detections_topic = rospy.get_param("~detections_topic", "/waste/detections")
        self.legacy_detection_topic = rospy.get_param("~legacy_detection_topic", "/waste/detection")
        self.tracked_topic = rospy.get_param("~tracked_topic", "/waste/tracked_detection")
        self.pick_event_topic = rospy.get_param("~pick_event_topic", "/waste/pick_event")
        self.sample_topic = rospy.get_param("~sample_topic", "/dataset_camera/current_sample")

        self.max_match_distance_px = float(rospy.get_param("~max_match_distance_px", 140.0))
        self.max_track_age_sec = float(rospy.get_param("~max_track_age_sec", 1.5))
        self.min_confirmed_hits = int(rospy.get_param("~min_confirmed_hits", 2))

        self.pick_line_y = float(rospy.get_param("~pick_line_y", 330.0))
        self.pick_tolerance_px = float(rospy.get_param("~pick_tolerance_px", 80.0))
        self.cooldown_sec = float(rospy.get_param("~cooldown_sec", 1.5))
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.25))

        # Dataset mode için crossing şartını kapalı tutuyoruz.
        self.require_crossing = bool(rospy.get_param("~require_crossing", False))

        # /waste/detections aktifse /waste/detection legacy input ignore edilir.
        self.array_active_timeout_sec = float(rospy.get_param("~array_active_timeout_sec", 1.0))
        self.last_array_msg_wall_sec = -9999.0

        self.tracks: Dict[int, Track] = {}
        self.next_track_id = 1

        self.emitted_track_ids: Set[int] = set()
        self.last_event_time_by_class: Dict[str, float] = {}

        # Dataset sample state
        self.current_sample_model_name: str = ""
        self.current_sample_class_name: str = ""
        self.current_sample_image_path: str = ""
        self.emitted_sample_model_names: Set[str] = set()

        self.pub_tracked = rospy.Publisher(self.tracked_topic, String, queue_size=20)
        self.pub_pick_event = rospy.Publisher(self.pick_event_topic, String, queue_size=20)

        self.sub_sample = rospy.Subscriber(
            self.sample_topic,
            String,
            self.sample_callback,
            queue_size=10,
        )

        self.sub_detections = rospy.Subscriber(
            self.detections_topic,
            Detection2DArray,
            self.detections_callback,
            queue_size=10,
        )

        self.sub_legacy = rospy.Subscriber(
            self.legacy_detection_topic,
            Detection2D,
            self.legacy_detection_callback,
            queue_size=10,
        )

        self.cleanup_timer = rospy.Timer(rospy.Duration(0.2), self.cleanup_timer_callback)
        self.debug_timer = rospy.Timer(rospy.Duration(1.0), self.debug_timer_callback)

        rospy.loginfo("=" * 80)
        rospy.loginfo("[TRACKER] Started")
        rospy.loginfo("[TRACKER] detections_topic: %s", self.detections_topic)
        rospy.loginfo("[TRACKER] legacy_detection_topic: %s", self.legacy_detection_topic)
        rospy.loginfo("[TRACKER] sample_topic: %s", self.sample_topic)
        rospy.loginfo("[TRACKER] tracked_topic: %s", self.tracked_topic)
        rospy.loginfo("[TRACKER] pick_event_topic: %s", self.pick_event_topic)
        rospy.loginfo("[TRACKER] max_match_distance_px: %.1f", self.max_match_distance_px)
        rospy.loginfo("[TRACKER] max_track_age_sec: %.2f", self.max_track_age_sec)
        rospy.loginfo("[TRACKER] min_confirmed_hits: %d", self.min_confirmed_hits)
        rospy.loginfo("[TRACKER] pick_line_y: %.1f", self.pick_line_y)
        rospy.loginfo("[TRACKER] pick_tolerance_px: %.1f", self.pick_tolerance_px)
        rospy.loginfo("[TRACKER] cooldown_sec: %.2f", self.cooldown_sec)
        rospy.loginfo("[TRACKER] min_confidence: %.2f", self.min_confidence)
        rospy.loginfo("=" * 80)

    def sample_callback(self, msg: String) -> None:
        with self.lock:
            try:
                payload = json.loads(msg.data)
            except Exception:
                return

            new_model_name = str(payload.get("model_name", "")).strip()
            new_class_name = str(payload.get("class_name", "")).strip().lower()
            new_image_path = str(payload.get("image_path", "")).strip()

            if not new_model_name or new_class_name not in VALID_CLASSES:
                return

            if new_model_name == self.current_sample_model_name:
                return

            self.current_sample_model_name = new_model_name
            self.current_sample_class_name = new_class_name
            self.current_sample_image_path = new_image_path

            # Yeni sample geldiğinde eski track'leri temizliyoruz.
            self.tracks.clear()

            rospy.loginfo(
                "[TRACKER] New dataset sample model=%s class=%s. Cleared old tracks.",
                self.current_sample_model_name,
                self.current_sample_class_name,
            )

    def detections_callback(self, msg: Detection2DArray) -> None:
        with self.lock:
            now_sec = self._stamp_to_sec(msg.header.stamp)
            if now_sec <= 0.0:
                now_sec = rospy.get_time()

            self.last_array_msg_wall_sec = rospy.get_time()

            detections = []

            for det_msg in msg.detections:
                det = self._convert_detection_msg(det_msg, now_sec)
                if det is not None:
                    detections.append(det)

            self.process_detections(detections, now_sec)

    def legacy_detection_callback(self, msg: Detection2D) -> None:
        with self.lock:
            wall_now = rospy.get_time()

            if wall_now - self.last_array_msg_wall_sec <= self.array_active_timeout_sec:
                return

            now_sec = self._stamp_to_sec(msg.header.stamp)
            if now_sec <= 0.0:
                now_sec = wall_now

            det = self._convert_detection_msg(msg, now_sec)
            if det is None:
                return

            self.process_detections([det], now_sec)

    def _convert_detection_msg(self, msg: Detection2D, stamp_sec: float) -> Optional[Detection]:
        class_name = str(msg.class_name).strip().lower()

        if class_name not in VALID_CLASSES:
            return None

        # Dataset sample belli ise yanlış sınıf detection'ı ignore ediyoruz.
        # Bu, aynı frame içinde yanlış class ghost track oluşmasını azaltır.
        if self.current_sample_class_name and class_name != self.current_sample_class_name:
            return None

        confidence = float(msg.confidence)
        if confidence < self.min_confidence:
            return None

        return Detection(
            class_name=class_name,
            confidence=confidence,
            x_min=int(msg.x_min),
            y_min=int(msg.y_min),
            x_max=int(msg.x_max),
            y_max=int(msg.y_max),
            center_x=int(msg.center_x),
            center_y=int(msg.center_y),
            stamp_sec=float(stamp_sec),
        )

    def process_detections(self, detections: List[Detection], now_sec: float) -> None:
        if self.current_sample_model_name in self.emitted_sample_model_names:
            self._publish_tracks(now_sec)
            return

        detections = self._deduplicate_detections(detections)

        matched_track_ids: Set[int] = set()
        matched_detection_indices: Set[int] = set()

        candidate_pairs: List[Tuple[float, int, int]] = []

        for det_idx, det in enumerate(detections):
            for track_id, track in self.tracks.items():
                if track.state == "EXPIRED":
                    continue
                if track.event_emitted or track.picked:
                    continue
                if track.class_name != det.class_name:
                    continue

                distance = self._distance(track.center_x, track.center_y, det.center_x, det.center_y)

                if distance <= self.max_match_distance_px:
                    candidate_pairs.append((distance, track_id, det_idx))

        candidate_pairs.sort(key=lambda item: item[0])

        for _, track_id, det_idx in candidate_pairs:
            if track_id in matched_track_ids:
                continue
            if det_idx in matched_detection_indices:
                continue
            if track_id not in self.tracks:
                continue

            self.tracks[track_id].update(detections[det_idx])
            matched_track_ids.add(track_id)
            matched_detection_indices.add(det_idx)

        for track_id, track in self.tracks.items():
            if track_id not in matched_track_ids:
                track.mark_missed()

        for det_idx, det in enumerate(detections):
            if det_idx in matched_detection_indices:
                continue

            if self._has_nearby_same_class_track(det):
                continue

            self._create_track(det)

        self._update_track_states(now_sec)
        self._publish_tracks(now_sec)
        self._maybe_emit_pick_events(now_sec)

    def _deduplicate_detections(self, detections: List[Detection]) -> List[Detection]:
        if not detections:
            return []

        detections_sorted = sorted(detections, key=lambda d: d.confidence, reverse=True)
        kept: List[Detection] = []

        dedup_distance_px = max(25.0, self.max_match_distance_px * 0.45)

        for det in detections_sorted:
            duplicate = False

            for kept_det in kept:
                if kept_det.class_name != det.class_name:
                    continue

                distance = self._distance(
                    kept_det.center_x,
                    kept_det.center_y,
                    det.center_x,
                    det.center_y,
                )

                if distance <= dedup_distance_px:
                    duplicate = True
                    break

            if not duplicate:
                kept.append(det)

        return kept

    def _has_nearby_same_class_track(self, det: Detection) -> bool:
        for track in self.tracks.values():
            if track.state == "EXPIRED":
                continue
            if track.event_emitted or track.picked:
                continue
            if track.class_name != det.class_name:
                continue

            distance = self._distance(track.center_x, track.center_y, det.center_x, det.center_y)

            if distance <= self.max_match_distance_px * 0.75:
                return True

        return False

    def _create_track(self, det: Detection) -> None:
        track = Track(
            track_id=self.next_track_id,
            class_name=det.class_name,
            center_x=float(det.center_x),
            center_y=float(det.center_y),
            confidence=float(det.confidence),
            created_sec=float(det.stamp_sec),
            last_seen_sec=float(det.stamp_sec),
            x_min=int(det.x_min),
            y_min=int(det.y_min),
            x_max=int(det.x_max),
            y_max=int(det.y_max),
            hits=1,
            misses=0,
            state="NEW",
            picked=False,
            event_emitted=False,
            confirm_logged=False,
            previous_center_x=None,
            previous_center_y=None,
            history=[(float(det.stamp_sec), float(det.center_x), float(det.center_y))],
        )

        self.tracks[track.track_id] = track

        rospy.loginfo(
            "[TRACKER] New track #%d class=%s center=(%.1f, %.1f) conf=%.2f sample=%s",
            track.track_id,
            track.class_name,
            track.center_x,
            track.center_y,
            track.confidence,
            self.current_sample_model_name,
        )

        self.next_track_id += 1

    def _update_track_states(self, now_sec: float) -> None:
        for track in self.tracks.values():
            if track.state == "EXPIRED":
                continue

            if track.time_since_seen_sec(now_sec) > self.max_track_age_sec:
                track.state = "EXPIRED"
                rospy.loginfo("[TRACKER] Expired track #%d class=%s", track.track_id, track.class_name)
                continue

            if track.event_emitted or track.picked:
                track.state = "PICKED"
                continue

            if track.hits >= self.min_confirmed_hits:
                if not track.confirm_logged:
                    rospy.loginfo(
                        "[TRACKER] Confirmed track #%d class=%s hits=%d sample=%s",
                        track.track_id,
                        track.class_name,
                        track.hits,
                        self.current_sample_model_name,
                    )
                    track.confirm_logged = True

                track.state = "CONFIRMED"

    def _maybe_emit_pick_events(self, now_sec: float) -> None:
        if self.current_sample_model_name in self.emitted_sample_model_names:
            return

        for track in list(self.tracks.values()):
            if track.track_id in self.emitted_track_ids:
                continue
            if track.event_emitted:
                continue
            if track.picked:
                continue
            if track.state != "CONFIRMED":
                continue
            if not self._track_is_pick_ready(track):
                continue

            last_class_event_sec = self.last_event_time_by_class.get(track.class_name, -9999.0)
            if now_sec - last_class_event_sec < self.cooldown_sec:
                continue

            self._emit_pick_event(track, now_sec)
            return

    def _track_is_pick_ready(self, track: Track) -> bool:
        current_y = float(track.center_y)
        inside_zone = abs(current_y - self.pick_line_y) <= self.pick_tolerance_px

        if not inside_zone:
            return False

        if not self.require_crossing:
            return True

        if track.previous_center_y is None:
            return False

        prev_y = float(track.previous_center_y)

        crossed_down = prev_y < self.pick_line_y <= current_y
        crossed_up = prev_y > self.pick_line_y >= current_y

        return crossed_down or crossed_up

    def _emit_pick_event(self, track: Track, now_sec: float) -> None:
        track.event_emitted = True
        track.picked = True
        track.state = "PICKED"

        self.emitted_track_ids.add(track.track_id)

        if self.current_sample_model_name:
            self.emitted_sample_model_names.add(self.current_sample_model_name)

        self.last_event_time_by_class[track.class_name] = now_sec

        event = {
            "event_type": "pick_event",
            "track_id": int(track.track_id),
            "class_name": str(track.class_name),
            "model_name": str(self.current_sample_model_name),
            "confidence": float(track.confidence),
            "center_x": int(round(track.center_x)),
            "center_y": int(round(track.center_y)),
            "pick_line_y": float(self.pick_line_y),
            "stamp": float(now_sec),
            "state": "CONFIRMED",
        }

        self.pub_pick_event.publish(String(data=json.dumps(event, sort_keys=True)))

        rospy.loginfo(
            "[TRACKER] PICK_EVENT track_id=%d class=%s model=%s center=(%.1f, %.1f)",
            track.track_id,
            track.class_name,
            self.current_sample_model_name,
            track.center_x,
            track.center_y,
        )

    def _publish_tracks(self, now_sec: float) -> None:
        active_tracks = []

        for track in self.tracks.values():
            if track.state == "EXPIRED":
                continue

            active_tracks.append(
                {
                    "track_id": int(track.track_id),
                    "class_name": str(track.class_name),
                    "model_name": str(self.current_sample_model_name),
                    "confidence": float(track.confidence),
                    "center_x": int(round(track.center_x)),
                    "center_y": int(round(track.center_y)),
                    "x_min": int(track.x_min),
                    "y_min": int(track.y_min),
                    "x_max": int(track.x_max),
                    "y_max": int(track.y_max),
                    "hits": int(track.hits),
                    "misses": int(track.misses),
                    "state": str(track.state),
                    "picked": bool(track.picked),
                    "event_emitted": bool(track.event_emitted),
                    "age_sec": float(track.age_sec(now_sec)),
                    "time_since_seen_sec": float(track.time_since_seen_sec(now_sec)),
                }
            )

        payload = {
            "stamp": float(now_sec),
            "sample_model_name": str(self.current_sample_model_name),
            "sample_class_name": str(self.current_sample_class_name),
            "active_track_count": len(active_tracks),
            "tracks": active_tracks,
        }

        self.pub_tracked.publish(String(data=json.dumps(payload, sort_keys=True)))

    def cleanup_timer_callback(self, _event) -> None:
        with self.lock:
            now_sec = rospy.get_time()
            delete_ids = []

            for track_id, track in self.tracks.items():
                if track.state == "EXPIRED":
                    delete_ids.append(track_id)
                    continue

                if track.time_since_seen_sec(now_sec) > self.max_track_age_sec * 2.0:
                    delete_ids.append(track_id)

            for track_id in delete_ids:
                self.tracks.pop(track_id, None)

    def debug_timer_callback(self, _event) -> None:
        with self.lock:
            active = [t for t in self.tracks.values() if t.state != "EXPIRED"]

            if not active:
                rospy.loginfo_throttle(5.0, "[TRACKER] No active tracks")
                return

            summary = []
            for track in active:
                summary.append(
                    "#{}:{}:{}:hits{}:({:.0f},{:.0f})".format(
                        track.track_id,
                        track.class_name,
                        track.state,
                        track.hits,
                        track.center_x,
                        track.center_y,
                    )
                )

            rospy.loginfo("[TRACKER] Active tracks: %s", " | ".join(summary))

    @staticmethod
    def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.sqrt((float(x1) - float(x2)) ** 2 + (float(y1) - float(y2)) ** 2)

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        try:
            return float(stamp.to_sec())
        except Exception:
            return 0.0


def main() -> None:
    try:
        WasteTrackerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logfatal("[TRACKER] Fatal error: %s", str(exc))
        rospy.logfatal(traceback.format_exc())
        sys.exit(1)


if __name__ == "__main__":
    main()
