#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Dataset camera feeder for the waste sorting robot.

This node:
  1. Reads YOLO-format dataset samples from datasets/waste_4class/waste_dataset.
  2. Publishes moving synthetic camera frames to /dataset_camera/image_raw.
  3. Publishes current sample metadata to /dataset_camera/current_sample.
  4. Spawns a synchronized Gazebo proxy object named:
       waste_glass_001
       waste_metal_001
       waste_paper_001
       waste_plastic_001
  5. Applies dataset crop as an OGRE/Gazebo Classic material texture.

Important production behavior:
  - Class-balanced round-robin sampling:
      glass -> metal -> paper -> plastic -> ...
  - Prevents class starvation in KPI tests.
  - Generates textured Gazebo proxy models.
  - Deletes previous waste models before spawning a new synchronized model.
  - Keeps YOLO image stream and Gazebo model name synchronized.
"""

import glob
import json
import os
import random
import shutil
import sys
import traceback
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import String


CLASS_ID_TO_NAME = {
    0: "glass",
    1: "metal",
    2: "paper",
    3: "plastic",
}

VALID_CLASSES = {"glass", "metal", "paper", "plastic"}

CLASS_CYCLE = ["glass", "metal", "paper", "plastic"]


class DatasetSample:
    def __init__(
        self,
        image_path: str,
        label_path: str,
        class_id: int,
        class_name: str,
        bbox: Tuple[float, float, float, float],
    ) -> None:
        self.image_path = image_path
        self.label_path = label_path
        self.class_id = class_id
        self.class_name = class_name
        self.bbox = bbox


class DatasetCameraFeeder:
    def __init__(self) -> None:
        rospy.init_node("dataset_camera_feeder", anonymous=False)

        package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

        default_dataset_root = os.path.abspath(
            os.path.join(
                package_root,
                "..",
                "..",
                "datasets",
                "waste_4class",
                "waste_dataset",
            )
        )

        default_generated_root = os.path.abspath(
            os.path.join(
                package_root,
                "generated_dataset_models",
            )
        )

        self.dataset_root = rospy.get_param("~dataset_root", default_dataset_root)
        self.split = rospy.get_param("~split", "valid")

        self.image_topic = rospy.get_param("~image_topic", "/dataset_camera/image_raw")
        self.sample_topic = rospy.get_param("~sample_topic", "/dataset_camera/current_sample")

        self.width = int(rospy.get_param("~width", 640))
        self.height = int(rospy.get_param("~height", 480))
        self.fps = float(rospy.get_param("~fps", 10.0))

        self.frames_per_object = int(rospy.get_param("~frames_per_object", 45))
        self.object_width_px = int(rospy.get_param("~object_width_px", 260))
        self.start_y = int(rospy.get_param("~start_y", 100))
        self.end_y = int(rospy.get_param("~end_y", 355))
        self.object_center_x = int(rospy.get_param("~object_center_x", 320))

        self.spawn_gazebo_model = bool(rospy.get_param("~spawn_gazebo_model", True))
        self.delete_existing = bool(rospy.get_param("~delete_existing", True))

        self.generated_root = rospy.get_param("~texture_root", default_generated_root)

        self.gazebo_spawn_x = float(rospy.get_param("~gazebo_spawn_x", -0.45))
        self.gazebo_spawn_y = float(rospy.get_param("~gazebo_spawn_y", 0.0))
        self.gazebo_spawn_z = float(rospy.get_param("~gazebo_spawn_z", 0.18))

        self.proxy_size_x = float(rospy.get_param("~proxy_size_x", 0.55))
        self.proxy_size_y = float(rospy.get_param("~proxy_size_y", 0.38))
        self.proxy_size_z = float(rospy.get_param("~proxy_size_z", 0.018))

        self.bridge = CvBridge()

        self.pub_image = rospy.Publisher(self.image_topic, Image, queue_size=2)
        self.pub_sample = rospy.Publisher(self.sample_topic, String, queue_size=10)

        self.samples: List[DatasetSample] = []
        self.samples_by_class: Dict[str, List[DatasetSample]] = {
            "glass": [],
            "metal": [],
            "paper": [],
            "plastic": [],
        }

        self.class_cycle: List[str] = list(CLASS_CYCLE)
        self.class_cycle_index: int = 0

        self.current_sample: Optional[DatasetSample] = None
        self.current_crop: Optional[np.ndarray] = None
        self.current_model_name: str = ""

        self.frame_index: int = 0
        self.model_counter: int = 1

        self.spawn_srv = None
        self.delete_srv = None
        self.world_props_srv = None

        self.prepare_generated_root()
        self.load_dataset()

        if not self.samples:
            rospy.logfatal("[DATASET CAMERA] No valid samples found.")
            rospy.logfatal("[DATASET CAMERA] dataset_root=%s", self.dataset_root)
            sys.exit(1)

        self.validate_class_availability()

        if self.spawn_gazebo_model:
            self.wait_for_gazebo_services()

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.fps), self.timer_callback)

        rospy.loginfo("=" * 80)
        rospy.loginfo("[DATASET CAMERA] Started")
        rospy.loginfo("[DATASET CAMERA] dataset_root: %s", self.dataset_root)
        rospy.loginfo("[DATASET CAMERA] split: %s", self.split)
        rospy.loginfo("[DATASET CAMERA] loaded total samples: %d", len(self.samples))
        rospy.loginfo("[DATASET CAMERA] image_topic: %s", self.image_topic)
        rospy.loginfo("[DATASET CAMERA] sample_topic: %s", self.sample_topic)
        rospy.loginfo("[DATASET CAMERA] generated_root: %s", self.generated_root)
        rospy.loginfo("[DATASET CAMERA] frame size: %dx%d", self.width, self.height)
        rospy.loginfo("[DATASET CAMERA] fps: %.1f", self.fps)
        rospy.loginfo("[DATASET CAMERA] sampling mode: class-balanced round-robin")
        rospy.loginfo("=" * 80)

    def prepare_generated_root(self) -> None:
        os.makedirs(self.generated_root, exist_ok=True)

        for path in glob.glob(os.path.join(self.generated_root, "waste_*")):
            if os.path.isdir(path):
                try:
                    shutil.rmtree(path)
                except Exception:
                    pass

    def wait_for_gazebo_services(self) -> None:
        rospy.loginfo("[DATASET CAMERA] Waiting for Gazebo services...")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.wait_for_service("/gazebo/get_world_properties")

        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.world_props_srv = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)

    def load_dataset(self) -> None:
        images_dir = os.path.join(self.dataset_root, self.split, "images")
        labels_dir = os.path.join(self.dataset_root, self.split, "labels")

        if not os.path.isdir(images_dir):
            rospy.logfatal("[DATASET CAMERA] images_dir not found: %s", images_dir)
            return

        if not os.path.isdir(labels_dir):
            rospy.logfatal("[DATASET CAMERA] labels_dir not found: %s", labels_dir)
            return

        image_paths: List[str] = []

        for ext in ("*.jpg", "*.jpeg", "*.png", "*.JPG", "*.JPEG", "*.PNG"):
            image_paths.extend(glob.glob(os.path.join(images_dir, ext)))

        for image_path in image_paths:
            base = os.path.splitext(os.path.basename(image_path))[0]
            label_path = os.path.join(labels_dir, base + ".txt")

            if not os.path.exists(label_path):
                continue

            labels = self.read_yolo_label_file(label_path)

            for class_id, x_center, y_center, box_w, box_h in labels:
                class_name = CLASS_ID_TO_NAME.get(class_id)

                if class_name not in VALID_CLASSES:
                    continue

                sample = DatasetSample(
                    image_path=image_path,
                    label_path=label_path,
                    class_id=class_id,
                    class_name=class_name,
                    bbox=(x_center, y_center, box_w, box_h),
                )

                self.samples.append(sample)
                self.samples_by_class[class_name].append(sample)

        random.shuffle(self.samples)

        for class_name in self.samples_by_class:
            random.shuffle(self.samples_by_class[class_name])

        for class_name in self.class_cycle:
            rospy.loginfo(
                "[DATASET CAMERA] Loaded %d samples for class=%s",
                len(self.samples_by_class[class_name]),
                class_name,
            )

    def validate_class_availability(self) -> None:
        missing_classes = []

        for class_name in self.class_cycle:
            if len(self.samples_by_class.get(class_name, [])) == 0:
                missing_classes.append(class_name)

        if missing_classes:
            rospy.logwarn(
                "[DATASET CAMERA] Some classes have zero samples and will be skipped: %s",
                missing_classes,
            )

    @staticmethod
    def read_yolo_label_file(label_path: str) -> List[Tuple[int, float, float, float, float]]:
        labels: List[Tuple[int, float, float, float, float]] = []

        try:
            with open(label_path, "r") as f:
                lines = f.readlines()
        except Exception:
            return labels

        for line in lines:
            parts = line.strip().split()

            if len(parts) < 5:
                continue

            try:
                class_id = int(float(parts[0]))
                x_center = float(parts[1])
                y_center = float(parts[2])
                box_w = float(parts[3])
                box_h = float(parts[4])
            except Exception:
                continue

            labels.append((class_id, x_center, y_center, box_w, box_h))

        return labels

    def select_balanced_sample(self) -> Optional[DatasetSample]:
        """
        Class-balanced round-robin sample selector.

        Selection order:
          glass -> metal -> paper -> plastic -> ...

        If one class has no samples, it is skipped safely.
        """

        for _ in range(len(self.class_cycle)):
            class_name = self.class_cycle[self.class_cycle_index]
            self.class_cycle_index = (self.class_cycle_index + 1) % len(self.class_cycle)

            class_samples = self.samples_by_class.get(class_name, [])

            if not class_samples:
                rospy.logwarn_throttle(
                    5.0,
                    "[DATASET CAMERA] No samples available for class=%s. Skipping.",
                    class_name,
                )
                continue

            return random.choice(class_samples)

        return None

    def select_new_sample(self) -> None:
        sample = self.select_balanced_sample()

        if sample is None:
            rospy.logwarn("[DATASET CAMERA] Balanced sampling failed. Falling back to random sample.")
            sample = random.choice(self.samples)

        image = cv2.imread(sample.image_path, cv2.IMREAD_COLOR)

        if image is None:
            rospy.logwarn("[DATASET CAMERA] Could not read image: %s", sample.image_path)
            return

        crop = self.crop_object(image, sample.bbox)

        if crop is None or crop.size == 0:
            rospy.logwarn("[DATASET CAMERA] Invalid crop: %s", sample.image_path)
            return

        crop = self.resize_crop(crop, self.object_width_px)

        self.current_sample = sample
        self.current_crop = crop
        self.frame_index = 0

        self.current_model_name = "waste_{}_{:03d}".format(
            sample.class_name,
            self.model_counter,
        )
        self.model_counter += 1

        model_dir = self.create_textured_model_files(
            model_name=self.current_model_name,
            class_name=sample.class_name,
            crop=crop,
        )

        if self.spawn_gazebo_model:
            self.spawn_textured_gazebo_model(
                class_name=sample.class_name,
                model_name=self.current_model_name,
                model_dir=model_dir,
            )

        payload = {
            "class_name": sample.class_name,
            "model_name": self.current_model_name,
            "image_path": sample.image_path,
            "label_path": sample.label_path,
            "model_dir": model_dir,
            "sampling_mode": "class_balanced_round_robin",
        }

        self.pub_sample.publish(String(data=json.dumps(payload, sort_keys=True)))

        rospy.loginfo(
            "[DATASET CAMERA] New sample class=%s model=%s image=%s",
            sample.class_name,
            self.current_model_name,
            os.path.basename(sample.image_path),
        )

    def crop_object(
        self,
        image: np.ndarray,
        bbox: Tuple[float, float, float, float],
    ) -> Optional[np.ndarray]:
        img_h, img_w = image.shape[:2]
        x_center, y_center, box_w, box_h = bbox

        x1 = int((x_center - box_w / 2.0) * img_w)
        y1 = int((y_center - box_h / 2.0) * img_h)
        x2 = int((x_center + box_w / 2.0) * img_w)
        y2 = int((y_center + box_h / 2.0) * img_h)

        pad_x = int((x2 - x1) * 0.30)
        pad_y = int((y2 - y1) * 0.30)

        x1 = max(0, x1 - pad_x)
        y1 = max(0, y1 - pad_y)
        x2 = min(img_w, x2 + pad_x)
        y2 = min(img_h, y2 + pad_y)

        if x2 <= x1 or y2 <= y1:
            return None

        return image[y1:y2, x1:x2].copy()

    @staticmethod
    def resize_crop(crop: np.ndarray, target_width: int) -> np.ndarray:
        h, w = crop.shape[:2]

        if w <= 0 or h <= 0:
            return crop

        scale = float(target_width) / float(w)
        target_height = max(24, int(h * scale))

        if target_height > 260:
            scale = 260.0 / float(h)
            target_width = max(24, int(w * scale))
            target_height = 260

        return cv2.resize(crop, (target_width, target_height), interpolation=cv2.INTER_AREA)

    def create_textured_model_files(
        self,
        model_name: str,
        class_name: str,
        crop: np.ndarray,
    ) -> str:
        model_dir = os.path.join(self.generated_root, model_name)
        textures_dir = os.path.join(model_dir, "materials", "textures")
        scripts_dir = os.path.join(model_dir, "materials", "scripts")

        os.makedirs(textures_dir, exist_ok=True)
        os.makedirs(scripts_dir, exist_ok=True)

        texture_file = "{}.png".format(model_name)
        texture_path = os.path.join(textures_dir, texture_file)

        texture_img = self.make_square_texture(crop)
        cv2.imwrite(texture_path, texture_img)

        material_name = "WasteGenerated/{}".format(model_name)
        material_path = os.path.join(scripts_dir, "{}.material".format(model_name))

        material_text = """material {material_name}
{{
  technique
  {{
    pass
    {{
      lighting off
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0.1 0.1 0.1 1

      texture_unit
      {{
        texture {texture_file}
        filtering anisotropic
      }}
    }}
  }}
}}
""".format(
            material_name=material_name,
            texture_file=texture_file,
        )

        with open(material_path, "w") as f:
            f.write(material_text)

        rospy.loginfo("[DATASET CAMERA] Generated material: %s", material_path)
        rospy.loginfo("[DATASET CAMERA] Generated texture: %s", texture_path)

        return model_dir

    @staticmethod
    def make_square_texture(crop: np.ndarray) -> np.ndarray:
        canvas_size = 512
        canvas = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)
        canvas[:, :] = (35, 35, 35)

        h, w = crop.shape[:2]

        if h <= 0 or w <= 0:
            return canvas

        max_w = 450
        max_h = 450

        scale = min(float(max_w) / float(w), float(max_h) / float(h), 1.0)

        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale))

        resized = cv2.resize(crop, (new_w, new_h), interpolation=cv2.INTER_AREA)

        x1 = int((canvas_size - new_w) / 2)
        y1 = int((canvas_size - new_h) / 2)

        canvas[y1:y1 + new_h, x1:x1 + new_w] = resized

        return canvas

    def make_background(self) -> np.ndarray:
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        frame[:, :] = (35, 35, 35)

        belt_x1 = 110
        belt_x2 = self.width - 110
        belt_y1 = 40
        belt_y2 = self.height - 40

        cv2.rectangle(frame, (belt_x1, belt_y1), (belt_x2, belt_y2), (70, 70, 70), -1)
        cv2.line(frame, (belt_x1, belt_y1), (belt_x1, belt_y2), (120, 120, 120), 3)
        cv2.line(frame, (belt_x2, belt_y1), (belt_x2, belt_y2), (120, 120, 120), 3)

        # Visual pick line.
        cv2.line(frame, (belt_x1, self.end_y), (belt_x2, self.end_y), (0, 180, 255), 2)

        for y in range(60, self.height - 40, 45):
            cv2.line(frame, (belt_x1 + 10, y), (belt_x2 - 10, y), (85, 85, 85), 1)

        return frame

    def compose_frame(self) -> Optional[np.ndarray]:
        if self.current_crop is None or self.current_sample is None:
            return None

        frame = self.make_background()
        crop = self.current_crop.copy()

        progress = float(self.frame_index) / float(max(1, self.frames_per_object - 1))
        center_y = int(self.start_y + progress * (self.end_y - self.start_y))
        center_x = self.object_center_x

        ch, cw = crop.shape[:2]

        x1 = int(center_x - cw / 2)
        y1 = int(center_y - ch / 2)
        x2 = x1 + cw
        y2 = y1 + ch

        if x1 < 0:
            crop = crop[:, -x1:]
            x1 = 0
        if y1 < 0:
            crop = crop[-y1:, :]
            y1 = 0
        if x2 > self.width:
            crop = crop[:, : self.width - x1]
            x2 = self.width
        if y2 > self.height:
            crop = crop[: self.height - y1, :]
            y2 = self.height

        if crop.size != 0:
            frame[y1:y2, x1:x2] = crop

        text = "{} | {} | balanced".format(
            self.current_sample.class_name,
            self.current_model_name,
        )

        cv2.putText(
            frame,
            text,
            (18, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        return frame

    def spawn_textured_gazebo_model(
        self,
        class_name: str,
        model_name: str,
        model_dir: str,
    ) -> None:
        if self.spawn_srv is None:
            return

        if self.delete_existing:
            self.delete_existing_waste_models()

        sdf_xml = self.make_textured_sdf(
            model_name=model_name,
            model_dir=model_dir,
        )

        pose = Pose()
        pose.position = Point(self.gazebo_spawn_x, self.gazebo_spawn_y, self.gazebo_spawn_z)
        pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        try:
            response = self.spawn_srv(model_name, sdf_xml, "", pose, "world")
        except Exception as exc:
            rospy.logwarn("[DATASET CAMERA] Gazebo spawn service failed: %s", str(exc))
            return

        if not response.success:
            rospy.logwarn("[DATASET CAMERA] Gazebo rejected spawn: %s", response.status_message)
            return

        rospy.loginfo("[DATASET CAMERA] Spawned textured Gazebo model: %s", model_name)

    def delete_existing_waste_models(self) -> None:
        if self.world_props_srv is None or self.delete_srv is None:
            return

        try:
            response = self.world_props_srv()
            model_names = list(response.model_names)
        except Exception:
            return

        for model_name in model_names:
            if not model_name.startswith("waste_"):
                continue

            try:
                self.delete_srv(model_name)
                rospy.loginfo("[DATASET CAMERA] Deleted old model: %s", model_name)
            except Exception:
                pass

    def make_textured_sdf(
        self,
        model_name: str,
        model_dir: str,
    ) -> str:
        scripts_uri = "file://{}".format(os.path.join(model_dir, "materials", "scripts"))
        textures_uri = "file://{}".format(os.path.join(model_dir, "materials", "textures"))
        material_name = "WasteGenerated/{}".format(model_name)

        return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>false</static>
    <allow_auto_disable>false</allow_auto_disable>

    <link name="link">
      <pose>0 0 0 0 0 0</pose>

      <inertial>
        <mass>0.06</mass>
        <inertia>
          <ixx>0.0006</ixx>
          <iyy>0.0006</iyy>
          <izz>0.0006</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>

      <collision name="collision">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>{sx} {sy} {sz}</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>8.0</mu>
              <mu2>8.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>

      <visual name="top_textured_plane">
        <pose>0 0 0.012 0 0 0</pose>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{sx} {sy}</size>
          </plane>
        </geometry>
        <material>
          <script>
            <uri>{scripts_uri}</uri>
            <uri>{textures_uri}</uri>
            <name>{material_name}</name>
          </script>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
        </material>
      </visual>

      <visual name="thin_body">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>{sx} {sy} {sz}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.15 0.15 0.15 1</ambient>
          <diffuse>0.15 0.15 0.15 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""".format(
            model_name=model_name,
            sx=self.proxy_size_x,
            sy=self.proxy_size_y,
            sz=self.proxy_size_z,
            scripts_uri=scripts_uri,
            textures_uri=textures_uri,
            material_name=material_name,
        )

    def timer_callback(self, _event) -> None:
        if self.current_sample is None or self.current_crop is None:
            self.select_new_sample()

        if self.current_sample is None or self.current_crop is None:
            return

        frame = self.compose_frame()

        if frame is None:
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "dataset_camera_link"

        self.pub_image.publish(msg)

        self.frame_index += 1

        if self.frame_index >= self.frames_per_object:
            self.select_new_sample()


def main() -> None:
    try:
        DatasetCameraFeeder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logfatal("[DATASET CAMERA] Fatal error: %s", str(exc))
        rospy.logfatal(traceback.format_exc())
        sys.exit(1)


if __name__ == "__main__":
    main()
