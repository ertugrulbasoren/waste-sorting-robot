#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Prepare downloaded realistic Objaverse assets for Gazebo Classic.

Input:
  src/waste_sorting_gazebo/raw_assets/<class>/<asset_folder>/source.glb

Output:
  src/waste_sorting_gazebo/models/realistic_assets/<class>/<asset_folder>/
    ├── model.config
    ├── model.sdf
    ├── meshes/model.obj
    ├── metadata.json
    └── bbox.json

This version exports OBJ instead of DAE.

Why OBJ:
  Some DAE exports are accepted by Gazebo but do not render visually.
  OBJ is simpler and more robust for Gazebo Classic visualization.

Important fix:
  The Blender Python code is generated with token replacement, not .format(),
  so JSON/dict braces inside Blender code cannot break the script.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from typing import Dict, List, Optional, Tuple


VALID_CLASSES = ["glass", "metal", "paper", "plastic"]

CLASS_TARGET_MAX_DIM = {
    "glass": 0.30,
    "metal": 0.18,
    "paper": 0.26,
    "plastic": 0.30,
}

CLASS_MASS = {
    "glass": 0.12,
    "metal": 0.08,
    "paper": 0.03,
    "plastic": 0.04,
}

CLASS_COLOR = {
    "glass": (0.55, 0.90, 1.00, 0.85),
    "metal": (0.70, 0.70, 0.70, 1.00),
    "paper": (0.86, 0.72, 0.48, 1.00),
    "plastic": (0.25, 0.45, 1.00, 1.00),
}


def package_root_from_script() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def clean(value) -> str:
    return str(value or "").replace("\n", " ").replace("\r", " ").strip()


def blender_exists() -> bool:
    try:
        subprocess.check_call(
            ["blender", "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return True
    except Exception:
        return False


def find_source_file(asset_dir: str) -> Optional[str]:
    preferred = ["source.glb", "source.gltf", "source.obj", "source.dae", "source.stl"]

    for name in preferred:
        path = os.path.join(asset_dir, name)
        if os.path.exists(path):
            return path

    for name in os.listdir(asset_dir):
        lower = name.lower()
        if lower.endswith((".glb", ".gltf", ".obj", ".dae", ".stl")):
            return os.path.join(asset_dir, name)

    return None


def load_metadata(asset_dir: str) -> Dict:
    path = os.path.join(asset_dir, "metadata.json")

    if not os.path.exists(path):
        return {}

    try:
        with open(path, "r") as f:
            return json.load(f)
    except Exception:
        return {}


def run_blender_convert_to_obj(
    source_path: str,
    obj_path: str,
    bbox_json_path: str,
    target_max_dim: float,
) -> None:
    ensure_dir(os.path.dirname(obj_path))

    blender_code = r'''
import bpy
import json
import os

source_path = "__SOURCE_PATH__"
obj_path = "__OBJ_PATH__"
bbox_json_path = "__BBOX_JSON_PATH__"
target_max_dim = float("__TARGET_MAX_DIM__")

def fail(msg):
    raise RuntimeError(msg)

bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

ext = os.path.splitext(source_path)[1].lower()

if ext in [".glb", ".gltf"]:
    bpy.ops.import_scene.gltf(filepath=source_path)
elif ext == ".obj":
    bpy.ops.import_scene.obj(filepath=source_path)
elif ext == ".dae":
    bpy.ops.wm.collada_import(filepath=source_path)
elif ext == ".stl":
    bpy.ops.import_mesh.stl(filepath=source_path)
else:
    fail("Unsupported extension: " + ext)

# Delete non-mesh objects.
for obj in list(bpy.context.scene.objects):
    if obj.type != "MESH":
        bpy.data.objects.remove(obj, do_unlink=True)

mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]

if not mesh_objects:
    fail("No mesh object found")

# Apply transforms to all mesh objects.
for obj in mesh_objects:
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

# Join all meshes into one mesh.
bpy.ops.object.select_all(action='DESELECT')
mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]

for obj in mesh_objects:
    obj.select_set(True)

bpy.context.view_layer.objects.active = mesh_objects[0]

if len(mesh_objects) > 1:
    bpy.ops.object.join()

obj = bpy.context.view_layer.objects.active

if obj is None or obj.type != "MESH":
    fail("Joined object is invalid")

mesh = obj.data

if len(mesh.vertices) == 0:
    fail("Mesh has zero vertices")

def compute_bounds():
    xs = [v.co.x for v in mesh.vertices]
    ys = [v.co.y for v in mesh.vertices]
    zs = [v.co.z for v in mesh.vertices]

    return {
        "min_x": min(xs),
        "max_x": max(xs),
        "min_y": min(ys),
        "max_y": max(ys),
        "min_z": min(zs),
        "max_z": max(zs),
    }

bounds = compute_bounds()

size_x = bounds["max_x"] - bounds["min_x"]
size_y = bounds["max_y"] - bounds["min_y"]
size_z = bounds["max_z"] - bounds["min_z"]
max_dim = max(size_x, size_y, size_z)

if max_dim <= 0:
    fail("Invalid max dimension")

scale_factor = target_max_dim / max_dim

for v in mesh.vertices:
    v.co.x *= scale_factor
    v.co.y *= scale_factor
    v.co.z *= scale_factor

bounds = compute_bounds()

center_x = (bounds["min_x"] + bounds["max_x"]) / 2.0
center_y = (bounds["min_y"] + bounds["max_y"]) / 2.0
bottom_z = bounds["min_z"]

# Normalize geometry:
#   X/Y centered around zero
#   bottom at Z=0
for v in mesh.vertices:
    v.co.x -= center_x
    v.co.y -= center_y
    v.co.z -= bottom_z

obj.location = (0.0, 0.0, 0.0)
obj.rotation_euler = (0.0, 0.0, 0.0)
obj.scale = (1.0, 1.0, 1.0)

bpy.ops.object.select_all(action='DESELECT')
obj.select_set(True)
bpy.context.view_layer.objects.active = obj
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

bounds = compute_bounds()

size_x = bounds["max_x"] - bounds["min_x"]
size_y = bounds["max_y"] - bounds["min_y"]
size_z = bounds["max_z"] - bounds["min_z"]

bbox = {
    "min_x": float(bounds["min_x"]),
    "max_x": float(bounds["max_x"]),
    "min_y": float(bounds["min_y"]),
    "max_y": float(bounds["max_y"]),
    "min_z": float(bounds["min_z"]),
    "max_z": float(bounds["max_z"]),
    "size_x": float(size_x),
    "size_y": float(size_y),
    "size_z": float(size_z),
    "scale_factor": float(scale_factor),
    "target_max_dim": float(target_max_dim),
}

with open(bbox_json_path, "w") as f:
    json.dump(bbox, f, indent=2)

bpy.ops.object.select_all(action='DESELECT')
obj.select_set(True)
bpy.context.view_layer.objects.active = obj

# Export OBJ.
# use_materials=False avoids missing MTL/texture rendering issues.
bpy.ops.export_scene.obj(
    filepath=obj_path,
    use_selection=True,
    use_materials=False,
    axis_forward='-Z',
    axis_up='Y',
)

print("[BLENDER OK] exported OBJ:", obj_path)
print("[BLENDER OK] bbox:", bbox)
'''

    blender_code = blender_code.replace("__SOURCE_PATH__", source_path.replace("\\", "\\\\"))
    blender_code = blender_code.replace("__OBJ_PATH__", obj_path.replace("\\", "\\\\"))
    blender_code = blender_code.replace("__BBOX_JSON_PATH__", bbox_json_path.replace("\\", "\\\\"))
    blender_code = blender_code.replace("__TARGET_MAX_DIM__", str(target_max_dim))

    with tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False) as tmp:
        tmp.write(blender_code)
        tmp_path = tmp.name

    try:
        subprocess.check_call(["blender", "--background", "--python", tmp_path])
    finally:
        try:
            os.remove(tmp_path)
        except Exception:
            pass


def read_bbox(path: str) -> Dict[str, float]:
    with open(path, "r") as f:
        return json.load(f)


def safe_collision_dims(class_name: str, bbox: Dict[str, float]) -> Tuple[float, float, float]:
    sx = float(bbox.get("size_x", 0.16))
    sy = float(bbox.get("size_y", 0.16))
    sz = float(bbox.get("size_z", 0.16))

    sx = max(0.04, min(sx * 0.95, 0.45))
    sy = max(0.04, min(sy * 0.95, 0.45))
    sz = max(0.025, min(sz * 0.95, 0.45))

    if class_name == "paper":
        sz = max(sz, 0.035)

    return sx, sy, sz


def write_model_config(model_dir: str, model_name: str, asset_name: str) -> None:
    path = os.path.join(model_dir, "model.config")

    content = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Waste Sorting Robot Project</name>
    <email>none</email>
  </author>
  <description>Realistic waste asset imported from Objaverse: {asset_name}</description>
</model>
"""

    with open(path, "w") as f:
        f.write(content)


def write_model_sdf(
    model_dir: str,
    model_name: str,
    class_name: str,
    mesh_filename: str,
    bbox: Dict[str, float],
) -> None:
    path = os.path.join(model_dir, "model.sdf")

    sx, sy, sz = safe_collision_dims(class_name, bbox)
    mass = CLASS_MASS[class_name]
    r, g, b, a = CLASS_COLOR[class_name]

    mesh_path = os.path.abspath(os.path.join(model_dir, "meshes", mesh_filename))
    mesh_uri = "file://" + mesh_path

    content = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>false</static>
    <allow_auto_disable>false</allow_auto_disable>

    <link name="link">
      <pose>0 0 0 0 0 0</pose>

      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>

      <collision name="collision">
        <pose>0 0 {sz / 2.0} 0 0 0</pose>
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

      <visual name="visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>{mesh_uri}</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

    with open(path, "w") as f:
        f.write(content)


def copy_metadata(src_asset_dir: str, dst_model_dir: str, extra: Dict) -> None:
    metadata = load_metadata(src_asset_dir)
    metadata.update(extra)

    with open(os.path.join(dst_model_dir, "metadata.json"), "w") as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)


def iter_raw_assets(raw_root: str, class_filter: str) -> List[Tuple[str, str, str]]:
    result = []

    classes = VALID_CLASSES if class_filter == "all" else [class_filter]

    for class_name in classes:
        class_dir = os.path.join(raw_root, class_name)

        if not os.path.isdir(class_dir):
            continue

        for folder in sorted(os.listdir(class_dir)):
            if folder.startswith("."):
                continue

            asset_dir = os.path.join(class_dir, folder)

            if not os.path.isdir(asset_dir):
                continue

            source_file = find_source_file(asset_dir)

            if source_file:
                result.append((class_name, folder, source_file))

    return result


def prepare_asset(
    class_name: str,
    asset_folder: str,
    source_file: str,
    output_root: str,
    force: bool,
) -> bool:
    src_asset_dir = os.path.dirname(source_file)

    model_name = asset_folder
    model_dir = os.path.join(output_root, class_name, model_name)
    meshes_dir = os.path.join(model_dir, "meshes")

    if os.path.exists(model_dir) and not force:
        print("[SKIP] already prepared:", model_dir)
        return True

    if os.path.exists(model_dir) and force:
        shutil.rmtree(model_dir)

    ensure_dir(meshes_dir)

    metadata = load_metadata(src_asset_dir)
    asset_name = clean(metadata.get("asset_name", model_name))

    mesh_filename = "model.obj"
    obj_path = os.path.join(meshes_dir, mesh_filename)
    bbox_json_path = os.path.join(model_dir, "bbox.json")

    try:
        run_blender_convert_to_obj(
            source_path=source_file,
            obj_path=obj_path,
            bbox_json_path=bbox_json_path,
            target_max_dim=CLASS_TARGET_MAX_DIM[class_name],
        )

        bbox = read_bbox(bbox_json_path)

        write_model_config(model_dir, model_name, asset_name)
        write_model_sdf(model_dir, model_name, class_name, mesh_filename, bbox)

        copy_metadata(
            src_asset_dir=src_asset_dir,
            dst_model_dir=model_dir,
            extra={
                "prepared_model_name": model_name,
                "prepared_class_name": class_name,
                "prepared_mesh_filename": mesh_filename,
                "source_file": source_file,
                "bbox": bbox,
                "mesh_format": "obj",
            },
        )

        print("[OK] prepared:", class_name, model_name, "bbox=", bbox)
        return True

    except Exception as exc:
        print("[ERROR] failed:", class_name, model_name, "error:", str(exc))
        return False


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--class-name", default="all", choices=["all"] + VALID_CLASSES)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--force", action="store_true")

    args = parser.parse_args()

    package_root = package_root_from_script()

    raw_root = os.path.join(package_root, "raw_assets")
    output_root = os.path.join(package_root, "models", "realistic_assets")

    print("=" * 100)
    print("PREPARE REALISTIC ASSETS FOR GAZEBO - OBJ EXPORT")
    print("=" * 100)
    print("[INFO] raw_root:", raw_root)
    print("[INFO] output_root:", output_root)
    print("[INFO] class_name:", args.class_name)
    print("[INFO] limit:", args.limit)
    print("=" * 100)

    if not blender_exists():
        print("[ERROR] Blender not found.")
        print("Install with:")
        print("  sudo apt update && sudo apt install -y blender")
        sys.exit(1)

    assets = iter_raw_assets(raw_root, args.class_name)

    if args.limit > 0:
        assets = assets[: args.limit]

    if not assets:
        print("[ERROR] no raw assets found.")
        sys.exit(1)

    ok_count = 0
    fail_count = 0

    for class_name, folder, source_file in assets:
        ok = prepare_asset(
            class_name=class_name,
            asset_folder=folder,
            source_file=source_file,
            output_root=output_root,
            force=args.force,
        )

        if ok:
            ok_count += 1
        else:
            fail_count += 1

    print("=" * 100)
    print("[OK] prepare finished.")
    print("[OK] success:", ok_count)
    print("[OK] failed:", fail_count)
    print("=" * 100)


if __name__ == "__main__":
    main()
