#!/usr/bin/env python3

import os
from math import pi  # if you want to use it somewhere
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def LoadMarkersFromYaml():
    packageShare = get_package_share_directory("amr_lv4")
    yamlPath = os.path.join(packageShare, "config", "aruco_markers.yaml")

    if not os.path.exists(yamlPath):
        raise FileNotFoundError(f"Marker config YAML not found: {yamlPath}")

    with open(yamlPath, "r") as f:
        data = yaml.safe_load(f) or {}

    markers = data.get("markers", [])
    if not markers:
        print(f"[spawn_aruco_markers] No 'markers' list in {yamlPath}")
    return markers, packageShare


def generate_launch_description():
    markers, packageShare = LoadMarkersFromYaml()

    spawnNodes = []

    for m in markers:
        mid = m["id"]
        x = float(m.get("x", 0.0))
        y = float(m.get("y", 0.0))
        z = float(m.get("z", 0.0))
        roll = float(m.get("roll", 0.0))
        pitch = float(m.get("pitch", 0.0))
        yaw = float(m.get("yaw", 0.0)) + pi/2

        sdfPath = os.path.join(
            packageShare, "models", f"aruco_marker_{mid}", "model.sdf"
        )

        if not os.path.exists(sdfPath):
            print(
                f"[spawn_aruco_markers] WARNING: SDF for marker {mid} not found at {sdfPath}"
            )
            continue

        spawnNodes.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_aruco_{mid}",
                output="screen",
                arguments=[
                    "-file",
                    sdfPath,
                    "-name",
                    f"aruco_marker_{mid}",
                    "-allow_renaming",
                    "false",
                    "-x",
                    str(x),
                    "-y",
                    str(y),
                    "-z",
                    str(z),
                    "-R",
                    str(roll),
                    "-P",
                    str(pitch),
                    "-Y",
                    str(yaw),
                ],
            )
        )

    return LaunchDescription(spawnNodes)