#!/usr/bin/env python3
import argparse
import pathlib

import cv2
from cv2 import aruco
import numpy as np

SdfTemplate = """<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="aruco_marker_{id}">
    <static>true</static>
    <link name="link">
      <pose>0 0 -0.0005 0 0 1.5708</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>{size} {size} 0.001 </size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{size} {size} 0.001 </size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <emissive>0 0 0 1</emissive>
          <pbr>
            <metal>
              <albedo_map>materials/textures/aruco_{id}.png</albedo_map>
              <roughness>0.4</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


ModelConfigTemplate = """<?xml version="1.0"?>
<model>
  <name>aruco_marker_{id}</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>generated</name>
    <email>n/a</email>
  </author>
  <description>ArUco marker {id}</description>
</model>
"""


def GenerateMarkerImage(markerId: int, sidePixels: int) -> np.ndarray:
    dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
    img = np.zeros((sidePixels, sidePixels), dtype=np.uint8)
    aruco.drawMarker(dictionary, markerId, sidePixels, img, borderBits=1)

    colorImage = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return colorImage


def Main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dirpath",
        type=str,
        help="Path to root directory",
        default='/home/user/amr/amr_lv4_ws/src/amr_lv4/models',

    )
    parser.add_argument(
        "--ids",
        type=int,
        nargs="+",
        default=[23, 42, 2],
        help="Marker IDs to generate",
    )
    parser.add_argument(
        "--size",
        type=float,
        default=0.3,
        help="Marker physical size in meters (edge length)",
    )
    parser.add_argument(
        "--pixels",
        type=int,
        default=400,
        help="Marker image size in pixels (edge length)",
    )
    args = parser.parse_args()

    packagePath = pathlib.Path(args.dirpath).parent
    modelsDirectory = packagePath / "models"
    modelsDirectory.mkdir(exist_ok=True)

    for markerId in args.ids:
        modelDirectory = modelsDirectory / f"aruco_marker_{markerId}"
        texturesDirectory = modelDirectory / "materials" / "textures"
        texturesDirectory.mkdir(parents=True, exist_ok=True)

        img = GenerateMarkerImage(markerId, args.pixels)
        pngPath = texturesDirectory / f"aruco_{markerId}.png"
        cv2.imwrite(str(pngPath), img)

        sdfText = SdfTemplate.format(id=markerId, size=args.size)
        (modelDirectory / "model.sdf").write_text(sdfText)

        configText = ModelConfigTemplate.format(id=markerId)
        (modelDirectory / "model.config").write_text(configText)

        print(f"Generated model for marker {markerId} at {modelDirectory}")


if __name__ == "__main__":
    Main()