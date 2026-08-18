#!/usr/bin/env python3
"""
Bag Visualizer - Vizualizacija putanje robota iz ROS2 bag snimke

Učitava:
- ROS2 bag datoteku sa odometrijom robota
- Mapu skladišta (depot.pgm i depot.yaml)
- Waypoints (ključne točke)

Prikazuje:
- Kartu skladišta
- Stvarnu putanju robota (iz bag-a)
- Planirane ključne točke
"""

import yaml
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import argparse
import os
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions



class BagVisualizer:
    def __init__(self, bag_path, map_yaml_path):
        self.bag_path = bag_path
        self.map_yaml_path = map_yaml_path
        self.trajectory = []  # Stvarna putanja robota
        self.waypoints = []   # Ciljne točke iz bag-a
        self.map_image = None
        self.map_info = {}
        
    def load_map(self):
        """Učitava mapu iz YAML i PGM datoteka"""
        print(f"Loading map from {self.map_yaml_path}...")
        
        # Učitaj YAML
        with open(self.map_yaml_path, 'r') as f:
            self.map_info = yaml.safe_load(f)
        
        # Učitaj PGM sliku
        map_dir = Path(self.map_yaml_path).parent
        pgm_path = map_dir / self.map_info['image']
        
        if not pgm_path.exists():
            raise FileNotFoundError(f"Map image not found: {pgm_path}")
        
        # Učitaj sliku (grayscale)
        self.map_image = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
        
        if self.map_image is None:
            raise ValueError(f"Failed to load map image: {pgm_path}")
        
        # Invertiraj boje: crno = prepreka, bijelo = slobodno
        self.map_image = 255 - self.map_image
        
        print(f"Map loaded: {self.map_image.shape}")
        print(f"Resolution: {self.map_info['resolution']} m/pixel")
        print(f"Origin: {self.map_info['origin']}")
        
    def extract_trajectory_from_bag(self):
        """Ekstraktira putanju robota i waypoints iz ROS2 bag datoteke koristeći SequentialReader"""
        print(f"📦 Reading bag: {self.bag_path}...")
        
        # Setup storage options
        storage_options = StorageOptions(uri=str(self.bag_path), storage_id='mcap')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        # Create reader
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        
        # Get topic types
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        
        print(f"  Available topics: {list(type_map.keys())}")
        
        # Read all messages
        pozicija_count = 0
        cilja_count = 0
        seen_waypoints = set()  # Za provjeru duplikata
        
        while reader.has_next():
            topic_name, data, timestamp = reader.read_next()
            
            if topic_name == '/pozicija':
                if topic_name in type_map:
                    msg_type = get_message(type_map[topic_name])
                    msg = deserialize_message(data, msg_type)
                    self.trajectory.append((msg.position.x, msg.position.y))
                    pozicija_count += 1
                
            elif topic_name == '/pozicija_cilja':
                if topic_name in type_map:
                    msg_type = get_message(type_map[topic_name])
                    msg = deserialize_message(data, msg_type)
                    # Dodaj waypoint u liste samo ako je novi (eliminiraj duplikate)
                    pos = (round(msg.position.x, 3), round(msg.position.y, 3))
                    if pos not in seen_waypoints:
                        self.waypoints.append(pos)
                        seen_waypoints.add(pos)
                    cilja_count += 1
            
            elif topic_name == '/odom' and pozicija_count == 0:
                # Fallback na /odom ako /pozicija ne postoji
                if topic_name in type_map:
                    msg_type = get_message(type_map[topic_name])
                    msg = deserialize_message(data, msg_type)
                    self.trajectory.append((
                        msg.pose.pose.position.x,
                        msg.pose.pose.position.y
                    ))
        
        print(f"✓ Loaded {pozicija_count} robot positions (/pozicija)")
        if pozicija_count == 0:
            print(f"✓ Fallback: Loaded {len(self.trajectory)} positions from /odom")
        print(f"✓ Loaded {len(self.waypoints)} unique waypoints (/pozicija_cilja from {cilja_count} messages)")
        print(f"Total: {len(self.trajectory)} trajectory points, {len(self.waypoints)} waypoints")
        
    def world_to_map(self, x, y):
        """Pretvara world koordinate u map pixel koordinate"""
        resolution = self.map_info['resolution']
        origin = self.map_info['origin']
        
        map_x = int((x - origin[0]) / resolution)
        map_y = int((y - origin[1]) / resolution)
        
        # Invertiraj Y os (map koordinate rastu prema gore, image niz)
        map_y = self.map_image.shape[0] - map_y
        
        return map_x, map_y
    
    def plot_visualization(self, output_file='trajectory_visualization.png'):
        """Iscrtava mapu, putanju i waypoints"""
        print("Creating visualization...")
        
        fig, ax = plt.subplots(figsize=(16, 12), dpi=150)
        
        # Prikaži mapu
        extent = [
            self.map_info['origin'][0],
            self.map_info['origin'][0] + self.map_image.shape[1] * self.map_info['resolution'],
            self.map_info['origin'][1],
            self.map_info['origin'][1] + self.map_image.shape[0] * self.map_info['resolution']
        ]
        
        ax.imshow(self.map_image, cmap='gray', extent=extent, origin='upper', alpha=0.7)
        
        # Iscrtaj stvarnu putanju
        if self.trajectory:
            traj_x = [p[0] for p in self.trajectory]
            traj_y = [p[1] for p in self.trajectory]
            ax.plot(traj_x, traj_y, 'b-', linewidth=2, alpha=0.6, label='Robot trajectory (from bag)')
            
            # Start i end točke
            ax.plot(traj_x[0], traj_y[0], 'go', markersize=15, label='Start', markeredgecolor='black', markeredgewidth=2)
            ax.plot(traj_x[-1], traj_y[-1], 'ro', markersize=15, label='End', markeredgecolor='black', markeredgewidth=2)
        
        # Iscrtaj waypoints (iz bag datoteke)
        if self.waypoints:
            wp_x = [wp[0] for wp in self.waypoints]
            wp_y = [wp[1] for wp in self.waypoints]
            ax.scatter(wp_x, wp_y, c='red', s=200, marker='*', 
                       edgecolors='yellow', linewidths=2, label='Goal waypoints', zorder=5)
            
            # Označi waypoints brojevima
            for idx, (x, y) in enumerate(self.waypoints, 1):
                ax.annotate(f'{idx}', xy=(x, y), xytext=(5, 5), 
                           textcoords='offset points', fontsize=12, 
                           fontweight='bold', color='white',
                           bbox=dict(boxstyle='round,pad=0.3', facecolor='red', alpha=0.7))
        else:
            print("⚠ No waypoints to display")
        
        ax.set_xlabel('X [m]', fontsize=14)
        ax.set_ylabel('Y [m]', fontsize=14)
        ax.set_title('Robot Navigation - Depot Map\nTrajectory from ROS2 Bag + Waypoints', 
                     fontsize=16, fontweight='bold')
        ax.legend(loc='upper right', fontsize=12)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        plt.tight_layout()
        plt.savefig(output_file, bbox_inches='tight')
        print(f"Visualization saved to: {output_file}")
        plt.show()
        
    def run(self, output_file='trajectory_visualization.png'):
        """Pokreni cijeli proces vizualizacije"""
        try:
            self.load_map()
            self.extract_trajectory_from_bag()
            self.plot_visualization(output_file)
            print("✓ Visualization completed successfully!")
        except Exception as e:
            print(f"✗ Error: {e}")
            import traceback
            traceback.print_exc()


def main():
    parser = argparse.ArgumentParser(description='Visualize robot trajectory from ROS2 bag')
    parser.add_argument('bag_path', help='Path to ROS2 bag directory')
    parser.add_argument('--map', default='~/Documents/ros2_lv/amr_lv_ws/src/lv2/robot_nav2_bringup/maps/depot.yaml',
                       help='Path to map YAML file')
    parser.add_argument('--output', default='trajectory_visualization.png',
                       help='Output image file name')
    
    args = parser.parse_args()
    
    # Expandiraj ~ u putanji
    map_path = os.path.expanduser(args.map)
    
    visualizer = BagVisualizer(args.bag_path, map_path)
    visualizer.run(args.output)


if __name__ == '__main__':
    main()
