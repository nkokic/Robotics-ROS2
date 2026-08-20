#!/usr/bin/env python3
"""
ArUco Patrol Node - patrolira kroz točke i zaustavlja se ispred detektiranih ArUco markera.

Funkcionalnosti:
1. Patrolira kroz definirane točke (patrol_points.yaml)
2. Detektira ArUco markere pomoću RGB kamere
3. Kada detektira marker, navigira do pozicije ispred markera
4. Ostaje ispred markera definirano vrijeme
5. Nastavlja s patroliranjem
"""

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import cv2
from cv2 import aruco
import numpy as np
import yaml
import os
import math
import time

from tf2_ros import Buffer, TransformListener, TransformException


class ArucoPatrolNode(Node):
    def __init__(self):
        super().__init__('aruco_patrol_node')
        
        # ========== PARAMETRI ==========
        self.declare_parameter('patrol_file', 'patrol_points.yaml')
        self.declare_parameter('image_topic', '/oak/rgb/color')
        self.declare_parameter('camera_info_topic', '/oak/rgb/camera_info')
        self.declare_parameter('camera_frame', 'oak_rgb_camera_optical_frame')
        self.declare_parameter('marker_size', 0.2)  # Veličina markera u metrima
        self.declare_parameter('stop_distance', 0.5)  # Udaljenost zaustavljanja ispred markera
        self.declare_parameter('wait_time', 5.0)  # Vrijeme čekanja ispred markera (sekunde)
        self.declare_parameter('detection_cooldown', 30.0)  # Cooldown prije ponovne detekcije istog markera
        
        self.patrol_file = self.get_parameter('patrol_file').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.marker_size = self.get_parameter('marker_size').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.wait_time = self.get_parameter('wait_time').value
        self.detection_cooldown = self.get_parameter('detection_cooldown').value
        
        # ========== CV BRIDGE & ARUCO ==========
        self.bridge = CvBridge()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        
        # Camera intrinsics (bit će popunjeno iz camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # ========== TF2 ==========
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ========== NAV2 ==========
        self.navigator = BasicNavigator()
        self.get_logger().info('Čekam da Nav2 bude spreman...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 je spreman!')
        
        # ========== PATROL POINTS ==========
        self.patrol_points = self.load_patrol_points()
        if not self.patrol_points:
            self.get_logger().error('Nema patrol točaka! Izlazim.')
            return
        self.get_logger().info(f'Učitano {len(self.patrol_points)} patrol točaka')
        
        # ========== STATE MACHINE ==========
        self.state = 'PATROLLING'  # PATROLLING, GOING_TO_MARKER, WAITING_AT_MARKER
        self.current_waypoint_index = 0
        self.current_direction = 'forward'
        self.is_navigating = False
        
        # Marker tracking
        self.visited_markers = set()  # Markeri koje smo već posjetili (trajno ignoriraj)
        self.current_marker_id = None
        self.wait_end_time = None
        self.last_goal_update_time = 0.0  # Vrijeme zadnjeg ažuriranja cilja
        self.goal_update_interval = 1.0  # Ažuriraj cilj svakih 1 sekundu
        
        # ========== SUBSCRIBERS ==========
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 1
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 1
        )
        
        # ========== PUBLISHER ZA ZAUSTAVLJANJE ==========
        # Šaljemo direktno na cmd_vel_nav (nakon collision_monitora) za eksplicitno zaustavljanje
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        
        # ========== TIMER ==========
        self.timer = self.create_timer(0.5, self.patrol_callback)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('ArUco Patrol Node pokrenut!')
        self.get_logger().info(f'  - Image topic: {self.image_topic}')
        self.get_logger().info(f'  - Camera frame: {self.camera_frame}')
        self.get_logger().info(f'  - Marker size: {self.marker_size}m')
        self.get_logger().info(f'  - Stop distance: {self.stop_distance}m')
        self.get_logger().info(f'  - Wait time: {self.wait_time}s')
        self.get_logger().info('=' * 60)
    
    # ==================== LOAD PATROL POINTS ====================
    def load_patrol_points(self):
        """Učitaj patrol točke iz YAML datoteke."""
        if not os.path.exists(self.patrol_file):
            self.get_logger().error(f'Datoteka ne postoji: {self.patrol_file}')
            return []
        
        try:
            with open(self.patrol_file, 'r') as f:
                data = yaml.safe_load(f)
                points = data.get('patrol_points', [])
                return points
        except Exception as e:
            self.get_logger().error(f'Greška pri učitavanju: {e}')
            return []
    
    # ==================== CAMERA INFO CALLBACK ====================
    def camera_info_callback(self, msg: CameraInfo):
        """Primi parametre kamere."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Primljeni parametri kamere!')
    
    # ==================== IMAGE CALLBACK ====================
    def image_callback(self, msg: Image):
        """Obradi sliku i detektiraj ArUco markere."""
        # Detektiraj u PATROLLING ili GOING_TO_MARKER stanju
        if self.state not in ['PATROLLING', 'GOING_TO_MARKER']:
            return
        
        if self.camera_matrix is None:
            return
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'CV Bridge greška: {e}')
            return
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detektiraj markere
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None and len(ids) > 0:
            # Estimiraj pozu markera
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            
            for i, marker_id in enumerate(ids.flatten()):
                current_time = time.time()
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]
                
                # Ako idemo prema markeru, ažuriraj cilj periodički
                if self.state == 'GOING_TO_MARKER' and marker_id == self.current_marker_id:
                    # Provjeri interval ažuriranja
                    if current_time - self.last_goal_update_time >= self.goal_update_interval:
                        self.update_marker_goal(tvec, rvec, msg.header.stamp)
                    continue
                
                # Ako patroliramo, provjeri je li marker već posjećen
                if self.state == 'PATROLLING':
                    if marker_id in self.visited_markers:
                        continue  # Preskoči, već smo posjetili ovaj marker
                    
                    # Pronađen novi marker!
                    self.get_logger().info(f'Detektiran ArUco marker ID: {marker_id}')
                    self.get_logger().info(f'  Pozicija u kameri: x={tvec[0]:.3f}, y={tvec[1]:.3f}, z={tvec[2]:.3f}')
                    
                    # Izračunaj cilj u map frameu
                    goal_pose = self.calculate_goal_pose(tvec, rvec, msg.header.stamp)
                    
                    if goal_pose is not None:
                        self.interrupt_patrol_for_marker(marker_id, goal_pose)
                        break  # Obradi samo prvi marker
    
    # ==================== UPDATE MARKER GOAL ====================
    def update_marker_goal(self, tvec, rvec, stamp):
        """Ažuriraj cilj navigacije prema markeru s novom pozicijom."""
        # Ako smo jako blizu markera (< 1m), ne ažuriraj više cilj
        distance_to_marker = np.linalg.norm(tvec)
        if distance_to_marker < 1.0:
            self.get_logger().info(f'Blizu markera ({distance_to_marker:.2f}m), ne ažuriram cilj')
            return
        
        goal_pose = self.calculate_goal_pose(tvec, rvec, stamp)
        
        if goal_pose is not None:
            self.get_logger().info(f'Ažuriram cilj za marker {self.current_marker_id} (udaljenost: {distance_to_marker:.2f}m)')
            self.get_logger().info(f'  Nova pozicija: x={goal_pose.pose.position.x:.3f}, y={goal_pose.pose.position.y:.3f}')
            
            # Ažuriraj navigacijski cilj
            self.navigator.goToPose(goal_pose)
            self.last_goal_update_time = time.time()
    
    # ==================== CALCULATE GOAL POSE ====================
    def calculate_goal_pose(self, tvec, rvec, stamp):
        """Izračunaj cilj ispred markera u map frameu.
        
        Robot se pozicionira na stop_distance ispred markera,
        centriran i okrenut direktno prema markeru.
        """
        try:
            # Dohvati transform camera -> map
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except TransformException as e:
            self.get_logger().warn(f'TF greška: {e}')
            return None
        
        # Transform camera -> map
        t = transform.transform.translation
        q = transform.transform.rotation
        
        # Quaternion to rotation matrix
        R_cam_to_map = self.quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        T_cam_to_map = np.array([t.x, t.y, t.z])
        
        # Marker pozicija u map frameu
        marker_in_map = R_cam_to_map @ tvec + T_cam_to_map
        
        # Kamera pozicija u map frameu (origin transformacije)
        camera_in_map = T_cam_to_map
        
        # Vektor od markera prema kameri (smjer u kojem robot treba stati)
        direction = camera_in_map - marker_in_map
        direction[2] = 0.0  # Ignoriraj Z komponentu (ostajemo na tlu)
        
        # Normaliziraj smjer
        distance = np.linalg.norm(direction[:2])
        if distance < 0.01:
            self.get_logger().warn('Marker preblizu kameri!')
            return None
        
        direction_normalized = direction / distance
        
        # Ciljna pozicija: stop_distance ispred markera, u smjeru kamere
        goal_x = marker_in_map[0] + direction_normalized[0] * self.stop_distance
        goal_y = marker_in_map[1] + direction_normalized[1] * self.stop_distance
        
        # Orijentacija: robot gleda PREMA markeru (suprotno od direction)
        yaw = math.atan2(-direction_normalized[1], -direction_normalized[0])
        
        # Kreiraj PoseStamped
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(goal_x)
        goal.pose.position.y = float(goal_y)
        goal.pose.position.z = 0.0
        
        # Yaw to quaternion
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)
        
        self.get_logger().info(f'  Marker u map: x={marker_in_map[0]:.3f}, y={marker_in_map[1]:.3f}')
        self.get_logger().info(f'  Cilj u map: x={goal_x:.3f}, y={goal_y:.3f}, yaw={math.degrees(yaw):.1f}°')
        
        return goal
    
    def quaternion_to_rotation_matrix(self, x, y, z, w):
        """Pretvori quaternion u rotacijsku matricu."""
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
        return R
    
    # ==================== INTERRUPT PATROL ====================
    def interrupt_patrol_for_marker(self, marker_id, goal_pose):
        """Prekini patroliranje i idi do markera."""
        self.get_logger().info(f'Prekidam patroliranje za marker {marker_id}!')
        
        # Zaustavi trenutnu navigaciju
        if self.is_navigating:
            self.navigator.cancelTask()
            self.is_navigating = False
        
        # Postavi stanje
        self.state = 'GOING_TO_MARKER'
        self.current_marker_id = marker_id
        self.last_goal_update_time = time.time()  # Resetiraj vrijeme ažuriranja
        
        # Navigiraj do markera
        self.navigator.goToPose(goal_pose)
        self.is_navigating = True
    
    # ==================== STOP ROBOT ====================
    def stop_robot(self):
        """Eksplicitno zaustavi robota slanjem nulte brzine."""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)
    
    # ==================== PATROL CALLBACK ====================
    def patrol_callback(self):
        """Glavni patrol loop."""
        
        # ===== GOING TO MARKER =====
        if self.state == 'GOING_TO_MARKER':
            if not self.navigator.isTaskComplete():
                return
            
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Stigao ispred markera {self.current_marker_id}!')
                self.get_logger().info(f'Čekam {self.wait_time} sekundi...')
                # Eksplicitno zaustavi robota
                self.stop_robot()
                self.state = 'WAITING_AT_MARKER'
                self.wait_end_time = time.time() + self.wait_time
                self.is_navigating = False
            else:
                self.get_logger().warn('Neuspješna navigacija do markera. Nastavljam patroliranje.')
                self.stop_robot()
                self.state = 'PATROLLING'
                self.is_navigating = False
            return
        
        # ===== WAITING AT MARKER =====
        if self.state == 'WAITING_AT_MARKER':
            # Kontinuirano šalji nultu brzinu da se robot ne miče
            self.stop_robot()
            if time.time() >= self.wait_end_time:
                self.get_logger().info('Čekanje završeno. Nastavljam patroliranje!')
                # Dodaj marker u listu posjećenih
                self.visited_markers.add(self.current_marker_id)
                self.get_logger().info(f'Marker {self.current_marker_id} dodan u posjećene. Ukupno posjećeno: {len(self.visited_markers)}')
                self.state = 'PATROLLING'
                self.current_marker_id = None
                self.wait_end_time = None
            return
        
        # ===== PATROLLING =====
        if self.state == 'PATROLLING':
            if self.is_navigating:
                if not self.navigator.isTaskComplete():
                    return
                
                result = self.navigator.getResult()
                
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f'Stigao na waypoint {self.current_waypoint_index}')
                    self.advance_waypoint()
                elif result == TaskResult.CANCELED:
                    self.get_logger().warn('Navigacija prekinuta!')
                elif result == TaskResult.FAILED:
                    self.get_logger().error('Navigacija neuspješna!')
                    self.advance_waypoint()
                
                self.is_navigating = False
            else:
                # Navigiraj do sljedećeg waypoineta
                self.navigate_to_current_waypoint()
    
    def advance_waypoint(self):
        """Pomakni se na sljedeći waypoint."""
        if self.current_direction == 'forward':
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.patrol_points):
                self.get_logger().info('Kraj patrole, mijenjam smjer na backward')
                self.current_direction = 'backward'
                self.current_waypoint_index = len(self.patrol_points) - 1
        else:
            self.current_waypoint_index -= 1
            if self.current_waypoint_index < 0:
                self.get_logger().info('Početak patrole, mijenjam smjer na forward')
                self.current_direction = 'forward'
                self.current_waypoint_index = 0
    
    def navigate_to_current_waypoint(self):
        """Navigiraj do trenutnog waypoineta."""
        if 0 <= self.current_waypoint_index < len(self.patrol_points):
            point = self.patrol_points[self.current_waypoint_index]
            
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = float(point['x'])
            goal.pose.position.y = float(point['y'])
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0
            
            self.get_logger().info(
                f'Navigiram do waypoint {self.current_waypoint_index} '
                f'({self.current_direction}): ({point["x"]:.2f}, {point["y"]:.2f})'
            )
            
            self.navigator.goToPose(goal)
            self.is_navigating = True
    
    # ==================== SHUTDOWN ====================
    def shutdown(self):
        """Čisto gašenje."""
        self.get_logger().info('Gasim ArUco Patrol Node...')
        if self.is_navigating:
            self.navigator.cancelTask()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPatrolNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
