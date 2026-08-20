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
        
        self.patrolFile = self.get_parameter('patrol_file').value
        self.imageTopic = self.get_parameter('image_topic').value
        self.cameraInfoTopic = self.get_parameter('camera_info_topic').value
        self.cameraFrame = self.get_parameter('camera_frame').value
        self.markerSize = self.get_parameter('marker_size').value
        self.stopDistance = self.get_parameter('stop_distance').value
        self.waitTime = self.get_parameter('wait_time').value
        self.detectionCooldown = self.get_parameter('detection_cooldown').value
        
        # ========== CV BRIDGE & ARUCO ==========
        self.bridge = CvBridge()
        self.arucoDictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.arucoParameters = aruco.DetectorParameters_create()
        
        # Camera intrinsics (bit će popunjeno iz camera_info)
        self.cameraMatrix = None
        self.distortionCoefficients = None
        
        # ========== TF2 ==========
        self.transformBuffer = Buffer()
        self.transformListener = TransformListener(self.transformBuffer, self)
        
        # ========== NAV2 ==========
        self.navigator = BasicNavigator()
        self.get_logger().info('Čekam da Nav2 bude spreman...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 je spreman!')
        
        # ========== PATROL POINTS ==========
        self.patrolPoints = self.LoadPatrolPoints()
        if not self.patrolPoints:
            self.get_logger().error('Nema patrol točaka! Izlazim.')
            return
        self.get_logger().info(f'Učitano {len(self.patrolPoints)} patrol točaka')
        
        # ========== STATE MACHINE ==========
        self.state = 'PATROLLING'  # PATROLLING, GOING_TO_MARKER, WAITING_AT_MARKER
        self.currentWaypointIndex = 0
        self.currentDirection = 'forward'
        self.isNavigating = False
        
        # Marker tracking
        self.visitedMarkers = set()  # Markeri koje smo već posjetili (trajno ignoriraj)
        self.currentMarkerId = None
        self.waitEndTime = None
        self.lastGoalUpdateTime = 0.0  # Vrijeme zadnjeg ažuriranja cilja
        self.goalUpdateInterval = 1.0  # Ažuriraj cilj svakih 1 sekundu
        
        # ========== SUBSCRIBERS ==========
        self.imageSubscription = self.create_subscription(
            Image, self.imageTopic, self.ImageCallback, 1
        )
        self.cameraInfoSubscription = self.create_subscription(
            CameraInfo, self.cameraInfoTopic, self.CameraInfoCallback, 1
        )
        
        # ========== PUBLISHER ZA ZAUSTAVLJANJE ==========
        # Šaljemo direktno na cmd_vel_nav (nakon collision_monitora) za eksplicitno zaustavljanje
        self.velocityPublisher = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        
        # ========== TIMER ==========
        self.timer = self.create_timer(0.5, self.PatrolCallback)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('ArUco Patrol Node pokrenut!')
        self.get_logger().info(f'  - Image topic: {self.imageTopic}')
        self.get_logger().info(f'  - Camera frame: {self.cameraFrame}')
        self.get_logger().info(f'  - Marker size: {self.markerSize}m')
        self.get_logger().info(f'  - Stop distance: {self.stopDistance}m')
        self.get_logger().info(f'  - Wait time: {self.waitTime}s')
        self.get_logger().info('=' * 60)
    
    # ==================== LOAD PATROL POINTS ====================
    def LoadPatrolPoints(self):
        """Učitaj patrol točke iz YAML datoteke."""
        if not os.path.exists(self.patrolFile):
            self.get_logger().error(f'Datoteka ne postoji: {self.patrolFile}')
            return []
        
        try:
            with open(self.patrolFile, 'r') as f:
                data = yaml.safe_load(f)
                points = data.get('patrol_points', [])
                return points
        except Exception as e:
            self.get_logger().error(f'Greška pri učitavanju: {e}')
            return []
    
    # ==================== CAMERA INFO CALLBACK ====================
    def CameraInfoCallback(self, msg: CameraInfo):
        """Primi parametre kamere."""
        if self.cameraMatrix is None:
            self.cameraMatrix = np.array(msg.k).reshape((3, 3))
            self.distortionCoefficients = np.array(msg.d)
            self.get_logger().info('Primljeni parametri kamere!')
    
    # ==================== IMAGE CALLBACK ====================
    def ImageCallback(self, msg: Image):
        """Obradi sliku i detektiraj ArUco markere."""
        # Detektiraj u PATROLLING ili GOING_TO_MARKER stanju
        if self.state not in ['PATROLLING', 'GOING_TO_MARKER']:
            return
        
        if self.cameraMatrix is None:
            return
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'CV Bridge greška: {e}')
            return
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detektiraj markere
        corners, ids, rejectedCandidates = aruco.detectMarkers(
            gray, self.arucoDictionary, parameters=self.arucoParameters
        )
        
        if ids is not None and len(ids) > 0:
            # Estimiraj pozu markera
            rvecs, tvecs, objectPoints = aruco.estimatePoseSingleMarkers(
                corners, self.markerSize, self.cameraMatrix, self.distortionCoefficients
            )
            
            for i, markerId in enumerate(ids.flatten()):
                currentTime = time.time()
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]
                
                # Ako idemo prema markeru, ažuriraj cilj periodički
                if self.state == 'GOING_TO_MARKER' and markerId == self.currentMarkerId:
                    # Provjeri interval ažuriranja
                    if currentTime - self.lastGoalUpdateTime >= self.goalUpdateInterval:
                        self.UpdateMarkerGoal(tvec, rvec, msg.header.stamp)
                    continue
                
                # Ako patroliramo, provjeri je li marker već posjećen
                if self.state == 'PATROLLING':
                    if markerId in self.visitedMarkers:
                        continue  # Preskoči, već smo posjetili ovaj marker
                    
                    # Pronađen novi marker!
                    self.get_logger().info(f'Detektiran ArUco marker ID: {markerId}')
                    self.get_logger().info(f'  Pozicija u kameri: x={tvec[0]:.3f}, y={tvec[1]:.3f}, z={tvec[2]:.3f}')
                    
                    # Izračunaj cilj u map frameu
                    goalPose = self.CalculateGoalPose(tvec, rvec, msg.header.stamp)
                    
                    if goalPose is not None:
                        self.InterruptPatrolForMarker(markerId, goalPose)
                        break  # Obradi samo prvi marker
    
    # ==================== UPDATE MARKER GOAL ====================
    def UpdateMarkerGoal(self, tvec, rvec, stamp):
        """Ažuriraj cilj navigacije prema markeru s novom pozicijom."""
        # Ako smo jako blizu markera (< 1m), ne ažuriraj više cilj
        distanceToMarker = np.linalg.norm(tvec)
        if distanceToMarker < 1.0:
            self.get_logger().info(f'Blizu markera ({distanceToMarker:.2f}m), ne ažuriram cilj')
            return
        
        goalPose = self.CalculateGoalPose(tvec, rvec, stamp)
        
        if goalPose is not None:
            self.get_logger().info(f'Ažuriram cilj za marker {self.currentMarkerId} (udaljenost: {distanceToMarker:.2f}m)')
            self.get_logger().info(f'  Nova pozicija: x={goalPose.pose.position.x:.3f}, y={goalPose.pose.position.y:.3f}')
            
            # Ažuriraj navigacijski cilj
            self.navigator.goToPose(goalPose)
            self.lastGoalUpdateTime = time.time()
    
    # ==================== CALCULATE GOAL POSE ====================
    def CalculateGoalPose(self, tvec, rvec, stamp):
        """Izračunaj cilj ispred markera u map frameu.
        
        Robot se pozicionira na stop_distance ispred markera,
        centriran i okrenut direktno prema markeru.
        """
        try:
            # Dohvati transform camera -> map
            transform = self.transformBuffer.lookup_transform(
                'map',
                self.cameraFrame,
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
        cameraToMapRotation = self.QuaternionToRotationMatrix(q.x, q.y, q.z, q.w)
        cameraToMapTranslation = np.array([t.x, t.y, t.z])
        
        # Marker pozicija u map frameu
        markerInMap = cameraToMapRotation @ tvec + cameraToMapTranslation
        
        # Kamera pozicija u map frameu (origin transformacije)
        cameraInMap = cameraToMapTranslation
        
        # Vektor od markera prema kameri (smjer u kojem robot treba stati)
        direction = cameraInMap - markerInMap
        direction[2] = 0.0  # Ignoriraj Z komponentu (ostajemo na tlu)
        
        # Normaliziraj smjer
        distance = np.linalg.norm(direction[:2])
        if distance < 0.01:
            self.get_logger().warn('Marker preblizu kameri!')
            return None
        
        normalizedDirection = direction / distance
        
        # Ciljna pozicija: stop_distance ispred markera, u smjeru kamere
        goalX = markerInMap[0] + normalizedDirection[0] * self.stopDistance
        goalY = markerInMap[1] + normalizedDirection[1] * self.stopDistance
        
        # Orijentacija: robot gleda PREMA markeru (suprotno od direction)
        yaw = math.atan2(-normalizedDirection[1], -normalizedDirection[0])
        
        # Kreiraj PoseStamped
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(goalX)
        goal.pose.position.y = float(goalY)
        goal.pose.position.z = 0.0
        
        # Yaw to quaternion
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)
        
        self.get_logger().info(f'  Marker u map: x={markerInMap[0]:.3f}, y={markerInMap[1]:.3f}')
        self.get_logger().info(f'  Cilj u map: x={goalX:.3f}, y={goalY:.3f}, yaw={math.degrees(yaw):.1f}°')
        
        return goal
    
    def QuaternionToRotationMatrix(self, x, y, z, w):
        """Pretvori quaternion u rotacijsku matricu."""
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
        return R
    
    # ==================== INTERRUPT PATROL ====================
    def InterruptPatrolForMarker(self, markerId, goalPose):
        """Prekini patroliranje i idi do markera."""
        self.get_logger().info(f'Prekidam patroliranje za marker {markerId}!')
        
        # Zaustavi trenutnu navigaciju
        if self.isNavigating:
            self.navigator.cancelTask()
            self.isNavigating = False
        
        # Postavi stanje
        self.state = 'GOING_TO_MARKER'
        self.currentMarkerId = markerId
        self.lastGoalUpdateTime = time.time()  # Resetiraj vrijeme ažuriranja
        
        # Navigiraj do markera
        self.navigator.goToPose(goalPose)
        self.isNavigating = True
    
    # ==================== STOP ROBOT ====================
    def StopRobot(self):
        """Eksplicitno zaustavi robota slanjem nulte brzine."""
        stopCommand = Twist()
        stopCommand.linear.x = 0.0
        stopCommand.linear.y = 0.0
        stopCommand.angular.z = 0.0
        self.velocityPublisher.publish(stopCommand)
    
    # ==================== PATROL CALLBACK ====================
    def PatrolCallback(self):
        """Glavni patrol loop."""
        
        # ===== GOING TO MARKER =====
        if self.state == 'GOING_TO_MARKER':
            if not self.navigator.isTaskComplete():
                return
            
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Stigao ispred markera {self.currentMarkerId}!')
                self.get_logger().info(f'Čekam {self.waitTime} sekundi...')
                # Eksplicitno zaustavi robota
                self.StopRobot()
                self.state = 'WAITING_AT_MARKER'
                self.waitEndTime = time.time() + self.waitTime
                self.isNavigating = False
            else:
                self.get_logger().warn('Neuspješna navigacija do markera. Nastavljam patroliranje.')
                self.StopRobot()
                self.state = 'PATROLLING'
                self.isNavigating = False
            return
        
        # ===== WAITING AT MARKER =====
        if self.state == 'WAITING_AT_MARKER':
            # Kontinuirano šalji nultu brzinu da se robot ne miče
            self.StopRobot()
            if time.time() >= self.waitEndTime:
                self.get_logger().info('Čekanje završeno. Nastavljam patroliranje!')
                # Dodaj marker u listu posjećenih
                self.visitedMarkers.add(self.currentMarkerId)
                self.get_logger().info(f'Marker {self.currentMarkerId} dodan u posjećene. Ukupno posjećeno: {len(self.visitedMarkers)}')
                self.state = 'PATROLLING'
                self.currentMarkerId = None
                self.waitEndTime = None
            return
        
        # ===== PATROLLING =====
        if self.state == 'PATROLLING':
            if self.isNavigating:
                if not self.navigator.isTaskComplete():
                    return
                
                result = self.navigator.getResult()
                
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f'Stigao na waypoint {self.currentWaypointIndex}')
                    self.AdvanceWaypoint()
                elif result == TaskResult.CANCELED:
                    self.get_logger().warn('Navigacija prekinuta!')
                elif result == TaskResult.FAILED:
                    self.get_logger().error('Navigacija neuspješna!')
                    self.AdvanceWaypoint()
                
                self.isNavigating = False
            else:
                # Navigiraj do sljedećeg waypoineta
                self.NavigateToCurrentWaypoint()
    
    def AdvanceWaypoint(self):
        """Pomakni se na sljedeći waypoint."""
        if self.currentDirection == 'forward':
            self.currentWaypointIndex += 1
            if self.currentWaypointIndex >= len(self.patrolPoints):
                self.get_logger().info('Kraj patrole, mijenjam smjer na backward')
                self.currentDirection = 'backward'
                self.currentWaypointIndex = len(self.patrolPoints) - 1
        else:
            self.currentWaypointIndex -= 1
            if self.currentWaypointIndex < 0:
                self.get_logger().info('Početak patrole, mijenjam smjer na forward')
                self.currentDirection = 'forward'
                self.currentWaypointIndex = 0
    
    def NavigateToCurrentWaypoint(self):
        """Navigiraj do trenutnog waypoineta."""
        if 0 <= self.currentWaypointIndex < len(self.patrolPoints):
            point = self.patrolPoints[self.currentWaypointIndex]
            
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = float(point['x'])
            goal.pose.position.y = float(point['y'])
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0
            
            self.get_logger().info(
                f'Navigiram do waypoint {self.currentWaypointIndex} '
                f'({self.currentDirection}): ({point["x"]:.2f}, {point["y"]:.2f})'
            )
            
            self.navigator.goToPose(goal)
            self.isNavigating = True
    
    # ==================== SHUTDOWN ====================
    def Shutdown(self):
        """Čisto gašenje."""
        self.get_logger().info('Gasim ArUco Patrol Node...')
        if self.isNavigating:
            self.navigator.cancelTask()


def Main(args=None):
    rclpy.init(args=args)
    node = ArucoPatrolNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.Shutdown()
        node.destroy_node()
        rclpy.Shutdown()


if __name__ == '__main__':
    Main()
