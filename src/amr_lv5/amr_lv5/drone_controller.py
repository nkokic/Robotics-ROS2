#!/usr/bin/env python3
# zadatak2_mission.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
import numpy as np
import math
import time

class DroneMissionController(Node):
    def __init__(self):
        super().__init__('drone_mission_controller')

        # Parametri
        self.declare_parameter('target_distance', 0.6)
        self.declare_parameter('search_speed', 0.2)
        self.target_dist = self.get_parameter('target_distance').value
        self.valid_ids = [23, 42]
        
        # PID gainovi
        self.kp_lin = 0.3
        self.kp_ang = 0.5

        # Publisheri
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/takeoff', 1)
        self.land_pub = self.create_publisher(Empty, '/land', 1)
        
        # Subscriberi
        # SLUŠAMO TEMU IZ PRVOG ZADATKA umjesto kamere
        self.aruco_sub = self.create_subscription(PoseStamped, '/aruco_pose_detected', self.aruco_data_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Varijable
        self.state = 'INIT'
        self.start_pose = None
        self.current_pose = None
        self.state_start_time = 0.0
        
        self.visited_markers = set()
        self.current_target_marker = None
        self.marker_pos = None
        
        # Logika potvrde
        self.detection_counter = 0
        self.required_detections = 5
        self.last_seen_id = -1
        self.last_msg_time = 0

        self.get_logger().info("Z2: Kontroler čeka podatke od Z1...")
        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.start_pose is None and self.state == 'INIT':
            self.start_pose = msg.pose.pose

    def aruco_data_callback(self, msg):
        """Ova funkcija se poziva kad Zadatak 1 detektira marker."""
        try:
            # ID markera smo poslali u frame_id polju stringa
            marker_id = int(msg.header.frame_id)
        except ValueError:
            return

        # Filtriranje (već smo posjetili?)
        if marker_id in self.visited_markers and self.state != 'BACKING_UP':
            return

        # Debouncing logika
        if marker_id == self.last_seen_id:
            self.detection_counter += 1
        else:
            self.detection_counter = 1
            self.last_seen_id = marker_id
        
        # Spremi poziciju
        self.marker_pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.last_msg_time = time.time()

        # Ako je potvrđen
        if self.detection_counter >= self.required_detections:
            self.current_target_marker = marker_id
            # Održavaj counter visokim da ne resetira
            self.detection_counter = self.required_detections

    def send_vel(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = float(x), float(y), float(z)
        msg.angular.z = float(yaw)
        self.cmd_vel_pub.publish(msg)

    def control_loop(self):
        curr_time = time.time()
        
        # 1. POLIJETANJE
        if self.state == 'INIT':
            if self.start_pose:
                self.takeoff_pub.publish(Empty())
                self.state_start_time = curr_time
                self.state = 'TAKING_OFF'
        
        elif self.state == 'TAKING_OFF':
            if (curr_time - self.state_start_time) < 8.0:
                self.send_vel(0, 0, 0.3, 0)
            else:
                self.send_vel(0, 0, 0, 0)
                self.state = 'SEARCHING'

        # 2. TRAŽENJE
        elif self.state == 'SEARCHING':
            if all(m in self.visited_markers for m in self.valid_ids):
                self.get_logger().info("Kraj. Povratak kući.", throttle_duration_sec=2.0)
                self.state = 'RETURNING'
                return

            # Je li zadnja poruka od Z1 bila nedavno?
            is_valid = (curr_time - self.last_msg_time < 0.5) and (self.detection_counter >= self.required_detections)
            
            if is_valid:
                self.get_logger().info(f"Vidim ID {self.current_target_marker}! Prilazim.")
                self.send_vel(0,0,0,0)
                self.state = 'APPROACHING'
            else:
                self.get_logger().info("Tražim...", throttle_duration_sec=2.0)
                self.send_vel(0, 0, 0, self.get_parameter('search_speed').value)

        # 3. PRILAZAK
        elif self.state == 'APPROACHING':
            if (curr_time - self.last_msg_time) > 2.0:
                self.get_logger().warn("Izgubio signal od Z1. Stop.")
                self.send_vel(0,0,0,0)
                self.state = 'SEARCHING'
                return

            # Čitamo koordinate koje nam je poslao Z1
            x, y, z = self.marker_pos
            err_dist = z - self.target_dist
            
            if err_dist < 0.1:
                self.get_logger().info(f"ODRADIO MARKER {self.current_target_marker}")
                self.visited_markers.add(self.current_target_marker)
                self.send_vel(0,0,0,0)
                self.state = 'BACKING_UP'
                self.state_start_time = curr_time
                return

            # Regulacija
            vx = np.clip(self.kp_lin * err_dist, -0.2, 0.4)
            vyaw = np.clip(-self.kp_ang * x, -0.4, 0.4)
            vz = np.clip(-self.kp_lin * y, -0.2, 0.2)
            self.send_vel(vx, 0, vz, vyaw)

        # 4. ODMICANJE
        elif self.state == 'BACKING_UP':
            if (curr_time - self.state_start_time) < 2.0:
                self.send_vel(-0.3, 0, 0, 0)
            else:
                self.state = 'SEARCHING'
                self.detection_counter = 0

        # 5. POVRATAK I SLIJETANJE
        elif self.state == 'RETURNING':
            # ... (Logika povratka ista kao prije) ...
            cx, cy = self.current_pose.position.x, self.current_pose.position.y
            sx, sy = self.start_pose.position.x, self.start_pose.position.y
            dist = math.sqrt((sx-cx)**2 + (sy-cy)**2)
            
            if dist < 0.4:
                self.send_vel(0,0,0,0)
                self.state = 'LANDING'
                self.state_start_time = curr_time
            else:
                target_yaw = math.atan2(sy-cy, sx-cx)
                q = self.current_pose.orientation
                curr_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
                yaw_err = target_yaw - curr_yaw
                while yaw_err > math.pi: yaw_err -= 2*math.pi
                while yaw_err < -math.pi: yaw_err += 2*math.pi
                
                if abs(yaw_err) > 0.3:
                    self.send_vel(0,0,0, 0.5 * np.sign(yaw_err))
                else:
                    self.send_vel(0.3,0,0, 0.5 * yaw_err)

        elif self.state == 'LANDING':
            self.send_vel(0, 0, -0.3, 0)
            self.land_pub.publish(Empty())
            if (curr_time - self.state_start_time) > 8.0:
                self.send_vel(0,0,0,0)
                self.state = 'FINISHED'
                self.get_logger().info("Gotovo.")
        
        elif self.state == 'FINISHED':
            pass

def main(args=None):
    rclpy.init(args=args)
    node = DroneMissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()