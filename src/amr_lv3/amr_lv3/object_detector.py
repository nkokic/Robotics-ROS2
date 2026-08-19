#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import tf2_ros


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # CV Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()
        
        # TF2 buffer and listener for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Camera intrinsics (will be updated from camera_info)
        self.camera_matrix = None
        self.camera_frame = None
        
        # Declare parameters for HSV thresholds (for yellow color detection)
        self.declare_parameter('hsv_lower', [20, 100, 100])
        self.declare_parameter('hsv_upper', [30, 255, 255])
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('min_area', 500.0)  # Minimum contour area to consider
        
        # Get parameters
        hsv_lower = self.get_parameter('hsv_lower').value
        hsv_upper = self.get_parameter('hsv_upper').value
        self.target_frame = self.get_parameter('target_frame').value
        self.min_area = self.get_parameter('min_area').value
        
        self.hsv_lower = np.array(hsv_lower)
        self.hsv_upper = np.array(hsv_upper)
        
        # Subscribe to camera info
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/oakd/rgb/preview/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Create synchronized subscribers for RGB and Depth images
        # Using uncompressed image since OAK-D publishes on /image_raw not /compressed
        self.rgb_sub = message_filters.Subscriber(
            self,
            Image,
            '/oakd/rgb/preview/image_raw'
        )
        
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            '/oakd/rgb/preview/depth'
        )
        
        # Synchronize the messages
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synchronized_callback)
        
        # Publisher for detected object position in map frame
        self.object_point_pub = self.create_publisher(
            Point,
            '/detected_object_point',
            10
        )
        
        # Publisher for binary mask visualization
        self.mask_pub = self.create_publisher(
            Image,
            '/object_detector/mask',
            10
        )
        
        # Publisher for visualization (optional - debug image)
        self.debug_image_pub = self.create_publisher(
            CompressedImage,
            '/object_detector/debug_image/compressed',
            10
        )
        
        self.get_logger().info('Object Detector initialized')
        self.get_logger().info(f'HSV Lower: {self.hsv_lower}')
        self.get_logger().info(f'HSV Upper: {self.hsv_upper}')
        self.get_logger().info(f'Target frame: {self.target_frame}')
    
    def camera_info_callback(self, msg):
        """Process camera info to extract intrinsic parameters"""
        if self.camera_matrix is None:
            # Extract camera matrix (K)
            K = np.array(msg.k).reshape(3, 3)
            self.camera_matrix = K
            self.camera_frame = msg.header.frame_id
            
            self.get_logger().info(f'Camera matrix received from frame: {self.camera_frame}')
            self.get_logger().info(f'fx: {K[0,0]:.2f}, fy: {K[1,1]:.2f}')
            self.get_logger().info(f'cx: {K[0,2]:.2f}, cy: {K[1,2]:.2f}')
    
    def detect_yellow_object(self, rgb_image):
        """
        Detect yellow object using HSV color space and connected components analysis
        
        Method:
        1. Convert RGB to HSV color space
        2. Apply HSV threshold to create binary mask
        3. Apply morphological operations to reduce noise
        4. Find connected components (contours)
        5. Select largest component
        6. Calculate centroid using image moments
        
        Returns: centroid (x, y) or None if no object detected
        """
        # Step 1: Convert BGR to HSV color space
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        
        # Step 2: Create binary mask for yellow color using HSV threshold
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        
        # Step 3: Morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Close small holes
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # Remove small noise
        
        # Step 4: Find connected components (contours) in binary image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, mask
        
        # Step 5: Select the largest connected component
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Filter out components that are too small
        if area < self.min_area:
            self.get_logger().debug(f'Contour area {area} too small (min: {self.min_area})')
            return None, mask
        
        # Step 6: Calculate centroid using image moments
        M = cv2.moments(largest_contour)
        
        if M['m00'] == 0:
            return None, mask
        
        # Centroid formula: cx = M10/M00, cy = M01/M00
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        
        # Log the contour area
        self.get_logger().info(f'Largest yellow contour area: {area:.1f} pixels')
        
        return (cx, cy), mask
    
    def project_to_3d(self, u, v, depth_image):
        """
        Project 2D pixel coordinates to 3D camera coordinates
        u, v: pixel coordinates
        depth_image: depth image in meters
        Returns: (x, y, z) in camera frame or None
        """
        if self.camera_matrix is None:
            self.get_logger().warn('Camera matrix not yet received')
            return None
        
        # Get depth value at pixel location
        h, w = depth_image.shape[:2]
        
        if u < 0 or u >= w or v < 0 or v >= h:
            self.get_logger().warn(f'Pixel coordinates out of bounds: ({u}, {v})')
            return None
        
        # Get depth value at pixel
        depth_raw = depth_image[v, u]
        self.get_logger().info(f'Raw depth value at ({u}, {v}): {depth_raw}')
        
        # OAK-D depth is typically in millimeters, convert to meters
        Z = depth_raw
        
        if Z == 0 or np.isnan(Z) or np.isinf(Z):
            self.get_logger().warn(f'Invalid depth value at ({u}, {v}): raw={depth_raw}, Z={Z}m')
            return None
        
        self.get_logger().info(f'Depth Z: {Z:.3f}m')
        
        # Extract camera intrinsics
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Project to 3D
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        
        return (X, Y, Z)
    
    def transform_to_map(self, point_camera, timestamp):
        """
        Transform point from camera frame to map frame using TF
        point_camera: (x, y, z) tuple in camera frame
        timestamp: time of the measurement
        Returns: Point in map frame or None
        """
        if self.camera_frame is None:
            self.get_logger().warn('Camera frame not yet known')
            return None
        
        try:
            # Create PointStamped in camera frame
            point_stamped = PointStamped()
            point_stamped.header.frame_id = self.camera_frame
            point_stamped.header.stamp = timestamp
            point_stamped.point.x = point_camera[0]
            point_stamped.point.y = point_camera[1]
            point_stamped.point.z = point_camera[2]
            
            # Get transform from camera frame to target frame
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                timestamp,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Transform point
            point_transformed = do_transform_point(point_stamped, transform)
            
            return point_transformed.point
            
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF transform failed: {str(e)}')
            return None
    
    def synchronized_callback(self, rgb_msg, depth_msg):
        """Process synchronized RGB and depth images"""
        try:
            # Convert uncompressed RGB image to OpenCV format
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            
            if rgb_image is None:
                self.get_logger().error('Failed to decode RGB image')
                return
            
            # Convert depth image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            
            # Detect yellow object
            centroid, mask = self.detect_yellow_object(rgb_image)
            
            # Publish mask for visualization (always publish, even if no object detected)
            if mask is not None:
                self.publish_mask(mask, rgb_msg.header.stamp)
            
            if centroid is None:
                return
            
            u, v = centroid
            self.get_logger().info(f'Yellow object detected at pixel: ({u}, {v})')
            
            # Project to 3D in camera frame
            point_3d_camera = self.project_to_3d(u, v, depth_image)
            
            if point_3d_camera is None:
                self.get_logger().warn('Failed to project to 3D - check depth value')
                return
            
            self.get_logger().info(f'3D point in camera frame: '
                                   f'X={point_3d_camera[0]:.3f}, '
                                   f'Y={point_3d_camera[1]:.3f}, '
                                   f'Z={point_3d_camera[2]:.3f}')
            
            # Transform to map frame
            point_map = self.transform_to_map(point_3d_camera, rgb_msg.header.stamp)
            
            if point_map is None:
                self.get_logger().warn('Failed to transform to map frame - check TF')
                return
            
            self.get_logger().info(f'3D point in {self.target_frame} frame: '
                                   f'X={point_map.x:.3f}, '
                                   f'Y={point_map.y:.3f}, '
                                   f'Z={point_map.z:.3f}')
            
            # Publish the point
            self.object_point_pub.publish(point_map)
            
            # Optional: Publish debug image with detection visualization
            self.publish_debug_image(rgb_image, centroid, mask)
            
        except Exception as e:
            self.get_logger().error(f'Error in synchronized callback: {str(e)}')
    
    def publish_debug_image(self, rgb_image, centroid, mask):
        """Publish debug image showing detection"""
        try:
            # Create debug image
            debug_img = rgb_image.copy()
            
            # Draw centroid
            cv2.circle(debug_img, centroid, 10, (0, 0, 255), -1)
            cv2.circle(debug_img, centroid, 20, (0, 255, 0), 2)
            
            # Add text
            text = f'Object at ({centroid[0]}, {centroid[1]})'
            cv2.putText(debug_img, text, (centroid[0] + 25, centroid[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Overlay mask
            mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            debug_img = cv2.addWeighted(debug_img, 0.7, mask_colored, 0.3, 0)
            
            # Convert to compressed image
            _, buffer = cv2.imencode('.jpg', debug_img)
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.header.frame_id = 'camera'
            compressed_msg.format = 'jpeg'
            compressed_msg.data = buffer.tobytes()
            
            self.debug_image_pub.publish(compressed_msg)
            
        except Exception as e:
            self.get_logger().debug(f'Error publishing debug image: {str(e)}')
    
    def publish_mask(self, mask, timestamp):
        """
        Publish binary mask as visible image.
        Binary mask (0 or 255) from cv2.inRange is published as mono8 image.
        White (255) = detected object, Black (0) = background
        """
        try:
            if mask is None:
                return
            
            # Convert to ROS Image message
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_msg.header.stamp = timestamp
            mask_msg.header.frame_id = self.camera_frame if self.camera_frame else 'camera'
            
            # Publish
            self.mask_pub.publish(mask_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing mask: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
