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
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        
        # Camera intrinsics (will be updated from camera_info)
        self.cameraMatrix = None
        self.cameraFrame = None
        
        # Declare parameters for HSV thresholds (for yellow color detection)
        self.declare_parameter('hsv_lower', [20, 100, 100])
        self.declare_parameter('hsv_upper', [30, 255, 255])
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('min_area', 500.0)  # Minimum contour area to consider
        
        # Get parameters
        hsvLower = self.get_parameter('hsv_lower').value
        hsvUpper = self.get_parameter('hsv_upper').value
        self.targetFrame = self.get_parameter('target_frame').value
        self.minArea = self.get_parameter('min_area').value
        
        self.hsvLower = np.array(hsvLower)
        self.hsvUpper = np.array(hsvUpper)
        
        # Subscribe to camera info
        self.cameraInfoSub = self.create_subscription(
            CameraInfo,
            '/oakd/rgb/preview/camera_info',
            self.CameraInfoCallback,
            10
        )
        
        # Create synchronized subscribers for RGB and Depth images
        # Using uncompressed image since OAK-D publishes on /image_raw not /compressed
        self.rgbSub = message_filters.Subscriber(
            self,
            Image,
            '/oakd/rgb/preview/image_raw'
        )
        
        self.depthSub = message_filters.Subscriber(
            self,
            Image,
            '/oakd/rgb/preview/depth'
        )
        
        # Synchronize the messages
        self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.rgbSub, self.depthSub],
            queue_size=10,
            slop=0.1
        )
        self.timeSynchronizer.registerCallback(self.SynchronizedCallback)
        
        # Publisher for detected object position in map frame
        self.objectPointPub = self.create_publisher(
            Point,
            '/detected_object_point',
            10
        )
        
        # Publisher for binary mask visualization
        self.maskPub = self.create_publisher(
            Image,
            '/object_detector/mask',
            10
        )
        
        # Publisher for visualization (optional - debug image)
        self.debugImagePub = self.create_publisher(
            CompressedImage,
            '/object_detector/debug_image/compressed',
            10
        )
        
        self.get_logger().info('Object Detector initialized')
        self.get_logger().info(f'HSV Lower: {self.hsvLower}')
        self.get_logger().info(f'HSV Upper: {self.hsvUpper}')
        self.get_logger().info(f'Target frame: {self.targetFrame}')
    
    def CameraInfoCallback(self, msg):
        """Process camera info to extract intrinsic parameters"""
        if self.cameraMatrix is None:
            # Extract camera matrix (K)
            cameraMatrix = np.array(msg.k).reshape(3, 3)
            self.cameraMatrix = cameraMatrix
            self.cameraFrame = msg.header.frame_id
            
            self.get_logger().info(f'Camera matrix received from frame: {self.cameraFrame}')
            self.get_logger().info(f'fx: {cameraMatrix[0,0]:.2f}, fy: {cameraMatrix[1,1]:.2f}')
            self.get_logger().info(f'cx: {cameraMatrix[0,2]:.2f}, cy: {cameraMatrix[1,2]:.2f}')
    
    def DetectYellowObject(self, rgbImage):
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
        hsv = cv2.cvtColor(rgbImage, cv2.COLOR_BGR2HSV)
        
        # Step 2: Create binary mask for yellow color using HSV threshold
        mask = cv2.inRange(hsv, self.hsvLower, self.hsvUpper)
        
        # Step 3: Morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Close small holes
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # Remove small noise
        
        # Step 4: Find connected components (contours) in binary image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, mask
        
        # Step 5: Select the largest connected component
        largestContour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largestContour)
        
        # Filter out components that are too small
        if area < self.minArea:
            self.get_logger().debug(f'Contour area {area} too small (min: {self.minArea})')
            return None, mask
        
        # Step 6: Calculate centroid using image moments
        moments = cv2.moments(largestContour)
        
        if moments['m00'] == 0:
            return None, mask
        
        # Centroid formula: cx = M10/M00, cy = M01/M00
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        
        # Log the contour area
        self.get_logger().info(f'Largest yellow contour area: {area:.1f} pixels')
        
        return (cx, cy), mask
    
    def ProjectTo3D(self, u, v, depthImage):
        """
        Project 2D pixel coordinates to 3D camera coordinates
        u, v: pixel coordinates
        depthImage: depth image in meters
        Returns: (x, y, z) in camera frame or None
        """
        if self.cameraMatrix is None:
            self.get_logger().warn('Camera matrix not yet received')
            return None
        
        # Get depth value at pixel location
        height, width = depthImage.shape[:2]
        
        if u < 0 or u >= width or v < 0 or v >= height:
            self.get_logger().warn(f'Pixel coordinates out of bounds: ({u}, {v})')
            return None
        
        # Get depth value at pixel
        depthRaw = depthImage[v, u]
        self.get_logger().info(f'Raw depth value at ({u}, {v}): {depthRaw}')
        
        # OAK-D depth is typically in millimeters, convert to meters
        z = depthRaw
        
        if z == 0 or np.isnan(z) or np.isinf(z):
            self.get_logger().warn(f'Invalid depth value at ({u}, {v}): raw={depthRaw}, Z={z}m')
            return None
        
        self.get_logger().info(f'Depth Z: {z:.3f}m')
        
        # Extract camera intrinsics
        fx = self.cameraMatrix[0, 0]
        fy = self.cameraMatrix[1, 1]
        cx = self.cameraMatrix[0, 2]
        cy = self.cameraMatrix[1, 2]
        
        # Project to 3D
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        return (x, y, z)
    
    def TransformToMap(self, pointCamera, timestamp):
        """
        Transform point from camera frame to map frame using TF
        pointCamera: (x, y, z) tuple in camera frame
        timestamp: time of the measurement
        Returns: Point in map frame or None
        """
        if self.cameraFrame is None:
            self.get_logger().warn('Camera frame not yet known')
            return None
        
        try:
            # Create PointStamped in camera frame
            pointStamped = PointStamped()
            pointStamped.header.frame_id = self.cameraFrame
            pointStamped.header.stamp = timestamp
            pointStamped.point.x = pointCamera[0]
            pointStamped.point.y = pointCamera[1]
            pointStamped.point.z = pointCamera[2]
            
            # Get transform from camera frame to target frame
            transform = self.tfBuffer.lookup_transform(
                self.targetFrame,
                self.cameraFrame,
                timestamp,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Transform point
            pointTransformed = do_transform_point(pointStamped, transform)
            
            return pointTransformed.point
            
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as exception:
            self.get_logger().warn(f'TF transform failed: {str(exception)}')
            return None
    
    def SynchronizedCallback(self, rgbMsg, depthMsg):
        """Process synchronized RGB and depth images"""
        try:
            # Convert uncompressed RGB image to OpenCV format
            rgbImage = self.bridge.imgmsg_to_cv2(rgbMsg, desired_encoding='bgr8')
            
            if rgbImage is None:
                self.get_logger().error('Failed to decode RGB image')
                return
            
            # Convert depth image to OpenCV format
            depthImage = self.bridge.imgmsg_to_cv2(depthMsg, desired_encoding='passthrough')
            
            # Detect yellow object
            centroid, mask = self.DetectYellowObject(rgbImage)
            
            # Publish mask for visualization (always publish, even if no object detected)
            if mask is not None:
                self.PublishMask(mask, rgbMsg.header.stamp)
            
            if centroid is None:
                return
            
            u, v = centroid
            self.get_logger().info(f'Yellow object detected at pixel: ({u}, {v})')
            
            # Project to 3D in camera frame
            point3dCamera = self.ProjectTo3D(u, v, depthImage)
            
            if point3dCamera is None:
                self.get_logger().warn('Failed to project to 3D - check depth value')
                return
            
            self.get_logger().info(f'3D point in camera frame: '
                                   f'X={point3dCamera[0]:.3f}, '
                                   f'Y={point3dCamera[1]:.3f}, '
                                   f'Z={point3dCamera[2]:.3f}')
            
            # Transform to map frame
            pointMap = self.TransformToMap(point3dCamera, rgbMsg.header.stamp)
            
            if pointMap is None:
                self.get_logger().warn('Failed to transform to map frame - check TF')
                return
            
            self.get_logger().info(f'3D point in {self.targetFrame} frame: '
                                   f'X={pointMap.x:.3f}, '
                                   f'Y={pointMap.y:.3f}, '
                                   f'Z={pointMap.z:.3f}')
            
            # Publish the point
            self.objectPointPub.publish(pointMap)
            
            # Optional: Publish debug image with detection visualization
            self.PublishDebugImage(rgbImage, centroid, mask)
            
        except Exception as exception:
            self.get_logger().error(f'Error in synchronized callback: {str(exception)}')
    
    def PublishDebugImage(self, rgbImage, centroid, mask):
        """Publish debug image showing detection"""
        try:
            # Create debug image
            debugImg = rgbImage.copy()
            
            # Draw centroid
            cv2.circle(debugImg, centroid, 10, (0, 0, 255), -1)
            cv2.circle(debugImg, centroid, 20, (0, 255, 0), 2)
            
            # Add text
            text = f'Object at ({centroid[0]}, {centroid[1]})'
            cv2.putText(debugImg, text, (centroid[0] + 25, centroid[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Overlay mask
            maskColored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            debugImg = cv2.addWeighted(debugImg, 0.7, maskColored, 0.3, 0)
            
            # Convert to compressed image
            _, buffer = cv2.imencode('.jpg', debugImg)
            compressedMsg = CompressedImage()
            compressedMsg.header.stamp = self.get_clock().now().to_msg()
            compressedMsg.header.frame_id = 'camera'
            compressedMsg.format = 'jpeg'
            compressedMsg.data = buffer.tobytes()
            
            self.debugImagePub.publish(compressedMsg)
            
        except Exception as exception:
            self.get_logger().debug(f'Error publishing debug image: {str(exception)}')
    
    def PublishMask(self, mask, timestamp):
        """
        Publish binary mask as visible image.
        Binary mask (0 or 255) from cv2.inRange is published as mono8 image.
        White (255) = detected object, Black (0) = background
        """
        try:
            if mask is None:
                return
            
            # Convert to ROS Image message
            maskMsg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            maskMsg.header.stamp = timestamp
            maskMsg.header.frame_id = self.cameraFrame if self.cameraFrame else 'camera'
            
            # Publish
            self.maskPub.publish(maskMsg)
            
        except Exception as exception:
            self.get_logger().error(f'Error publishing mask: {str(exception)}')


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