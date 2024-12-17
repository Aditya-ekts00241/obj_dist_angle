import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
from tf2_ros import TransformListener, Buffer
from scipy.spatial.transform import Rotation as R

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')

        # Initialize YOLO model
        self.model = YOLO('/home/kalyani-robotics/ros2_ws/src/distance_calculator/distance_calculator/best.pt')
        self.bridge = CvBridge()

        # Subscribe to image topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Subscribe to LiDAR topic
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # Subscribe to depth image topic
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',  
            self.depth_callback,
            10
        )
        self.depth_image = None

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera intrinsic parameters
        self.fx = 912.5653686523438  # Replace with your camera's fx
        self.fy = 912.4901733398438  # Replace with your camera's fy
        self.cx = 651.2647094726562  # Replace with your camera's cx
        self.cy = 377.9391784667969  # Replace with your camera's cy

        self.lidar_data = None
        self.get_logger().info('YOLO Detection Node Initialized.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform YOLO inference
            results = self.model(frame)
            boxes = results[0].boxes
            class_names = results[0].names

            if self.lidar_data:
                for box in boxes:
                    # Extract bounding box and class info
                    x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy()
                    class_id = int(box.cls[0].cpu().item())
                    class_name = class_names[class_id]

                    # Calculate center of bounding box
                    x_center = (x_min + x_max) / 2
                    y_center = (y_min + y_max) / 2

                    # Scale to depth image resolution
                    depth_x_center = x_center * (self.depth_image.shape[1] / frame.shape[1])
                    depth_y_center = y_center * (self.depth_image.shape[0] / frame.shape[0])

                    # Map pixel to 3D space (camera frame)
                    depth = self.get_depth_at_pixel(depth_x_center, depth_y_center)
                    if depth is None:
                        continue

                    # Compute 3D coordinates in camera frame
                    X = (depth_x_center - self.cx) * depth / self.fx
                    Y = (depth_y_center - self.cy) * depth / self.fy
                    Z = depth

                    # Transform to LiDAR/robot frame
                    lidar_point = self.transform_camera_to_lidar(X, Y, Z)
                    if lidar_point is None:
                        continue

                    # Calculate distance and angle
                    distance = np.linalg.norm(lidar_point)
                    # angle = np.arctan2(lidar_point[1], lidar_point[0])

                    # Log the results
                    self.get_logger().info(f'Class: {class_name}, Distance: {distance:.2f} m')

            # Display detections
            for box in boxes:
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0].cpu().numpy())
                class_id = int(box.cls[0].cpu().item())
                label = f'{class_names[class_id]}'
                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.putText(frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("YOLO Detection", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Error in image processing: {str(e)}')

    def lidar_callback(self, msg):
        # Filter LiDAR data (exclude "inf" values)
        self.lidar_data = [r for r in msg.ranges if r != float('inf')]
        
    def depth_callback(self, msg):
        try:
            # Convert ROS Image message to a NumPy array
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def get_depth_at_pixel(self, x, y):
        """Fetch depth value at a specific pixel."""
        if self.depth_image is None:
            self.get_logger().warning('Depth image not received yet.')
            return None

        try:
            # Ensure coordinates are within image bounds
            if 0 <= int(y) < self.depth_image.shape[0] and 0 <= int(x) < self.depth_image.shape[1]:
                depth_value = self.depth_image[int(y), int(x)]
                if np.isnan(depth_value) or depth_value <= 0:
                    self.get_logger().warning(f'Invalid depth value at pixel ({x}, {y}).')
                    return None
                return depth_value / 1000.0  # Convert from mm to meters if necessary
            else:
                self.get_logger().warning(f'Pixel coordinates ({x}, {y}) are out of bounds.')
                return None
        except Exception as e:
            self.get_logger().error(f'Error fetching depth at pixel ({x}, {y}): {str(e)}')
        return None
    
    def transform_camera_to_lidar(self, X, Y, Z):
        """Transform camera coordinates to LiDAR/robot frame."""
        try:
            trans = self.tf_buffer.lookup_transform('camera_color_optical_frame', 'camera_depth_optical_frame', rclpy.time.Time())
            # Convert the translation and rotation
            T = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            q = trans.transform.rotation
            rotation = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()  # Convert quaternion to rotation matrix
            point_camera = np.array([X, Y, Z])
            point_lidar = rotation @ point_camera + T
            return point_lidar
        except Exception as e:
            self.get_logger().error(f'Transformation failed: {str(e)}')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
