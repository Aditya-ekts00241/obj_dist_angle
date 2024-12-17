# rgb_click_to_depth.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
import numpy as np
import cv2
from cv_bridge import CvBridge
import struct
from distance_calculator.srv import Get3DPoint

class RGBClickToDepth(Node):
    def __init__(self):
        super().__init__('rgb_click_to_depth')
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_cloud = None

        # Subscribe to RGB and Depth PointCloud2 topics
        self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.create_subscription(PointCloud2, '/camera/depth/color/points', self.depth_callback, 10)

        # Service to take RGB pixel and return 3D depth point
        self.create_service(Get3DPoint, 'get_3d_point', self.handle_get_3d_point)

    def rgb_callback(self, msg):
        # Convert the incoming image message to OpenCV format
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        # Cache the latest depth cloud message
        self.depth_cloud = msg

    def handle_get_3d_point(self, request, response):
        # Extract pixel coordinates
        x, y = request.x, request.y
        if self.rgb_image is None or self.depth_cloud is None:
            self.get_logger().error("RGB or Depth data is not yet available.")
            return response

        # Find corresponding 3D point in the depth cloud
        try:
            point = self.get_depth_at_pixel(x, y)
            response.point.x = point[0]
            response.point.y = point[1]
            response.point.z = point[2]
        except Exception as e:
            self.get_logger().error(f"Error finding depth for pixel ({x}, {y}): {e}")
        return response

    def get_depth_at_pixel(self, x, y):
        # Extract depth cloud data at (x, y)
        assert self.depth_cloud, "Depth cloud is not available."
        data = np.frombuffer(self.depth_cloud.data, dtype=np.float32).reshape(-1, 4)
        index = y * self.depth_cloud.width + x
        return data[index][:3]  # Extract X, Y, Z coordinates

def main(args=None):
    rclpy.init(args=args)
    node = RGBClickToDepth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
