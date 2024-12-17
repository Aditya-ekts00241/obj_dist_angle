
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PointStamped
# import math

# class DistanceCalculatorNode(Node):
#     def __init__(self):
#         super().__init__('distance_calculator')
#         self.subscription = self.create_subscription(
#             PointStamped,
#             '/clicked_point',
#             self.clicked_point_callback,
#             10)
#         self.get_logger().info("Distance Calculator Node Started. Click a point in RViz2.")

#     def clicked_point_callback(self, msg):
#         # Extract coordinates of clicked point
#         x, y, z = msg.point.x, msg.point.y, msg.point.z
#         self.get_logger().info(f"Clicked Point Coordinates: (x: {x:.2f}, y: {y:.2f}, z: {z:.2f})")
#         # Calculate distance from origin (camera center)
#         distance = math.sqrt(x**2 + y**2 + z**2)
#         self.get_logger().info(f"Distance from camera to selected point: {distance:.2f} meters")

# def main(args=None):
#     rclpy.init(args=args)
#     distance_calculator_node = DistanceCalculatorNode()
#     rclpy.spin(distance_calculator_node)
#     distance_calculator_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




































import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import math
import subprocess

class DistanceCalculatorNode(Node):
    def __init__(self):
        super().__init__('distance_calculator')
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10)
        self.get_logger().info("Distance Calculator Node Started. Click a point in RViz2.")

    def clicked_point_callback(self, msg):
        # Extract coordinates of clicked point
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        self.get_logger().info(f"Clicked Point Coordinates: (x: {x:.2f}, y: {y:.2f}, z: {z:.2f})")
        
        # Calculate distance from origin (camera center)
        distance = math.sqrt(x**2 + y**2 + z**2)
        self.get_logger().info(f"Distance from camera to selected point: {distance:.2f} meters")
        
        # Define the camera frame and clicked point frame IDs
        camera_frame = 'camera_link'  # Replace with your actual camera frame ID
        clicked_point_frame = 'clicked_point_frame'
        
        # Use subprocess to call the static_transform_publisher with the clicked point coordinates
        cmd = [
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            str(x), str(y), str(z), '0', '0', '0', '1',  # Position (x, y, z) and quaternion (0,0,0,1) for identity rotation
            camera_frame,
            clicked_point_frame
        ]
        subprocess.Popen(cmd)
        self.get_logger().info(f"Published static transform from {camera_frame} to {clicked_point_frame}")

def main(args=None):
    rclpy.init(args=args)
    distance_calculator_node = DistanceCalculatorNode()
    rclpy.spin(distance_calculator_node)
    distance_calculator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
