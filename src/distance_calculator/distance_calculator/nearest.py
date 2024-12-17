# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs_py.point_cloud2 import read_points

# import numpy as np
# from geometry_msgs.msg import PointStamped
# import tf2_ros
# import tf2_geometry_msgs
# from std_msgs.msg import Header

# class NearestPointFinder(Node):
#     def __init__(self):
#         super().__init__('nearest_point_finder')
        
#         # Subscriber to the PointCloud2 message
#         self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.point_cloud_callback, 10)

#         # Publisher to mark the nearest point
#         self.nearest_point_pub = self.create_publisher(PointStamped, '/nearest_point', 10)

#         # TF Buffer and Listener
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#     def point_cloud_callback(self, msg):
#         # Convert the PointCloud2 message to a list of 3D points
#         pc_data = read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

#         # Initialize variables to store nearest point data
#         nearest_point = None
#         min_distance = float('inf')

#         # Iterate through all points in the cloud
#         for point in pc_data:
#             x, y, z = point
            
#             # Filter based on camera's straight direction (Z axis)
#             if abs(x) < 0.1 and abs(y) < 0.1:  # Close to straight in front
#                 # Check for the minimum distance in the Z direction
#                 if z > 0 and z < min_distance:
#                     min_distance = z
#                     nearest_point = point
        
#         if nearest_point is not None:
#             # Ensure the point values are floats before assigning
#             x, y, z = float(nearest_point[0]), float(nearest_point[1]), float(nearest_point[2])

#             # Create a PointStamped message for the nearest point
#             nearest_point_msg = PointStamped()
#             nearest_point_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='camera_link')
#             nearest_point_msg.point.x = x
#             nearest_point_msg.point.y = y
#             nearest_point_msg.point.z = z

#             # Publish the nearest point
#             self.nearest_point_pub.publish(nearest_point_msg)

#             # Transform the point to the camera frame if necessary
#             try:
#                 transform = self.tf_buffer.lookup_transform('camera_link', nearest_point_msg.header.frame_id, rclpy.time.Time())
#                 transformed_point = tf2_geometry_msgs.do_transform_point(nearest_point_msg, transform)
#                 distance = np.sqrt(transformed_point.point.x**2 + transformed_point.point.y**2 + transformed_point.point.z**2)
#                 self.get_logger().info(f"Nearest point: {transformed_point.point} with distance: {distance}")
#             except (tf2_ros.TransformException) as e:
#                 self.get_logger().error(f"Transform failed: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = NearestPointFinder()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


































# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs_py.point_cloud2 import read_points
# import numpy as np
# from geometry_msgs.msg import PointStamped
# import tf2_ros
# import tf2_geometry_msgs
# from std_msgs.msg import Header

# class NearestPointFinder(Node):
#     def __init__(self):
#         super().__init__('nearest_point_finder')
        
#         # Subscriber to the PointCloud2 message
#         self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.point_cloud_callback, 10)

#         # Publisher to mark the nearest point
#         self.nearest_point_pub = self.create_publisher(PointStamped, '/nearest_point', 10)

#         # TF Buffer and Listener
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         # Flag to indicate that the first point has been found and processed
#         self.first_point_found = False

#     def point_cloud_callback(self, msg):
#         if self.first_point_found:
#             return  # Stop processing after the first point is found
        
#         # Convert the PointCloud2 message to a list of 3D points
#         pc_data = read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

#         # Initialize variables to store nearest point data
#         nearest_point = None
#         min_distance = float('inf')

#         # Iterate through all points in the cloud
#         for point in pc_data:
#             x, y, z = point
            
#             # Filter based on camera's straight direction (Z axis)
#             if abs(x) < 0.1 and abs(y) < 0.1:  # Close to straight in front
#                 # Check for the minimum distance in the Z direction
#                 if z > 0 and z < min_distance:
#                     min_distance = z
#                     nearest_point = point
        
#         if nearest_point is not None:
#             # Ensure the point values are floats before assigning
#             x, y, z = float(nearest_point[0]), float(nearest_point[1]), float(nearest_point[2])

#             # Log the nearest point and its distance
#             self.get_logger().info(f"First nearest point: geometry_msgs.msg.Point(x={x}, y={y}, z={z}) with distance: {min_distance}")
            
#             # Create a PointStamped message for the nearest point
#             nearest_point_msg = PointStamped()
#             nearest_point_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='camera_link')
#             nearest_point_msg.point.x = x
#             nearest_point_msg.point.y = y
#             nearest_point_msg.point.z = z

#             # Publish the nearest point for RViz visualization
#             self.nearest_point_pub.publish(nearest_point_msg)

#             # Transform the point to the camera frame if necessary
#             try:
#                 transform = self.tf_buffer.lookup_transform('camera_link', nearest_point_msg.header.frame_id, rclpy.time.Time())
#                 transformed_point = tf2_geometry_msgs.do_transform_point(nearest_point_msg, transform)
#                 distance = np.sqrt(transformed_point.point.x**2 + transformed_point.point.y**2 + transformed_point.point.z**2)
#                 self.get_logger().info(f"Nearest point in camera frame: {transformed_point.point} with distance: {distance}")
#             except (tf2_ros.TransformException) as e:
#                 self.get_logger().error(f"Transform failed: {e}")
            
#             # Set flag to stop further processing
#             self.first_point_found = True

# def main(args=None):
#     rclpy.init(args=args)
#     node = NearestPointFinder()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()














































import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs

class NearestPointFinder(Node):
    def __init__(self):
        super().__init__('nearest_point_finder')
        
        # Subscriber to the PointCloud2 message
        self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.point_cloud_callback, 10)

        # Publisher for the nearest point as PointStamped
        self.nearest_point_pub = self.create_publisher(PointStamped, '/nearest_point', 10)

        # Publisher for RViz visualization as a Marker
        self.nearest_point_marker_pub = self.create_publisher(Marker, '/nearest_point_marker', 10)

        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Flag to indicate that the first point has been found
        self.first_point_found = False

    # def point_cloud_callback(self, msg):
    #     self.get_logger().info("PointCloud2 message received.")

    #     # If the first point has been found, no further processing is needed
    #     if self.first_point_found:
    #         return

    #     # Convert the PointCloud2 message to a list of 3D points
    #     pc_data = read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

    #     # Initialize variables to store the nearest point data
    #     nearest_point = None
    #     min_distance = float('inf')

    #     # Iterate through all points in the cloud
    #     for point in pc_data:
    #         x, y, z = point
            
    #         # Filter based on camera's straight direction (Z axis)
    #         if abs(x) < 0.1 and abs(y) < 0.1:  # Close to straight in front
    #             # Check for the minimum distance in the Z direction
    #             if z > 0 and z < min_distance:
    #                 min_distance = z
    #                 nearest_point = point

    #     if nearest_point is not None:
    #         # Extract the nearest point coordinates
    #         x, y, z = float(nearest_point[0]), float(nearest_point[1]), float(nearest_point[2])

    #         # Log the nearest point and its distance
    #         self.get_logger().info(f"First nearest point: geometry_msgs.msg.Point(x={x}, y={y}, z={z}) with distance: {min_distance}")

    #         # Publish the nearest point as a PointStamped message
    #         nearest_point_msg = PointStamped()
    #         nearest_point_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='camera_link')
    #         nearest_point_msg.point.x = x
    #         nearest_point_msg.point.y = y
    #         nearest_point_msg.point.z = z
    #         self.nearest_point_pub.publish(nearest_point_msg)

    #         # Publish the nearest point as a Marker message for visualization in RViz
    #         marker_msg = Marker()
    #         marker_msg.header = nearest_point_msg.header
    #         marker_msg.ns = "nearest_point"
    #         marker_msg.id = 0
    #         marker_msg.type = Marker.SPHERE
    #         marker_msg.action = Marker.ADD
    #         marker_msg.pose.position.x = x
    #         marker_msg.pose.position.y = y
    #         marker_msg.pose.position.z = z
    #         marker_msg.pose.orientation.x = 0.0
    #         marker_msg.pose.orientation.y = 0.0
    #         marker_msg.pose.orientation.z = 0.0
    #         marker_msg.pose.orientation.w = 1.0
    #         marker_msg.scale.x = 0.05  # Adjust size of the sphere
    #         marker_msg.scale.y = 0.05
    #         marker_msg.scale.z = 0.05
    #         marker_msg.color.a = 1.0  # Alpha (transparency)
    #         marker_msg.color.r = 1.0  # Red
    #         marker_msg.color.g = 0.0  # Green
    #         marker_msg.color.b = 0.0  # Blue
    #         self.nearest_point_marker_pub.publish(marker_msg)

    #         # Stop further processing by setting the flag
    #         self.first_point_found = True


    def point_cloud_callback(self, msg):
        self.get_logger().info("PointCloud2 message received.")

        # If the first point has been found, no further processing is needed
        if self.first_point_found:
            return

        # Convert the PointCloud2 message to a list of 3D points
        pc_data = read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # Initialize variables for nearest point along the optical axis
        nearest_point = None
        min_distance = float('inf')

        # Iterate through all points in the cloud
        for point in pc_data:
            x, y, z = point
            
            # Strictly filter points near the optical axis (0 degrees)
            if abs(x) < 0.005 and abs(y) < 0.005:  # Tighter constraints for optical axis
                # Check for the nearest point in Z direction (distance from camera)
                if z > 0 and z < min_distance:
                    min_distance = z
                    nearest_point = point

        if nearest_point is not None:
            # Extract the nearest point coordinates
            x, y, z = float(nearest_point[0]), float(nearest_point[1]), float(nearest_point[2])

            # Log the nearest point and its distance
            self.get_logger().info(f"Nearest point at optical axis: geometry_msgs.msg.Point(x={x}, y={y}, z={z}) with distance: {min_distance}")

            # Publish the nearest point as a PointStamped message
            nearest_point_msg = PointStamped()
            nearest_point_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='camera_link')
            nearest_point_msg.point.x = x
            nearest_point_msg.point.y = y
            nearest_point_msg.point.z = z
            self.nearest_point_pub.publish(nearest_point_msg)

            # Publish the nearest point as a Marker message for visualization in RViz
            marker_msg = Marker()
            marker_msg.header = nearest_point_msg.header
            marker_msg.ns = "nearest_point"
            marker_msg.id = 0
            marker_msg.type = Marker.SPHERE
            marker_msg.action = Marker.ADD
            marker_msg.pose.position.x = x
            marker_msg.pose.position.y = y
            marker_msg.pose.position.z = z
            marker_msg.pose.orientation.x = 0.0
            marker_msg.pose.orientation.y = 0.0
            marker_msg.pose.orientation.z = 0.0
            marker_msg.pose.orientation.w = 1.0
            marker_msg.scale.x = 0.05  # Adjust size of the sphere
            marker_msg.scale.y = 0.05
            marker_msg.scale.z = 0.05
            marker_msg.color.a = 1.0  # Alpha (transparency)
            marker_msg.color.r = 0.0  # Red
            marker_msg.color.g = 1.0  # Green
            marker_msg.color.b = 0.0  # Blue
            self.nearest_point_marker_pub.publish(marker_msg)

            # Stop further processing by setting the flag
            self.first_point_found = True


def main(args=None):
    rclpy.init(args=args)
    node = NearestPointFinder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
