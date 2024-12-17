# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2
# from visualization_msgs.msg import Marker
# from sensor_msgs_py.point_cloud2 import read_points
# import numpy as np
# import math


# class PointCloudCenterFinder(Node):
#     def __init__(self):
#         super().__init__('point_cloud_centroid_calculator')

#         # Subscribe to the point cloud topic
#         self.subscription = self.create_subscription(
#             PointCloud2,
#             '/camera/camera/depth/color/points',  # Replace with your topic name
#             self.point_cloud_callback,
#             10
#         )

#         # Create a publisher for the marker to visualize in RViz
#         self.marker_publisher = self.create_publisher(Marker, '/centroid_marker', 10)

#         # Timer to compute and log the centroid periodically
#         self.timer = self.create_timer(5.0, self.compute_global_centroid)

#         # Buffer to store all points
#         self.all_points = []

#         # Max number of points to store
#         self.MAX_POINTS = 1_000_000

#     def point_cloud_callback(self, msg):
#         if not msg.data:
#             self.get_logger().warn("Received empty point cloud message.")
#             return

#         try:
#             # Extract points from the incoming point cloud message
#             points = list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

#             # Append the points to the global buffer
#             self.all_points.extend(points)
#             self.get_logger().info(f"Added {len(points)} points. Total: {len(self.all_points)} points.")

#             # Trim the buffer if it exceeds the max limit
#             if len(self.all_points) > self.MAX_POINTS:
#                 self.all_points = self.all_points[-self.MAX_POINTS:]
#                 self.get_logger().info(f"Trimmed points buffer to {self.MAX_POINTS} points.")

#         except Exception as e:
#             self.get_logger().error(f"Error processing point cloud: {e}")

#     def compute_global_centroid(self):
#         if len(self.all_points) == 0:
#             self.get_logger().warn("No points accumulated to compute the centroid.")
#             return None

#         try:
#             # Convert to numpy array
#             points_array = np.array(self.all_points)

#             # Unpack x, y, z values
#             x, y, z = zip(*points_array)
#             x = np.array(x, dtype=float)
#             y = np.array(y, dtype=float)
#             z = np.array(z, dtype=float)

#             # Calculate global centroid
#             centroid_x = np.mean(x)
#             centroid_y = np.mean(y)
#             centroid_z = np.mean(z)

#             centroid = (float(centroid_x), float(centroid_y), float(centroid_z))
#             self.get_logger().info(f"Global Centroid: x={centroid[0]:.3f}, y={centroid[1]:.3f}, z={centroid[2]:.3f}")

#             # Publish the centroid as a marker
#             self.publish_marker(centroid)

#             # Exit after computing the centroid
#             rclpy.shutdown()

#             return centroid

#         except Exception as e:
#             self.get_logger().error(f"Error computing global centroid: {e}")
#             return None

#     def clear_points(self):
#         """Clear the buffer of accumulated points."""
#         self.all_points = []
#         self.get_logger().info("Cleared all accumulated points.")

#     def publish_marker(self, centroid):
#         """Publish a visualization marker in RViz for the centroid."""
#         marker = Marker()
#         marker.header.frame_id = "camera_link"  # Replace with your camera's frame ID
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.type = Marker.SPHERE
#         marker.action = Marker.ADD
#         marker.pose.position.x = centroid[0]
#         marker.pose.position.y = centroid[1]
#         marker.pose.position.z = centroid[2]
#         marker.scale.x = 0.1  # Adjust size as needed
#         marker.scale.y = 0.1
#         marker.scale.z = 0.1
#         marker.color.a = 1.0
#         marker.color.r = 1.0
#         marker.color.g = 0.0
#         marker.color.b = 0.0

#         self.marker_publisher.publish(marker)


# def main(args=None):
#     rclpy.init(args=args)

#     try:
#         node = PointCloudCenterFinder()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()



























# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import PointStamped
# from tf2_ros import Buffer, TransformListener
# from sensor_msgs_py.point_cloud2 import read_points
# import numpy as np
# import math


# class PointCloudCenterFinder(Node):
#     def __init__(self):
#         super().__init__('point_cloud_centroid_calculator')

#         # Subscribe to the point cloud topic
#         self.subscription = self.create_subscription(
#             PointCloud2,
#             '/camera/camera/depth/color/points',  # Replace with your topic name
#             self.point_cloud_callback,
#             10
#         )

#         # Create a publisher for the marker to visualize in RViz
#         self.marker_publisher = self.create_publisher(Marker, '/centroid_marker', 10)

#         # Timer to compute and log the centroid periodically
#         self.timer = self.create_timer(5.0, self.compute_global_centroid)

#         # TF2 buffer and listener for transformations
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         # Buffer to store all points
#         self.all_points = []

#         # Max number of points to store
#         self.MAX_POINTS = 1_000_000

#     def point_cloud_callback(self, msg):
#         if not msg.data:
#             self.get_logger().warn("Received empty point cloud message.")
#             return

#         try:
#             # Extract points from the incoming point cloud message
#             points = list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

#             # Append the points to the global buffer
#             self.all_points.extend(points)
#             self.get_logger().info(f"Added {len(points)} points. Total: {len(self.all_points)} points.")

#             # Trim the buffer if it exceeds the max limit
#             if len(self.all_points) > self.MAX_POINTS:
#                 self.all_points = self.all_points[-self.MAX_POINTS:]
#                 self.get_logger().info(f"Trimmed points buffer to {self.MAX_POINTS} points.")

#         except Exception as e:
#             self.get_logger().error(f"Error processing point cloud: {e}")

#     def compute_global_centroid(self):
#         if len(self.all_points) == 0:
#             self.get_logger().warn("No points accumulated to compute the centroid.")
#             return None

#         try:
#             # Convert to numpy array
#             points_array = np.array(self.all_points)

#             # Unpack x, y, z values
#             x, y, z = zip(*points_array)
#             x = np.array(x, dtype=float)
#             y = np.array(y, dtype=float)
#             z = np.array(z, dtype=float)

#             # Calculate global centroid
#             centroid_x = np.mean(x)
#             centroid_y = np.mean(y)
#             centroid_z = np.mean(z)

#             centroid = (float(centroid_x), float(centroid_y), float(centroid_z))
#             self.get_logger().info(f"Global Centroid: x={centroid[0]:.3f}, y={centroid[1]:.3f}, z={centroid[2]:.3f}")

#             # Publish the centroid as a marker
#             self.publish_marker(centroid)

#             # Calculate and log the distance to the camera
#             self.calculate_distance_to_camera(centroid)

#             # Exit after computing the centroid
#             rclpy.shutdown()

#             return centroid

#         except Exception as e:
#             self.get_logger().error(f"Error computing global centroid: {e}")
#             return None

#     def calculate_distance_to_camera(self, centroid):
#         """Calculate the Euclidean distance between the camera and the centroid."""
#         try:
#             # Directly calculate the distance (centroid is already in camera's frame)
#             distance = math.sqrt(
#                 centroid[0] ** 2 +
#                 centroid[1] ** 2 +
#                 centroid[2] ** 2
#             )
#             self.get_logger().info(f"Distance to camera: {distance:.3f} meters")

#         except Exception as e:
#             self.get_logger().warn(f"Could not calculate distance: {e}")

#     def publish_marker(self, centroid):
#         """Publish a visualization marker in RViz for the centroid."""
#         marker = Marker()
#         marker.header.frame_id = "camera_link"  # Replace with your camera's frame ID
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.type = Marker.SPHERE
#         marker.action = Marker.ADD
#         marker.pose.position.x = centroid[0]
#         marker.pose.position.y = centroid[1]
#         marker.pose.position.z = centroid[2]
#         marker.scale.x = 0.9  # Adjust size as needed
#         marker.scale.y = 0.9
#         marker.scale.z = 0.9
#         marker.color.a = 1.0
#         marker.color.r = 1.0
#         marker.color.g = 0.0
#         marker.color.b = 0.0

#         self.marker_publisher.publish(marker)
#         self.get_logger().info("Centroid marker published to RViz.")


# def main(args=None):
#     rclpy.init(args=args)

#     try:
#         node = PointCloudCenterFinder()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()





























































import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from sensor_msgs_py.point_cloud2 import read_points
import numpy as np
import math


class PointCloudCenterFinder(Node):
    def __init__(self):
        super().__init__('point_cloud_centroid_calculator')

        # Subscribe to the point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',  # Replace with your topic name
            self.point_cloud_callback,
            10
        )

        # Create a publisher for the marker to visualize in RViz
        self.marker_publisher = self.create_publisher(Marker, '/centroid_marker', 10)

        # Buffer to store all points
        self.all_points = []

        # Max number of points to store
        self.MAX_POINTS = 1_000_000

    def point_cloud_callback(self, msg):
        if not msg.data:
            self.get_logger().warn("Received empty point cloud message.")
            return

        try:
            # Extract points from the incoming point cloud message
            points = list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

            # Ensure points are valid by converting to floats and filtering NaN/inf
            points = [(float(x), float(y), float(z)) for x, y, z in points if np.isfinite(x) and np.isfinite(y) and np.isfinite(z)]

            # Append the points to the global buffer
            self.all_points.extend(points)
            self.get_logger().info(f"Added {len(points)} points. Total: {len(self.all_points)} points.")

            # Trim the buffer if it exceeds the max limit
            if len(self.all_points) > self.MAX_POINTS:
                self.all_points = self.all_points[-self.MAX_POINTS:]
                self.get_logger().info(f"Trimmed points buffer to {self.MAX_POINTS} points.")

            # After processing a point cloud, calculate centroid and stop
            centroid = self.compute_visible_centroid()
            if centroid:
                self.get_logger().info(f"Final Centroid: x={centroid[0]:.3f}, y={centroid[1]:.3f}, z={centroid[2]:.3f}")
                self.calculate_distance_to_camera(centroid)
                # Shutdown after computation
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    def compute_visible_centroid(self):
        # Ensure we have valid points
        if len(self.all_points) == 0:
            self.get_logger().warn("No points accumulated to compute the centroid.")
            return None

        try:
            # Convert valid points to numpy array (already filtered to ensure they're valid)
            points_array = np.array(self.all_points)

            # Unpack x, y, z values
            x, y, z = zip(*points_array)
            x = np.array(x, dtype=float)
            y = np.array(y, dtype=float)
            z = np.array(z, dtype=float)

            # Calculate visible centroid
            centroid_x = np.mean(x)
            centroid_y = np.mean(y)
            centroid_z = np.mean(z)

            centroid = (float(centroid_x), float(centroid_y), float(centroid_z))
            self.get_logger().info(f"Visible Centroid: x={centroid[0]:.3f}, y={centroid[1]:.3f}, z={centroid[2]:.3f}")

            # Publish the centroid as a marker
            self.publish_marker(centroid)
            self.get_logger().info(f"Returning centroid: x={centroid[0]}, y={centroid[1]}, z={centroid[2]}")

            return centroid

        except Exception as e:
            self.get_logger().error(f"Error computing visible centroid: {e}")
            return None

    def publish_marker(self, centroid):
        """Publish a visualization marker in RViz for the centroid."""
        marker = Marker()
        marker.header.frame_id = "camera_depth_optical_frame"  # Use the correct frame ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = centroid[0]
        marker.pose.position.y = centroid[1]
        marker.pose.position.z = centroid[2]
        marker.scale.x = 0.1  # Adjust size as needed
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.marker_publisher.publish(marker)
        self.get_logger().info("Centroid marker published to RViz.")

    def calculate_distance_to_camera(self, centroid):
     """Calculate the Euclidean distance between the camera and the centroid."""
     try:
         # Log the centroid before computing the distance
         self.get_logger().info(f"Centroid received for distance calculation: x={centroid[0]}, y={centroid[1]}, z={centroid[2]}")

         # The camera's frame is at the origin, so distance is simply the Euclidean distance
         distance = math.sqrt(
             centroid[0] ** 2 + centroid[1] ** 2 + centroid[2] ** 2
         )

         self.get_logger().info(f"Distance between camera and centroid: {distance:.3f} meters")

     except Exception as e:
         self.get_logger().warn(f"Could not calculate distance: {e}")

def main(args=None):
    rclpy.init(args=args)

    try:
        node = PointCloudCenterFinder()
        # Do not call rclpy.spin here, we are manually shutting down after centroid calculation
        rclpy.spin_once(node)  # Only process the first point cloud message and then shutdown
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
