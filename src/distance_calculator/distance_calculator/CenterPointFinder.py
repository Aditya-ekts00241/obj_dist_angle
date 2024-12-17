# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, PointCloud2
# import numpy as np
# import sensor_msgs_py.point_cloud2 as pc2

# class CenterPointFinder(Node):
#     def __init__(self):
#         super().__init__('center_point_finder')

#         # Subscriptions
#         self.rgb_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
#         self.pc_sub = self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.pointcloud_callback, 10)
        
#         self.image_width = None
#         self.image_height = None
#         self.center_x = None
#         self.center_y = None
#         self.pointcloud_data = None

#     def image_callback(self, msg):
#         # Extract image dimensions from the RGB image
#         self.image_width = msg.width
#         self.image_height = msg.height
        
#         # Compute center pixel
#         self.center_x = int(self.image_width / 2)
#         self.center_y = int(self.image_height / 2)
        
#         self.get_logger().info(f"Center pixel at: ({self.center_x}, {self.center_y})")

#     def pointcloud_callback(self, msg):
#         # Store point cloud data
#         self.pointcloud_data = msg
        
#         # Ensure that we have the necessary image dimensions and point cloud data
#         if self.center_x is not None and self.center_y is not None and self.pointcloud_data is not None:
#             # Get the 3D coordinates at the center pixel
#             point = self.get_xyz_at_pixel(self.center_x, self.center_y, self.pointcloud_data)
#             if point:
#                 x, y, z = point
#                 self.get_logger().info(f"3D coordinates at image center: x={x}, y={y}, z={z}")

#     def get_xyz_at_pixel(self, u, v, pointcloud_msg):
#         # Retrieve the point at (u, v) in the point cloud
#         for point in pc2.read_points(pointcloud_msg, skip_nans=True, field_names=("x", "y", "z")):
#             # Calculate the index in the point cloud data corresponding to (u, v)
#             index = v * self.image_width + u
#             if index == point[0]: # Check if we are at the correct point in the array
#                 return (point[0], point[1], point[2])
#         return None

# def main(args=None):
#     rclpy.init(args=args)
#     node = CenterPointFinder()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




































































import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

class ClickPointTo3D(Node):
    def __init__(self):
        super().__init__('click_point_to_3d')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize camera intrinsics placeholders
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Initialize scaling factors and image dimensions
        self.width_rgb = None
        self.height_rgb = None
        self.width_depth = None
        self.height_depth = None
        self.scale_x = None
        self.scale_y = None

        # Set up subscribers with message filters for synchronization
        self.rgb_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        # Synchronize RGB and depth images
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synchronized_callback)

        # Flags and placeholders
        self.rgb_image = None
        self.depth_image = None

        # Set up OpenCV window
        cv2.namedWindow("RGB Image")
        cv2.setMouseCallback("RGB Image", self.on_mouse_click)

    def camera_info_callback(self, msg):
        # Extract camera intrinsic parameters
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(f"Camera intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")
        self.camera_info_sub.unsubscribe()  # Unsubscribe after receiving the info once

    def synchronized_callback(self, rgb_msg, depth_msg):
        # Convert images
        self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        # Get image dimensions if not set
        if self.width_rgb is None or self.height_rgb is None:
            self.height_rgb, self.width_rgb = self.rgb_image.shape[:2]
        if self.width_depth is None or self.height_depth is None:
            self.height_depth, self.width_depth = self.depth_image.shape[:2]

        # Calculate scaling factors between RGB and depth images
        self.scale_x = self.width_depth / self.width_rgb
        self.scale_y = self.height_depth / self.height_rgb
        # self.get_logger().info(f"Scaling factors: scale_x={self.scale_x}, scale_y={self.scale_y}")

        # Display the RGB image in a loop to handle refresh
        if self.rgb_image is not None:
            cv2.imshow("RGB Image", self.rgb_image)
            cv2.waitKey(1)

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info(f"Mouse clicked at: ({x}, {y})")

            # Ensure depth data is available
            if self.depth_image is None:
                self.get_logger().warn("No depth data available!")
                return

            # Apply scaling to find corresponding point in depth image
            depth_x = int(x * self.scale_x)
            depth_y = int(y * self.scale_y)

            # Check if the scaled coordinates are within bounds of the depth image
            if depth_x >= self.width_depth or depth_y >= self.height_depth:
                self.get_logger().warn("Scaled coordinates are out of depth image bounds.")
                return

            # Get depth at the scaled point
            depth = self.depth_image[depth_y, depth_x]
            self.get_logger().info(f"Depth at clicked point: {depth/1000} meters")

            # Calculate 3D coordinates if depth is valid
            if depth > 0:  # Check for a valid depth
                point_3d = self.calculate_3d_point(x, y, depth)
                if point_3d:
                    x_3d, y_3d, z_3d = point_3d
                    self.get_logger().info(f"3D coordinates at clicked point: x={x_3d}, y={y_3d}, z={z_3d}")
            else:
                self.get_logger().warn("Invalid depth at clicked point.")

    def calculate_3d_point(self, u, v, depth):
        if depth == 0:
            return None

        # Compute 3D point in the camera frame
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        z = depth
        return (x, y, z)

def main(args=None):
    rclpy.init(args=args)
    node = ClickPointTo3D()
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
