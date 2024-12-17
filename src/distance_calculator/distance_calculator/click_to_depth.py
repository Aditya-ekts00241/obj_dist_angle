import rclpy
import struct
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

# Custom read_points function to replace the missing sensor_msgs.point_cloud2 utility
def read_points(cloud_msg, skip_nans=True):
    """
    Parse PointCloud2 data and return an iterator of 3D points (x, y, z).
    
    :param cloud_msg: The PointCloud2 message.
    :param skip_nans: If True, then don't return any point with a NaN value.
    :return: List of 3D points (x, y, z).
    """
    points = []
    fmt = 'fff'  # format for x, y, z (float, float, float)
    for i in range(cloud_msg.width * cloud_msg.height):
        offset = i * cloud_msg.point_step
        x, y, z = struct.unpack_from(fmt, cloud_msg.data, offset)
        if skip_nans and (np.isnan(x) or np.isnan(y) or np.isnan(z)):
            continue
        points.append((x, y, z))
    return points

class ClickPointTo3D(Node):
    def __init__(self):
        super().__init__('click_point_to_3d')
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            # '/camera/depth/points',
            self.pointcloud_callback,
            10)
        
        # Initialize image and point cloud data
        self.rgb_image = None
        self.pointcloud_data = None

        # Window for displaying RGB image and handling clicks
        cv2.namedWindow("RGB Image")
        cv2.setMouseCallback("RGB Image", self.mouse_callback)

    def image_callback(self, msg):
        self.get_logger().info("Received RGB image")
        # Convert the ROS Image message to OpenCV format
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Display the RGB image
        cv2.imshow("RGB Image", self.rgb_image)
        cv2.waitKey(10)

    def pointcloud_callback(self, msg):
        # Save the point cloud data
        self.pointcloud_data = msg

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_3d_point(x, y)

    def get_3d_point(self, x, y):
        # Ensure we have both RGB image and point cloud data
        if self.rgb_image is None or self.pointcloud_data is None:
            self.get_logger().warning("No data received yet!")
            return
        
        print(x,y)
        
        # Convert the 2D pixel location to 3D by iterating through the point cloud
        # PointCloud2 is stored row-major, so we need to map x, y to a row-major index
        width, height = self.rgb_image.shape[1], self.rgb_image.shape[0]
        index = y * width + x  # Map to the row-major index

        # Extract the 3D point from the point cloud using the custom read_points
        point_gen = read_points(self.pointcloud_data, skip_nans=True)
        
        # Get the corresponding 3D point
        if index < len(point_gen):
            point = point_gen[index]
            x, y, z = point
            self.get_logger().info(f"Clicked 2D point ({x}, {y}) maps to 3D point: ({x}, {y}, {z})")
        else:
            self.get_logger().warning("Could not find corresponding 3D point.")
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
