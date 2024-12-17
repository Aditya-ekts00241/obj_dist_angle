
##=============================================== convert pt cloud data to laser scan within visible range of +-30 degrees================================




import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, LaserScan
import numpy as np
from sensor_msgs_py.point_cloud2 import read_points

class PointCloudToLaserScan(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.laserscan_publisher_ = self.create_publisher(LaserScan, '/scan', 100)
        self.pointcloud_subscription_ = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.pointcloud_callback,
            100
        )

        # Updated angles to ignore the back side (degrees on each side from front center)
        self.angle_min = -2 * np.pi / 12 
        self.angle_max = 2 * np.pi / 12 
        self.angle_increment = np.pi / 80.0  # 1 degree
        self.range_min = 0.1  # Minimum range
        self.range_max = 100.0  # Maximum range

        # Set the z_thresholds to filter out top and bottom walls
        self.z_min_threshold = 0.05  # Adjust as needed for lower limit
        self.z_max_threshold = 1.5   # Adjust as needed for upper limit

        self.laserscan_data = None
        self.timer = self.create_timer(0.2, self.publish_laserscan)  # Reduce frequency to 5 Hz

    # def pointcloud_callback(self, msg):
    #     try:
    #         num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment)
    #         ranges = np.full(num_ranges, float('inf'))

    #         # points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
    #         # x, y, z = points[:, 0], points[:, 1], points[:, 2]

    #         points = list(read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
    #         points = np.array(points)  # Convert to NumPy array
    #         x, y, z = points[:, 0], points[:, 1], points[:, 2]

    #         # Filter out points that are too high or too low
    #         valid_z = (z >= self.z_min_threshold) & (z <= self.z_max_threshold)
    #         x, y, z = x[valid_z], y[valid_z], z[valid_z]

    #         angles = np.arctan2(y, x)
    #         ranges_values = np.sqrt(x**2 + y**2)

    #         valid_indices = (self.angle_min <= angles) & (angles <= self.angle_max) & (self.range_min <= ranges_values) & (ranges_values <= self.range_max)
    #         angles, ranges_values = angles[valid_indices], ranges_values[valid_indices]

    #         indices = ((angles - self.angle_min) / self.angle_increment).astype(int)

    #         for idx, range_val in zip(indices, ranges_values):
    #             if 0 <= idx < len(ranges):  # Ensure the index is within bounds
    #                 if range_val < ranges[idx]:
    #                     ranges[idx] = range_val

    #         self.laserscan_data = [r if r != float('inf') else self.range_max for r in ranges]

    #     except Exception as e:
    #         self.get_logger().error(f'Error in pointcloud_callback: {str(e)}')



    def pointcloud_callback(self, msg):
        try:
            num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment)
            ranges = np.full(num_ranges, float('inf'))

            # Properly read PointCloud2 data
            points = list(read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
            if not points:
                self.get_logger().warning('No points found in PointCloud2 message')
                return

            # Ensure proper 2D structure
            points_array = np.array([list(p) for p in points])  # Convert each tuple to a list and then create a 2D array

            # Log the shape to verify
            self.get_logger().info(f'Points shape: {points_array.shape}, type: {type(points_array)}')

            x, y, z = points_array[:, 0], points_array[:, 1], points_array[:, 2]

            # Filter out points that are too high or too low
            valid_z = (z >= self.z_min_threshold) & (z <= self.z_max_threshold)
            x, y, z = x[valid_z], y[valid_z], z[valid_z]

            angles = np.arctan2(y, x)
            ranges_values = np.sqrt(x**2 + y**2)

            valid_indices = (
                (self.angle_min <= angles) & (angles <= self.angle_max) &
                (self.range_min <= ranges_values) & (ranges_values <= self.range_max)
            )
            angles, ranges_values = angles[valid_indices], ranges_values[valid_indices]

            indices = ((angles - self.angle_min) / self.angle_increment).astype(int)

            for idx, range_val in zip(indices, ranges_values):
                if 0 <= idx < len(ranges):  # Ensure the index is within bounds
                    if range_val < ranges[idx]:
                        ranges[idx] = range_val

            self.laserscan_data = [r if r != float('inf') else self.range_max for r in ranges]

        except Exception as e:
            self.get_logger().error(f'Error in pointcloud_callback: {str(e)}')






    def publish_laserscan(self):
        if self.laserscan_data is not None:
            expected_readings = 241
            padded_data = self.laserscan_data

            # Pad with 'inf' if there are fewer readings than expected
            if len(padded_data) < expected_readings:
                padded_data.extend([float('inf')] * (expected_readings - len(padded_data)))
            elif len(padded_data) > expected_readings:
                padded_data = padded_data[:expected_readings]

            laserscan = LaserScan()
            laserscan.header.stamp = self.get_clock().now().to_msg()
            laserscan.header.frame_id = 'rslidar'
            laserscan.angle_min = self.angle_min
            laserscan.angle_max = self.angle_max
            laserscan.angle_increment = self.angle_increment
            laserscan.time_increment = 0.0
            laserscan.scan_time = 0.2  # Match with timer frequency
            laserscan.range_min = self.range_min
            laserscan.range_max = self.range_max
            laserscan.ranges = padded_data

            self.laserscan_publisher_.publish(laserscan)
            self.get_logger().info('Published LaserScan message')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
