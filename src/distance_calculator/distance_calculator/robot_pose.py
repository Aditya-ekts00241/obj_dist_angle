import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf_ros

class RobotPoseNode(Node):
    def __init__(self):
        super().__init__('robot_pose_node')

        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.get_robot_pose)  # Run every second

    def get_robot_pose(self):
        try:
            # Get the transform from map to base_link
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            # Extract position
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y

            # Extract orientation (quaternion) and convert to yaw
            q = transform.transform.rotation
            _, _, robot_theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

            self.get_logger().info(f'Robot Pose: x={robot_x}, y={robot_y}, theta={robot_theta}')

        except Exception as e:
            self.get_logger().error(f'Could not get transform: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
