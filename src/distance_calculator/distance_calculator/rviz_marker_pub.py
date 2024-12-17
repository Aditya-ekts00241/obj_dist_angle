import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import math
import json


class RVizTFPublisher(Node):
    def __init__(self):
        super().__init__('rviz_tf_publisher')

        # Subscribe to the topic from Node 1
        self.subscription = self.create_subscription(
            String,  # Assuming Node 1 publishes JSON strings
            '/object_data',  # Replace with actual topic name
            self.object_data_callback,
            10
        )

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Store processed objects and their IDs
        self.processed_objects = set()  # To avoid duplicates
        self.object_id_map = {}  # Maps class_name to unique IDs
        self.current_id = 0  # Incremental ID counter

        self.get_logger().info('RViz TF Publisher Node Initialized.')

    def object_data_callback(self, msg):
        try:
            # Parse the JSON message from Node 1
            object_data = json.loads(msg.data)

            class_name = object_data['class_name']
            distance = object_data['distance']
            angle = object_data['angle']

            # Check if the object is already processed
            if class_name in self.processed_objects:
                return

            # Add the object to the processed list
            self.processed_objects.add(class_name)

            # Assign a unique ID to the object
            if class_name not in self.object_id_map:
                self.object_id_map[class_name] = self.current_id
                self.current_id += 1

            # Convert polar coordinates (distance, angle) to Cartesian (x, y)
            x = distance * math.cos(math.radians(angle))
            y = distance * math.sin(math.radians(angle))

            # Publish the TF transform
            self.publish_tf(class_name, x, y)

            self.get_logger().info(f'Published TF for {class_name} at ({x:.2f}, {y:.2f}).')

        except Exception as e:
            self.get_logger().error(f'Error processing object data: {e}')

    def publish_tf(self, class_name, x, y):
        # Create a TransformStamped message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # Parent frame: map
        t.child_frame_id = f'object_{class_name}'  # Child frame for the object
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Send the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = RVizTFPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
