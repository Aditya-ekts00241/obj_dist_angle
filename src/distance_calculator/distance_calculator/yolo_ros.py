# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, LaserScan
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO

# class YOLODetectionNode(Node):
#     def __init__(self):
#         super().__init__('yolo_detection_node')

#         # Initialize YOLO model
#         self.model = YOLO('/home/kalyani-robotics/ros2_ws/src/distance_calculator/distance_calculator/best.pt')
#         self.bridge = CvBridge()

#         # Subscribe to image topic
#         self.image_subscription = self.create_subscription(
#             Image,
#             '/camera/camera/color/image_raw',
#             self.image_callback,
#             10
#         )

#         # Subscribe to LiDAR topic
#         self.lidar_subscription = self.create_subscription(
#             LaserScan,
#             '/scan',
#             self.lidar_callback,
#             10
#         )

#         self.lidar_data = None  # Store latest LiDAR data
#         self.get_logger().info('YOLO Detection Node Initialized.')

#     def image_callback(self, msg):
#         try:
#             # Convert ROS Image message to OpenCV format
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Perform YOLO inference
#             results = self.model(frame)  # Run inference
#             boxes = results[0].boxes  # Extract bounding boxes
#             class_names = results[0].names  # Class names dictionary

#             output_detections = []  # Store bounding box coordinates and class info

#             for box in boxes:
#                 # Extract information from each box
#                 x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy()  # Bounding box
#                 conf = box.conf[0].cpu().item()  # Confidence score
#                 class_id = int(box.cls[0].cpu().item())  # Class ID
#                 class_name = class_names[class_id]  # Class name

#                 # Save bounding box info for output
#                 output_detections.append({
#                     "class_name": class_name,
#                     "x_min": x_min / frame.shape[1],
#                     "x_max": x_max / frame.shape[1],
#                     "y_min": y_min / frame.shape[0],
#                     "y_max": y_max / frame.shape[0],
#                     "confidence": conf,
#                 })

#             # Log YOLO detections
#             if output_detections:
#                 self.get_logger().info(f'{len(output_detections)} detections:')
#                 for det in output_detections:
#                     self.get_logger().info(str(det))

#             # Display detections on frame
#             for box in boxes:
#                 x_min, y_min, x_max, y_max = map(int, box.xyxy[0].cpu().numpy())
#                 class_id = int(box.cls[0].cpu().item())
#                 label = f'{class_names[class_id]}: {box.conf[0].cpu().item():.2f}'
#                 cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
#                 cv2.putText(frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#             # Show the frame with detections
#             cv2.imshow("YOLO Detection", frame)
#             key = cv2.waitKey(1) & 0xFF
#             if key == ord("q"):  # Press 'q' to quit visualization
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error(f'Error in image processing: {str(e)}')

#         # Print combined LiDAR and YOLO data if LiDAR data is available
#         if self.lidar_data:
#             self.get_logger().info(f'LiDAR: {self.lidar_data}')

#     def lidar_callback(self, msg):
#         # Process LiDAR data
#         angle_min = msg.angle_min
#         angle_max = msg.angle_max
#         angle_increment = msg.angle_increment
#         num_angles = int((angle_max - angle_min) / angle_increment)

#         # Extract and filter ranges (avoid "inf" values)
#         filtered_ranges = [r for r in msg.ranges[:num_angles] if r != float('inf')]

#         # Reverse the filtered ranges
#         self.lidar_data = filtered_ranges[::-1]
#         self.get_logger().info(f'Received LiDAR data with {len(self.lidar_data)} valid ranges.')

# def main(args=None):
#     rclpy.init(args=args)
#     node = YOLODetectionNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()


























# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, LaserScan
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO
# from collections import defaultdict

# class YOLODetectionNode(Node):
#     def __init__(self):
#         super().__init__('yolo_detection_node')

#         # Initialize YOLO model
#         self.model = YOLO('/home/kalyani-robotics/ros2_ws/src/distance_calculator/distance_calculator/best.pt')
#         self.bridge = CvBridge()

#         # Subscribe to image topic
#         self.image_subscription = self.create_subscription(
#             Image,
#             '/camera/camera/color/image_raw',
#             self.image_callback,
#             10
#         )

#         # Subscribe to LiDAR topic
#         self.lidar_subscription = self.create_subscription(
#             LaserScan,
#             '/scan',
#             self.lidar_callback,
#             10
#         )

#         self.lidar_data = None  # Store latest LiDAR data
#         self.detection_history = defaultdict(list)  # Track detections over frames
#         self.get_logger().info('YOLO Detection Node Initialized.')

#     def image_callback(self, msg):
#         try:
#             # Convert ROS Image message to OpenCV format
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Perform YOLO inference
#             results = self.model(frame)  # Run inference
#             boxes = results[0].boxes  # Extract bounding boxes
#             class_names = results[0].names  # Class names dictionary

#             output_detections = []  # Store bounding box coordinates and class info

#             for box in boxes:
#                 # Extract information from each box
#                 x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy()  # Bounding box
#                 conf = box.conf[0].cpu().item()  # Confidence score
#                 class_id = int(box.cls[0].cpu().item())  # Class ID
#                 class_name = class_names[class_id]  # Class name

#                 # Save bounding box info for output
#                 detection = {
#                     "class_name": class_name,
#                     "x_min": x_min / frame.shape[1],
#                     "x_max": x_max / frame.shape[1],
#                     "y_min": y_min / frame.shape[0],
#                     "y_max": y_max / frame.shape[0],
#                     "confidence": conf,
#                 }
#                 output_detections.append(detection)

#                 # Update detection history
#                 key = (class_name, round(detection["x_min"], 3), round(detection["x_max"], 3))
#                 self.detection_history[key].append(detection)
#                 if len(self.detection_history[key]) > 3:
#                     self.detection_history[key].pop(0)

#             # Process detections seen in 3 consecutive frames
#             if self.lidar_data:
#                 lidar_length = len(self.lidar_data)
#                 for det in output_detections:
#                     key = (det["class_name"], round(det["x_min"], 3), round(det["x_max"], 3))
#                     if len(self.detection_history[key]) == 3:  # Seen in 3 consecutive frames
#                         start_index = int(det["x_min"] * lidar_length)
#                         stop_index = int(det["x_max"] * lidar_length)
#                         det["start_index"] = start_index
#                         det["stop_index"] = stop_index

#                         # Log detection with calculated indices
#                         self.get_logger().info(f'Detection with LiDAR indices: {det}')

#                 # Print the entire LiDAR array
#                 self.get_logger().info(f'LiDAR array: {self.lidar_data}')

#             # Display detections on frame
#             for box in boxes:
#                 x_min, y_min, x_max, y_max = map(int, box.xyxy[0].cpu().numpy())
#                 class_id = int(box.cls[0].cpu().item())
#                 label = f'{class_names[class_id]}: {box.conf[0].cpu().item():.2f}'
#                 cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
#                 cv2.putText(frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#             # Show the frame with detections
#             cv2.imshow("YOLO Detection", frame)
#             key = cv2.waitKey(1) & 0xFF
#             if key == ord("q"):  # Press 'q' to quit visualization
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error(f'Error in image processing: {str(e)}')

#     def lidar_callback(self, msg):
#         # Process LiDAR data
#         angle_min = msg.angle_min
#         angle_max = msg.angle_max
#         angle_increment = msg.angle_increment
#         num_angles = int((angle_max - angle_min) / angle_increment)

#         # Extract and filter ranges (avoid "inf" values)
#         filtered_ranges = [r for r in msg.ranges[:num_angles] if r != float('inf')]

#         # Reverse the filtered ranges
#         self.lidar_data = filtered_ranges[::-1]
#         self.get_logger().info(f'Received LiDAR data with {len(self.lidar_data)} valid ranges.')

# def main(args=None):
#     rclpy.init(args=args)
#     node = YOLODetectionNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()









































##====================================================== DISTANCE CALCULATOR (nearby objs) ==========================================================##








import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from collections import defaultdict

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

        self.lidar_data = None  # Store latest LiDAR data
        self.detection_history = defaultdict(list)  # Track detections over frames
        self.get_logger().info('YOLO Detection Node Initialized.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform YOLO inference
            results = self.model(frame)  # Run inference
            boxes = results[0].boxes  # Extract bounding boxes
            class_names = results[0].names  # Class names dictionary
            output_detections = []  # Store bounding box coordinates and class info

            for box in boxes:
                # Extract information from each box
                x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy()  # Bounding box
                conf = box.conf[0].cpu().item()  # Confidence score
                class_id = int(box.cls[0].cpu().item())  # Class ID
                class_name = class_names[class_id]  # Class name

                # Save bounding box info for output
                detection = {
                    "class_name": class_name,
                    "x_min": x_min / frame.shape[1],
                    "x_max": x_max / frame.shape[1],
                    "y_min": y_min / frame.shape[0],
                    "y_max": y_max / frame.shape[0],
                    "confidence": conf,
                }
                output_detections.append(detection)

                # Update detection history
                key = (class_name, round(detection["x_min"], 3), round(detection["x_max"], 3))
                self.detection_history[key].append(detection)
                if len(self.detection_history[key]) > 3:
                    self.detection_history[key].pop(0)

            # Process detections seen in 3 consecutive frames
            if self.lidar_data:
                lidar_length = len(self.lidar_data)
                for det in output_detections:
                    key = (det["class_name"], round(det["x_min"], 3), round(det["x_max"], 3))
                    if len(self.detection_history[key]) == 3:  # Seen in 3 consecutive frames
                        # Calculate refined x_min and x_max
                        x_centroid = (det["x_min"] + det["x_max"]) / 2
                        new_width = (det["x_max"] - det["x_min"]) / 2
                        det["x_min"] = max(0, x_centroid - (new_width / 2))
                        det["x_max"] = min(1, x_centroid + (new_width / 2))

                        start_index = int(det["x_min"] * lidar_length)
                        stop_index = int(det["x_max"] * lidar_length)


                        # det["start_index"] = start_index
                        # det["stop_index"] = stop_index


                        # Find the minimum value in the specified range
                        if start_index < stop_index:  # Valid range check
                            min_value = min(self.lidar_data[start_index:stop_index + 1])
                            det["min_lidar_value"] = min_value
                        else:
                            det["min_lidar_value"] = None  # Invalid range

                        # Log detection with calculated indices and minimum LiDAR value
                        self.get_logger().info(f'Detection with LiDAR indices and min value: {det}')


                        # # Print class name and min lidar value
                        self.get_logger().info(f'Class: {det["class_name"]}, Min LiDAR Value: {min_value}')


                # Print the entire LiDAR array
                self.get_logger().info(f'LiDAR array: {self.lidar_data}')

            # Display detections on frame
            for box in boxes:
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0].cpu().numpy())
                class_id = int(box.cls[0].cpu().item())
                label = f'{class_names[class_id]}: {box.conf[0].cpu().item():.2f}'
                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.putText(frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Show the frame with detections
            cv2.imshow("YOLO Detection", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):  # Press 'q' to quit visualization
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Error in image processing: {str(e)}')

    def lidar_callback(self, msg):


        # Process LiDAR data
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        num_angles = int((angle_max - angle_min) / angle_increment)
        # Extract and filter ranges (avoid "inf" values)
        filtered_ranges = [r for r in msg.ranges[:num_angles] if r != float('inf')]


        # filtered_ranges = [r for r in msg.ranges if r != float('inf')]
        # Reverse the filtered ranges
        self.lidar_data = filtered_ranges[::-1]
        self.get_logger().info(f'Received LiDAR data with {len(self.lidar_data)} valid ranges.')

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


















































##======================================================== OVERLAP BOUNDING BOX REMOVAL =============================================================##



# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, LaserScan
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO
# from collections import defaultdict

# class YOLODetectionNode(Node):
#     def __init__(self):
#         super().__init__('yolo_detection_node')

#         # Initialize YOLO model
#         self.model = YOLO('/home/kalyani-robotics/ros2_ws/src/distance_calculator/distance_calculator/best.pt')
#         self.bridge = CvBridge()

#         # Subscribe to image topic
#         self.image_subscription = self.create_subscription(
#             Image,
#             '/camera/camera/color/image_raw',
#             self.image_callback,
#             10
#         )

#         # Subscribe to LiDAR topic
#         self.lidar_subscription = self.create_subscription(
#             LaserScan,
#             '/scan',
#             self.lidar_callback,
#             10
#         )

#         self.lidar_data = None  # Store latest LiDAR data
#         self.detection_history = defaultdict(list)  # Track detections over frames
#         self.get_logger().info('YOLO Detection Node Initialized.')

#     def image_callback(self, msg):
#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             results = self.model(frame)
#             boxes = results[0].boxes
#             class_names = results[0].names
#             detections = []

#             for box in boxes:
#                 x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy()
#                 conf = box.conf[0].cpu().item()
#                 class_id = int(box.cls[0].cpu().item())
#                 class_name = class_names[class_id]

#                 detections.append({
#                     "class_name": class_name,
#                     "x_min": x_min / frame.shape[1],
#                     "x_max": x_max / frame.shape[1],
#                     "y_min": y_min / frame.shape[0],
#                     "y_max": y_max / frame.shape[0],
#                     "confidence": conf,
#                 })

#             # Filter detections to ignore ones above another box
#             filtered_detections = []
#             for det in detections:
#                 ignore = False
#                 for other_det in detections:
#                     if det == other_det:
#                         continue
#                     # Check if `det` is above `other_det` (partially or fully)
#                     if det["y_min"] < other_det["y_max"] and det["y_max"] <= other_det["y_max"]:
#                         ignore = True
#                         break
#                 if not ignore:
#                     filtered_detections.append(det)

#             if self.lidar_data:
#                 lidar_length = len(self.lidar_data)
#                 for det in filtered_detections:
#                     x_centroid = (det["x_min"] + det["x_max"]) / 2
#                     new_width = (det["x_max"] - det["x_min"]) / 2
#                     det["x_min"] = max(0, x_centroid - (new_width / 2))
#                     det["x_max"] = min(1, x_centroid + (new_width / 2))

#                     start_index = int(det["x_min"] * lidar_length)
#                     stop_index = int(det["x_max"] * lidar_length)
#                     # det["start_index"] = start_index
#                     # det["stop_index"] = stop_index

#                     if start_index < stop_index:
#                         min_value = min(self.lidar_data[start_index:stop_index + 1])
#                         det["min_lidar_value"] = min_value
#                     else:
#                         det["min_lidar_value"] = None

#                     #self.get_logger().info(f'Detection: {det}')

#                     self.get_logger().info(f'class name: {det["class_name"]}')
#                     self.get_logger().info(f'min_lidar_value: {det["min_lidar_value"]}')



#                 # self.get_logger().info(f'LiDAR array: {self.lidar_data}')

#             for box in filtered_detections:
#                 x_min, y_min, x_max, y_max = (
#                     int(box["x_min"] * frame.shape[1]),
#                     int(box["y_min"] * frame.shape[0]),
#                     int(box["x_max"] * frame.shape[1]),
#                     int(box["y_max"] * frame.shape[0])
#                 )
#                 label = f'{box["class_name"]}: {box["confidence"]:.2f}'
#                 cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
#                 cv2.putText(frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#             cv2.imshow("YOLO Detection", frame)
#             key = cv2.waitKey(1) & 0xFF
#             if key == ord("q"):
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error(f'Error in image processing: {str(e)}')

#     def lidar_callback(self, msg):
#         angle_min = msg.angle_min
#         angle_max = msg.angle_max
#         angle_increment = msg.angle_increment
#         num_angles = int((angle_max - angle_min) / angle_increment)
#         filtered_ranges = [r for r in msg.ranges[:num_angles] if r != float('inf')]
#         self.lidar_data = filtered_ranges[::-1]
#         # self.get_logger().info(f'Received LiDAR data with {len(self.lidar_data)} valid ranges.')

# def main(args=None):
#     rclpy.init(args=args)
#     node = YOLODetectionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()




















































##============================================================= JUST ANGLE CALCULATION =================================================================##


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, LaserScan
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO
# from collections import defaultdict
# import numpy as np

# class YOLODetectionNode(Node):
#     def __init__(self):
#         super().__init__('yolo_detection_node')

#         # Initialize YOLO model
#         self.model = YOLO('/home/kalyani-robotics/ros2_ws/src/distance_calculator/distance_calculator/best.pt')
#         self.bridge = CvBridge()

#         # Subscribe to image topic
#         self.image_subscription = self.create_subscription(
#             Image,
#             '/camera/camera/color/image_raw',
#             self.image_callback,
#             10
#         )

#         # Subscribe to LiDAR topic
#         self.lidar_subscription = self.create_subscription(
#             LaserScan,
#             '/scan',
#             self.lidar_callback,
#             10
#         )

#         self.lidar_data = None  # Store latest LiDAR data
#         self.detection_history = defaultdict(list)  # Track detections over frames
#         self.angle_min = -2 * np.pi / 12
#         self.angle_max = 2 * np.pi / 12
#         self.angle_increment = np.pi / 80.0
#         self.get_logger().info('YOLO Detection Node Initialized.')

#     def image_callback(self, msg):
#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             results = self.model(frame)
#             boxes = results[0].boxes
#             class_names = results[0].names
#             detections = []

#             for box in boxes:
#                 x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy()
#                 conf = box.conf[0].cpu().item()
#                 class_id = int(box.cls[0].cpu().item())
#                 class_name = class_names[class_id]

#                 detections.append({
#                     "class_name": class_name,
#                     "x_min": x_min / frame.shape[1],
#                     "x_max": x_max / frame.shape[1],
#                     "y_min": y_min / frame.shape[0],
#                     "y_max": y_max / frame.shape[0],
#                     "confidence": conf,
#                 })

#             # Filter detections to ignore ones above another box
#             filtered_detections = []
#             for det in detections:
#                 ignore = False
#                 for other_det in detections:
#                     if det == other_det:
#                         continue
#                     if det["y_min"] < other_det["y_max"] and det["y_max"] <= other_det["y_max"]:
#                         ignore = True
#                         break
#                 if not ignore:
#                     filtered_detections.append(det)

#             if self.lidar_data:
#                 lidar_length = len(self.lidar_data)
#                 for det in filtered_detections:
#                     x_centroid = (det["x_min"] + det["x_max"]) / 2
#                     new_width = (det["x_max"] - det["x_min"]) / 2
#                     det["x_min"] = max(0, x_centroid - (new_width / 2))
#                     det["x_max"] = min(1, x_centroid + (new_width / 2))

#                     start_index = int(det["x_min"] * lidar_length)
#                     stop_index = int(det["x_max"] * lidar_length)
#                     det["start_index"] = start_index
#                     det["stop_index"] = stop_index

#                     if start_index < stop_index:
#                         min_value = min(self.lidar_data[start_index:stop_index + 1])
#                         det["min_lidar_value"] = min_value
#                         centroid_index = int((start_index + stop_index) / 2)
#                         det["angle"] = self.angle_min + (centroid_index * self.angle_increment)
#                     else:
#                         det["min_lidar_value"] = None
#                         det["angle"] = None

#                     self.get_logger().info(f'Detection: {det}')

#                 self.get_logger().info(f'LiDAR array: {self.lidar_data}')

#             for box in filtered_detections:
#                 x_min, y_min, x_max, y_max = (
#                     int(box["x_min"] * frame.shape[1]),
#                     int(box["y_min"] * frame.shape[0]),
#                     int(box["x_max"] * frame.shape[1]),
#                     int(box["y_max"] * frame.shape[0])
#                 )
#                 label = f'{box["class_name"]}: {box["confidence"]:.2f}'
#                 cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
#                 cv2.putText(frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#             cv2.imshow("YOLO Detection", frame)
#             key = cv2.waitKey(1) & 0xFF
#             if key == ord("q"):
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error(f'Error in image processing: {str(e)}')

#     def lidar_callback(self, msg):
#         angle_min = msg.angle_min
#         angle_max = msg.angle_max
#         angle_increment = msg.angle_increment
#         num_angles = int((angle_max - angle_min) / angle_increment)
#         filtered_ranges = [r for r in msg.ranges[:num_angles] if r != float('inf')]
#         self.lidar_data = filtered_ranges[::-1]
#         self.get_logger().info(f'Received LiDAR data with {len(self.lidar_data)} valid ranges.')

# def main(args=None):
#     rclpy.init(args=args)
#     node = YOLODetectionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()








































































##========================================================== DISTANCE + ANGLE ===============================================================##





# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, LaserScan
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO
# from collections import defaultdict
# import numpy as np

# class YOLODetectionNode(Node):
#     def __init__(self):
#         super().__init__('yolo_detection_node')

#         # Initialize YOLO model
#         self.model = YOLO('/home/kalyani-robotics/ros2_ws/src/distance_calculator/distance_calculator/best.pt')
#         self.bridge = CvBridge()

#         # Subscribe to image topic
#         self.image_subscription = self.create_subscription(
#             Image,
#             '/camera/camera/color/image_raw',
#             self.image_callback,
#             10
#         )

#         # Subscribe to LiDAR topic
#         self.lidar_subscription = self.create_subscription(
#             LaserScan,
#             '/scan',
#             self.lidar_callback,
#             10
#         )

#         self.lidar_data = None  # Store latest LiDAR data
#         self.detection_history = defaultdict(list)  # Track detections over frames
#         self.angle_min = -2 * np.pi / 12
#         self.angle_max = 2 * np.pi / 12
#         self.angle_increment = np.pi / 80.0
#         self.get_logger().info('YOLO Detection Node Initialized.')

#     def image_callback(self, msg):
#         try:
#             # Convert ROS Image message to OpenCV format
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Perform YOLO inference
#             results = self.model(frame)  # Run inference
#             boxes = results[0].boxes  # Extract bounding boxes
#             class_names = results[0].names  # Class names dictionary
#             output_detections = []  # Store bounding box coordinates and class info

#             for box in boxes:
#                 # Extract information from each box
#                 x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy()  # Bounding box
#                 conf = box.conf[0].cpu().item()  # Confidence score
#                 class_id = int(box.cls[0].cpu().item())  # Class ID
#                 class_name = class_names[class_id]  # Class name

#                 # Save bounding box info for output
#                 detection = {
#                     "class_name": class_name,
#                     "x_min": x_min / frame.shape[1],
#                     "x_max": x_max / frame.shape[1],
#                     "y_min": y_min / frame.shape[0],
#                     "y_max": y_max / frame.shape[0],
#                     "confidence": conf,
#                 }
#                 output_detections.append(detection)

#                 # Update detection history
#                 key = (class_name, round(detection["x_min"], 3), round(detection["x_max"], 3))
#                 self.detection_history[key].append(detection)
#                 if len(self.detection_history[key]) > 3:
#                     self.detection_history[key].pop(0)

#             # Process detections seen in 3 consecutive frames
#             if self.lidar_data:
#                 lidar_length = len(self.lidar_data)
#                 for det in output_detections:
#                     key = (det["class_name"], round(det["x_min"], 3), round(det["x_max"], 3))
#                     if len(self.detection_history[key]) == 3:  # Seen in 3 consecutive frames
#                         # Calculate refined x_min and x_max
#                         x_centroid = (det["x_min"] + det["x_max"]) / 2
#                         new_width = (det["x_max"] - det["x_min"]) / 2
#                         det["x_min"] = max(0, x_centroid - (new_width / 2))
#                         det["x_max"] = min(1, x_centroid + (new_width / 2))

#                         start_index = int(det["x_min"] * lidar_length)
#                         stop_index = int(det["x_max"] * lidar_length)

#                         # Find the minimum value in the specified range
#                         if start_index < stop_index:  # Valid range check
#                             min_value = min(self.lidar_data[start_index:stop_index + 1])
#                             det["min_lidar_value"] = min_value

#                             # Calculate the angle of the object
#                             centroid_index = int((start_index + stop_index) / 2)
#                             angle = self.angle_min + (centroid_index * self.angle_increment)
#                             det["angle"] = angle

#                             # Print class name, distance, and angle
#                             self.get_logger().info(
#                                 f'Class: {det["class_name"]}, Distance: {min_value:.2f} m, Angle: {np.degrees(angle):.2f} degrees'
#                             )

#                         else:
#                             det["min_lidar_value"] = None  # Invalid range
#                             det["angle"] = None

#             # Show the frame with detections
#             for box in boxes:
#                 x_min, y_min, x_max, y_max = map(int, box.xyxy[0].cpu().numpy())
#                 class_id = int(box.cls[0].cpu().item())
#                 label = f'{class_names[class_id]}: {box.conf[0].cpu().item():.2f}'
#                 cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
#                 cv2.putText(frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#             cv2.imshow("YOLO Detection", frame)
#             key = cv2.waitKey(1) & 0xFF
#             if key == ord("q"):  # Press 'q' to quit visualization
#                 rclpy.shutdown()

#         except Exception as e:
#             self.get_logger().error(f'Error in image processing: {str(e)}')

#     def lidar_callback(self, msg):
#         # Process LiDAR data
#         angle_min = msg.angle_min
#         angle_max = msg.angle_max
#         angle_increment = msg.angle_increment
#         num_angles = int((angle_max - angle_min) / angle_increment)
#         # Extract and filter ranges (avoid "inf" values)
#         filtered_ranges = [r for r in msg.ranges[:num_angles] if r != float('inf')]
#         self.lidar_data = filtered_ranges[::-1]  # Reverse the filtered ranges
#         self.get_logger().info(f'Received LiDAR data with {len(self.lidar_data)} valid ranges.')

# def main(args=None):
#     rclpy.init(args=args)
#     node = YOLODetectionNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()














































##===================================================NODE THAT PUBLISHES DIST,ANGLE,NAME===================================================##

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import json
import math


class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')

        # Initialize YOLO model
        self.model = YOLO('/home/kalyani-robotics/ros2_ws/src/distance_calculator/distance_calculator/best.pt')
        self.bridge = CvBridge()

        # Publishers
        self.object_data_publisher = self.create_publisher(String, '/object_data', 10)

        # Subscribe to image topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
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

        self.lidar_data = None  # Store latest LiDAR data
        self.get_logger().info('YOLO Detection Node Initialized.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform YOLO inference
            results = self.model(frame)  # Run inference
            boxes = results[0].boxes  # Extract bounding boxes
            class_names = results[0].names  # Class names dictionary
            output_detections = []  # Store bounding box coordinates and class info

            for box in boxes:
                # Extract information from each box
                x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy()  # Bounding box
                conf = box.conf[0].cpu().item()  # Confidence score
                class_id = int(box.cls[0].cpu().item())  # Class ID
                class_name = class_names[class_id]  # Class name

                # Save bounding box info for output
                detection = {
                    "class_name": class_name,
                    "x_min": x_min / frame.shape[1],
                    "x_max": x_max / frame.shape[1],
                    "y_min": y_min / frame.shape[0],
                    "y_max": y_max / frame.shape[0],
                    "confidence": conf,
                }
                output_detections.append(detection)

            if self.lidar_data:
                self.process_detections(output_detections)

        except Exception as e:
            self.get_logger().error(f'Error in image processing: {str(e)}')

    def lidar_callback(self, msg):
        # Process LiDAR data
        # angle_min = msg.angle_min
        # angle_max = msg.angle_max
        angle_min = -0.5
        angle_max = +0.5
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        num_angles = len(ranges)

        # Extract and filter ranges (avoid "inf" values)
        filtered_ranges = [r for r in ranges if r != float('inf')]
        self.lidar_data = {
            "ranges": filtered_ranges,
            "angle_min": angle_min,
            "angle_max": angle_max,
            "angle_increment": angle_increment,
            "num_angles": num_angles,
        }

    def process_detections(self, detections):
        lidar_ranges = self.lidar_data["ranges"]
        angle_min = self.lidar_data["angle_min"]
        angle_increment = self.lidar_data["angle_increment"]

        for det in detections:
            # Calculate bounding box indices
            # lidar_length = len(lidar_ranges)
            lidar_length = len([value for value in lidar_ranges if value > 0])
            x_centroid = (det["x_min"] + det["x_max"]) / 2
            start_index = int(det["x_min"] * lidar_length)
            stop_index = int(det["x_max"] * lidar_length)

            # Calculate distance
            if start_index < stop_index:
                relevant_ranges = lidar_ranges[start_index:stop_index + 1]
                distance = min(filter(lambda x: x > 0, relevant_ranges), default=None)
            else:
                distance = None

            if distance:
                # Calculate angle
                center_index = (start_index + stop_index) // 2
                # angle = angle_min + center_index * angle_increment
                angle_deg = math.degrees(angle_min) + ((center_index/lidar_length)*60)

                # Publish object data
                object_data = {
                    "class_name": det["class_name"],
                    "distance": round(distance, 2),
                    "angle": round(angle_deg, 2),
                }
                self.publish_object_data(object_data)

    def publish_object_data(self, object_data):
        # Publish object data as a JSON string
        msg = String()
        msg.data = json.dumps(object_data)
        self.object_data_publisher.publish(msg)
        self.get_logger().info(f'Published Object Data: {object_data}')


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


if __name__ == '__main__':
    main()
