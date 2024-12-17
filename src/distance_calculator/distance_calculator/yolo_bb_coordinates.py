import torch
import cv2
from ultralytics import YOLO
import logging
import rclpy
from sensor_msgs.msg import LaserScan


# Callback function to process the first incoming message
def scan_callback(msg):
    # Define the angle range of interest
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    angle_increment = msg.angle_increment

    # Calculate the number of angles based on the given range
    num_angles = int((angle_max - angle_min) / angle_increment)

    # Extract relevant ranges (avoid "inf" values)
    filtered_ranges = []
    for i in range(num_angles):
        if msg.ranges[i] != float('inf'):  # Ignore "inf" values
            filtered_ranges.append(msg.ranges[i])

    # Reverse the filtered ranges
    reversed_ranges = filtered_ranges[::-1]

    print("LiDAR:", reversed_ranges)

rclpy.init()  # Initialize the ROS 2 Python client

# Create a node
node = rclpy.create_node('laser_scan_subscriber')

# Subscribe to the /scan topic, which publishes LaserScan messages
global subscription  # Declare the subscription as global so it can be accessed in the callback
subscription = node.create_subscription(
    LaserScan,
    '/scan',
    scan_callback,  # Callback function to process the incoming messages
    10  # Queue size
)

# Suppress YOLO logging
# logging.getLogger("ultralytics").setLevel(logging.WARNING)

# Load the YOLO model (replace with your best.pt file path)
model = YOLO('/home/kalyani-robotics/ros2_ws/src/distance_calculator/distance_calculator/best.pt')


for i in range(10, 0, -1):
    cam_index = f'/dev/video{i}'
    print(cam_index)
    cap = cv2.VideoCapture(cam_index)
    ret, frame = cap.read()
    if ret:
        print("Gotcha!")
        break
# Start capturing video
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame!")
        break
# Start capturing video
# cap = cv2.VideoCapture(3)

    # Perform inference
    results = model(frame)  # Run inference
    boxes = results[0].boxes  # Extract boxes object
    class_names = results[0].names  # Class names dictionary

    output_detections = []  # Store bounding box coordinates and class info

    for box in boxes:
        # Extract information from each box
        x_min, y_min, x_max, y_max = box.xyxy[0].cpu().numpy()  # Move to CPU and convert to NumPy
        conf = box.conf[0].cpu().item()  # Move to CPU and get confidence score
        class_id = int(box.cls[0].cpu().item())  # Move to CPU and get class ID
        class_name = class_names[class_id]  # Get class name from the model

        # Save bounding box info for output
        output_detections.append({
            "class_name": class_name,
            "x_min": x_min / frame.shape[1],
            "x_max": x_max / frame.shape[1],
            "y_min": y_min / frame.shape[0],
            "y_max": y_max / frame.shape[0],
            "confidence": conf,
        })

    # Display the live feed
    cv2.imshow("YOLO Detection", frame)


    print("Detections for current frame:")
    for det in output_detections:
        print(det)
    rclpy.spin_once(node)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):  # Press 'q' to quit
        break

# Release resources
cap.release()
node.destroy_node()
rclpy.shutdown()
cv2.destroyAllWindows()
