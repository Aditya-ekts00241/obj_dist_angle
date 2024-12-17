import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kalyani-robotics/ros2_ws/src/distance_calculator/install/distance_calculator'
