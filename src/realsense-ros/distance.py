import rospy
import math
from geometry_msgs.msg import PointStamped

def distance_from_camera(point):
    # Assuming the camera's origin is at (0, 0, 0) in its own frame
    distance = math.sqrt(point.x**2 + point.y**2 + point.z**2)
    rospy.loginfo("Distance from camera: {:.2f} meters".format(distance))

def clicked_point_callback(msg):
    # `msg` is a PointStamped containing the clicked point coordinates
    rospy.loginfo("Clicked Point: (x: {:.2f}, y: {:.2f}, z: {:.2f})".format(
        msg.point.x, msg.point.y, msg.point.z))
    distance_from_camera(msg.point)

def main():
    rospy.init_node('distance_calculator')
    rospy.Subscriber('/clicked_point', PointStamped, clicked_point_callback)
    rospy.loginfo("Distance Calculator Node Started. Click on a point in RViz.")
    rospy.spin()

if __name__ == '__main__':
    main()
