#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

def waypointCallback(msg):

    print("waypointCallback")

def cmd_velCallback(msg):
    cmd = Twist()
    cmd.angular.z = 0.0
    print("cmd_velCallback")
    cmd_vel_publisher.publish(cmd)
    

if __name__ == '__main__':
    rospy.init_node('waypoint_flag')
    rospy.Subscriber('/waypoint_manager/waypoint/is_reached', Bool, waypointCallback)
    rospy.Subscriber('/cmd_vel', Twist, cmd_velCallback)
    # Create a publisher for the cmd_vel topic
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        # Sleep to reduce CPU usage
        rospy.sleep(0.1)