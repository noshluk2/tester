#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from gazebo_msgs.srv import GetModelState
import os
import sys


desired_height = 0.6

def drone_flyer():
    global velocity_msg , pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("sonar_height", Range, sonar_callback)
    rospy.init_node('fly_node', anonymous=True)
    velocity_msg=Twist()
    rospy.spin()

def sonar_callback(msg):
    global velocity_msg , pub
    sonar_data=msg.range
    if(sonar_data>=desired_height):
        velocity_msg.linear.z=0.0
    else:
        velocity_msg.linear.z=1.2
        print("Reaching set point")
    pub.publish(velocity_msg)
    if(sonar_data>=desired_height):
        rospy.signal_shutdown("Reached the Point")

    print("Sonar Values : " , sonar_data)




if __name__ == '__main__':
    try:
        # if len(sys.argv) < 2:
        #     print("Not Reseting just hover Drone")
        # else:
        #     os.system("gz model -m quadrotor")
        #     print("Model reset now lets Hover It")
        #     print("Outing")
        # print("A")
        drone_flyer()


    except rospy.ROSInterruptException:
        pass