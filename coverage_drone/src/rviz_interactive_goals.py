#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PointStamped

class Pointers_to_goal:
    def __init__(self):
        self.number_subscriber = rospy.Subscriber("/clicked_point", PointStamped, self.rviz_pointer_call_back)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_points_x = []
        self.goal_points_y = []
        self.num_goals=5


    def rviz_pointer_call_back(self,msg):

        if(msg.header.seq == self.num_goals):
            print("X Points \n", self.goal_points_x , "\n Y Points \n",self.goal_points_y)
            self.client.wait_for_server()
            goal = MoveBaseGoal()
            for i in range(0,self.num_goals-1):
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = self.goal_points_x[i]
                goal.target_pose.pose.position.y = self.goal_points_y[i]
                goal.target_pose.pose.orientation.z = 0.99

                self.client.send_goal(goal)
                wait=self.client.wait_for_result()
                if not wait:
                    rospy.logerr("Action server Down !! ")
                else:
                    print("Goal {} is Completed".format(i))
                if(i==self.num_goals-1):
                    rospy.signal_shutdown("Reached the Point")
        else:
            self.goal_points_x.append(msg.point.x)
            self.goal_points_y.append(msg.point.y)
            print("A Point appended")



if __name__ == '__main__':
    rospy.init_node("Interactive_poninter_goals")
    Pointers_to_goal()
    rospy.spin()

