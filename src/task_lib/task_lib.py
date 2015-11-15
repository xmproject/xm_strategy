#!/usr/bin/env python

"""
author : X_Y 耀

"""
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rbx2_msgs.srv import *
from pi_trees_ros.pi_trees_ros import *
from rbx2_tasks.task_setup import *
from rbx2_tasks.clean_house_tasks_tree import *
from collections import OrderedDict
from math import pi, sqrt
import time
import easygui
from geometry_msgs.msg._Pose import Pose
from geometry_msgs.msg._Point import Point

class Stop(Task):
    def __init__(self, *args, **kwargs):
        name = "STOP" 
        super(Stop, self).__init__(name)    
        self.name = name
    def run(self):
        self.move_base.cancel_all_goals()
        
        self.cmd_vel_pub.publish(Twist())
        
        rospy.sleep(1)

        return TaskStatus.SUCCESS
    
class Nav2Point(Task):
    def __init__(self, *args, **kwargs):
        name = "ENTER_HOME" 
        super(Nav2Point, self).__init__(name)    
        self.name = name
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(1,0,0),0)
        move_base_task = SimpleActionTask("GET_IN_HOME", "move_base", MoveBaseAction, goal, reset_after=False)
    def run(self):
        self.move_base.cancel_all_goals()
        
        self.cmd_vel_pub.publish(Twist())
        
        rospy.sleep(1)

        return TaskStatus.SUCCESS

if __name__=="main":
    
    pass

    