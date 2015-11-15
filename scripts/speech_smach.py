#!/usr/bin/env python 
#encoding:utf8 

"""
author 
    
        X_Y耀  created on 2015-8-15

"""
from actionlib import GoalStatus
import actionlib
from collections import OrderedDict
import easygui
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from math import pi, sqrt
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from nav_msgs.msg import Odometry
from people_msgs.msg import PositionMeasurementArray, PositionMeasurement
from roscpp.srv._Empty import Empty
import rospy
from rospy.topics import Subscriber
from smach.state_machine import StateMachine
from smach_ros import MonitorState,SimpleActionState
from std_msgs.msg import Int32
import std_srvs
import subprocess
from tf.transformations import quaternion_from_euler
import threading
from time import sleep
import time

from Doorisopen.msg import *
from pi_trees_ros.pi_trees_ros import *
from xm_msgs.msg import *


POS =Point(0,0,0)

class speech():
    def __init__(self):
        rospy.init_node("speech")
        rospy.on_shutdown(self.shutdown)
        
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
        rospy.loginfo("Waiting for move_base action server...")
    
        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))    
        
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = 'base_link'
        nav_goal.target_pose.header.stamp = rospy.Time.now()
        q_angle = quaternion_from_euler(0, 0,0, axes='sxyz')
        q = Quaternion(*q_angle)
        nav_goal.target_pose.pose =Pose( Point(0.3,0,0),q)
        move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb, 
                                                exec_timeout=rospy.Duration(60.0),
                                                server_wait_timeout=rospy.Duration(60.0))
        
        
        smach_top=StateMachine(outcomes=['succeeded','aborted','preempted',"valid","invalid"])
        with smach_top:
            StateMachine.add("Wait_4_Statr", MonitorState("task_comming", xm_Task, self.start_cb),
                                                  transitions={'invalid':'NAV',
                                                               'valid':'Wait_4_Statr',
                                                               'preempted':'NAV'})
                         
            StateMachine.add("NAV", move_base_state, transitions={"succeeded":"Wait_4_Stop","aborted":"NAV","preempted":"Wait_4_Stop"})
            StateMachine.add("Wait_4_Stop",MonitorState("task_comming", xm_Task, self.stop_cb),
                                                  transitions={'invalid':'',
                                                               'valid':'Wait_4_Stop',
                                                               'preempted':''})
        
            smach_top.execute()
                         
    
    def start_cb(self,userdata,msg):
        if msg.task==0x01 :
            return False
        else:
            return True
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        rospy.sleep(1)
    pass

    def stop_cb(self,userdata,msg):
        if msg.task==0x02:
            return False
        else:
            return True
    def move_base_result_cb(self,userdata,status,result):
        if status:
            return "succeeded"
        
        pass
        
if __name__ == '__main__':
    try:
        speech()
    except KeyboardInterrupt:
        pass
