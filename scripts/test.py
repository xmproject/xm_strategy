#!/usr/bin/env python 
#encoding:utf8 

"""
author 
    
        X_Y耀  created on 2015-9-26
        

"""
import rospy
import tf
import actionlib
import math
from math import sqrt,copysign
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from rospy.exceptions import ROSInterruptException
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from people_msgs.msg import PositionMeasurementArray,PositionMeasurement
from actionlib_msgs.msg import GoalStatus
from time import sleep
from std_msgs.msg import String,Int32
import threading
from smach.concurrence import Concurrence
from xm_msgs.srv import *
from xm_msgs.msg import *
from Doorisopen.msg import *
from collections import OrderedDict
import subprocess
import dis
import PyKDL
from ar_track_alvar_msgs.msg import * 

OBJECT_ID=1

class ChangeAngular(State):
    def __init__(self,angle=0):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.speed = rospy.get_param('~speed', 0.20)
        self.secretindigal = 1.0
        self.tolerance = rospy.get_param('tolerance', math.radians(5))
        self.start = True
        self.cmd_vel = rospy.Publisher('mobile_base_controller/smooth_cmd_vel', Twist, queue_size=5)
        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        rospy.on_shutdown(self.shutdown)
        self.rate = 30
        self.start_test = True
        self.r = rospy.Rate(self.rate)
        self.angle = angle
        self.tf_listener = tf.TransformListener()
        rospy.sleep(0.5)
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60))
    def execute(self,userdata):
        move_cmd = Twist()
        while not rospy.is_shutdown():
            move_cmd = Twist()
            self.odom_angle = self.get_odom_angle()
            last_angle = self.odom_angle
            turn_angle = 0
            angular_speed = self.speed
            while abs(float(turn_angle)) < abs(float(self.angle)):
                if rospy.is_shutdown():
                    return
                if self.angle < 0:
                    move_cmd.angular.z = -angular_speed
                else:
                    move_cmd.angular.z = angular_speed
                self.cmd_vel.publish(move_cmd)
                self.r.sleep()
                self.odom_angle = self.get_odom_angle()
                delta_angle = self.secretindigal * self.normalize_angle(self.odom_angle - last_angle)
                turn_angle += delta_angle
                last_angle = self.odom_angle
            rospy.sleep(0.5)
            if move_cmd.angular.z != 0.0:
                self.cmd_vel.publish(move_cmd)
            else:
                break
            return "succeeded"
            self.cmd_vel.publish(Twist())

    def get_odom_angle(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return self.quat_to_angle(Quaternion(*rot))

    def quat_to_angle(self,quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]
    
    def normalize_angle(self,angle):
        res = angle
        while res > math.pi:
            res -= 2.0 * math.pi
        while res < -math.pi:
            res += 2.0 * math.pi
        return res

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        
class Lift_HEIGHT(State):
    def __init__(self,height=0):
        State.__init__(self, outcomes=["succeeded","aborted","preempted"])
        self.lift=actionlib.SimpleActionClient('xm_move_arm_height', xm_ArmHeightAction)
        self.height=height
    def execute(self, userdata):
        print self.height
        self.arm_goal = xm_ArmHeightGoal()
        self.arm_goal.height =self. height
        self.lift.send_goal(self.arm_goal)
        self.lift.wait_for_result(rospy.Duration(20))
        result=self.lift.get_result()
        print result
        print result.distance
        return "succeeded"
    
class ChangeMode(State):
    def __init__(self, msg=None):
        State.__init__(self, outcomes=['succeeded', "aborted", 'preempted'])
#         self.pub = rospy.Publisher('serial_switch_mode', xm_Serialswitchsode, queue_size=5)
        self.ser=rospy.ServiceProxy("serialswitchmode",xm_Serialswitchmode)
#         self.msg = xm_SerialSwitchMode()
        self.msg = msg

    def execute(self, userdata):
        if self.msg is not None:
            rospy.sleep(0.5)
            result=self.ser.call(self.msg)
            print result
            if result.switch_mode_ok ==True:
                return "succeeded"
            else :
                return 'aborted'
        pass
    
class Find_Object(State):
    def __init__(self, msg=None):
        State.__init__(self, outcomes=['succeeded', "aborted", 'preempted'])
#         self.pub = rospy.Publisher('serial_switch_mode', xm_Serialswitchsode, queue_size=5)
        self.ser=rospy.ServiceProxy("Find_Object",xm_ObjectDetect)
#         self.msg = xm_SerialSwitchMode()
        self.msg = msg

    def execute(self, userdata):
        global OBJECT_ID
        try:
            result=self.ser.call(OBJECT_ID)
            print result
            rospy.sleep(0.5)
            return "succeeded"
        except:
            print "Failed"
            return "aborted"
        pass
class test():
    def __init__(self):
        global  OBJECT_ID
        rospy.init_node("test")
        
        sm=StateMachine(outcomes=["succeeded","aborted","preempted","valid","invalid"])
                 
        self.lift=actionlib.SimpleActionClient('xm_move_arm_height', xm_ArmHeightAction)
        arm_goal = xm_ArmHeightGoal()
        arm_goal.distance_x = 1
        arm_goal.distance_z = -0.2
        self.arm_state=SimpleActionState('xm_move_arm_height',xm_ArmHeightAction,goal=arm_goal,
                                         result_cb=self.arm_state_cb, exec_timeout=rospy.Duration(60),
                                          server_wait_timeout=rospy.Duration(60))
        
        gripper_goal = xm_GripperCommandGoal()
        gripper_goal.command.position = 0.0 
#         gripper_goal.command.torque = 0.5
#         gripper_goal.header.frame
        gripper_state = SimpleActionState('xm_move_gripper', xm_GripperCommandAction, goal=gripper_goal,
                                          result_cb=self.wrist_cb, exec_timeout=rospy.Duration(10),
                                          server_wait_timeout=rospy.Duration(10))
        
        
        wrist_goal=xm_WristPositionGoal()
        wrist_goal.angle=0
        wrist_state=SimpleActionState("xm_move_wrist", xm_WristPositionAction,goal=wrist_goal,
                                          result_cb=self.gripper_state_cb, exec_timeout=rospy.Duration(10),
                                          server_wait_timeout=rospy.Duration(10))
        self.monitor_times=1
        sm.userdata.ob=0
        with sm:
#             StateMachine.add("FIND_OBJECT1", Find_Object(),
#                              transitions={"succeeded": "", 'aborted': ''}, )
#             StateMachine.add("angle", ChangeAngular(angle=0.2), transitions={"succeeded":""})
#             StateMachine.add("INIT",ServiceState('xm_move_arm_position/move_position', xm_ArmPosition,"prepare",response_cb=self.responce_cb),transitions={'succeeded':''})
#             StateMachine.add("HEIGHT",self.arm_state,transitions={"succeeded":''})
#             StateMachine.add("GRIP",gripper_state,transitions={"succeeded":''})
#                 StateMachine.add("AR_TARGET2", MonitorState("ar_pose_marker", AlvarMarkers,self.get_mark_pos_cb), transitions={"valid":"AR_TARGET2","invalid":""})
#                 StateMachine.add("WRIST", wrist_state, transitions={"succeeded":""})
            StateMachine.add("CHANGE_HEIGHT1", Lift_HEIGHT(height=0.0), transitions={"succeeded":""})
#             StateMachine.add("changemode", ChangeMode("arm"), transitions={"succeeded":""},)
        
#         for i in range(4):
        sm.execute()
#             OBJECT_ID+=1
        rospy.sleep(1)
        
    def get_mark_pos_cb(self,userdata,msg):
        global AR_POS
        if self.monitor_times>=15:
            self.monitor_times=0
            return True
        if len(msg.markers)==0:
            print msg.markers
            self.monitor_times+=1
            print "aaa" 
            return True
        print msg
        AR_POS=msg.markers[0].pose.pose.position
        print AR_POS
        self.monitor_times=0
        return False 

    def start_cb(self, userdata, msg):
        if msg.task == 0x01:
            return False
        else:
            return True

    def object_request_cb(self, userdata, request):
        global OBJECT_ID
        request = 1

    def find_object_cb(self, userdata, response):
        global OBJECT_POS
        if response.object.pos is None:
            return "aborted"
        OBJECT_POS = response.object.pos
        OBJECT_NAME=response.object.name
        OBJECT_WIDTH=response.object.width
        print OBJECT_POS
        return "succeeded"

    def responce_cb(self,userdata,response):
        return 'succeeded'
    
    def gripper_state_cb(self, userdata, status, result):
        print result
        rospy.loginfo("Complete!")
        
    def wrist_cb(self,userdata, status, result):
        print result
        
    def arm_state_cb(self,userdata,status,result):
        print result
        pass
if __name__ == '__main__':
    test()
