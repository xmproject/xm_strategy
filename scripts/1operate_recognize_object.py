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
from xm_open_door.msg import *
from collections import OrderedDict
import subprocess
import dis

target=Point(0,0,0)

class Forword_BacK(State):
    def __init__(self,dis=0):
        State.__init__(self, outcomes=['succeeded','preempted','aborted'])
        rospy.on_shutdown(self.shutdown)
        self.test_distance = dis
        self.rate = 100
        self.r = rospy.Rate(self.rate)
        self.speed = rospy.get_param('~speed',0.08)
        self.tolerance = rospy.get_param('~tolerance', 0.01)
        self.cmd_vel = rospy.Publisher('/mobile_base_controller/smooth_cmd_vel',Twist,queue_size=5)
        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1)
        self.tf_listener.waitForTransform(self.odom_frame,self.base_frame,rospy.Time(),rospy.Duration(60.0))
        self.flag = rospy.get_param('~flag', True)
        #tf get position
        self.position = Point()
        self.position = self.get_position()
        self.y_start = self.position.y
        self.x_start = self.position.x
        #publish cmd_vel
    def execute(self,userdata):
        while not rospy.is_shutdown():
            move_cmd = Twist()
            if self.flag:
                self.position = self.get_position()
                distance = float(sqrt(pow((self.position.x -self. x_start), 2) +
                                pow((self.position.y - self.y_start), 2)))
                if self.test_distance > 0:
                    self.error = float(distance - self.test_distance)
                else:
                    self.error = -float(distance + self.test_distance)
                if not self.flag or abs(self.error) < self.tolerance:
                    self.flag = False
                else:
                    move_cmd.linear.x = copysign(self.speed, -1 * self.error)
            else:
                self.position = self.get_position()
                self.x_start = self.position.x
                self.y_start = self.position.y
            self.cmd_vel.publish(move_cmd)
            if move_cmd.linear.x != 0.0:
                self.cmd_vel.publish(move_cmd)
            else:
                break
            self.r.sleep()
        self.cmd_vel.publish(Twist())
        return "succeeded"
    def get_position(self):
        try:
            (trans,rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.lookupException):
            rospy.loginfo("TF Exception")
            return
        return Point(*trans)
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
    pass

class Left_Right(State):
    def __init__(self,dis=0.0):
        State.__init__(self, outcomes=['succeeded','preempted','aborted'])
        rospy.on_shutdown(self.shutdown)
        self.test_distance = target.y-0.0
        self.rate = 100
        self.r = rospy.Rate(self.rate)
        self.speed = rospy.get_param('~speed',0.08)
        self.tolerance = rospy.get_param('~tolerance', 0.01)
        self.cmd_vel = rospy.Publisher('/mobile_base_controller/smooth_cmd_vel',Twist,queue_size=5)
        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1)
        self.tf_listener.waitForTransform(self.odom_frame,self.base_frame,rospy.Time(),rospy.Duration(60.0))
        #define a bianliang
        self.flag = rospy.get_param('~flag', True)
        rospy.loginfo(self.test_distance)
        #tf get position
        
        #publish cmd_vel
    def execute(self, userdata):  
        global target
        print target
        self.test_distance = target.y-0.0
        self.position = Point()
        self.position = self.get_position()
        self.y_start = self.position.y
        self.x_start = self.position.x
        rospy.loginfo(self.test_distance)
        rospy.loginfo(self.x_start)
        rospy.loginfo(self.y_start)
        move_cmd = Twist()
        while not rospy.is_shutdown():
            move_cmd = Twist()
            if self.flag:
                self.position = self.get_position()
                distance = float(sqrt(pow((self.position.x - self.x_start), 2) +
                                pow((self.position.y - self.y_start), 2)))
                rospy.loginfo(distance)
                if self.test_distance > 0:
                    self.error = float(distance - float(self.test_distance))
                else:
                    self.error = -float(distance + self.test_distance)
                if not self.flag or abs(self.error) < self.tolerance:
                    self.flag = False
                else:
                    move_cmd.linear.y = copysign(self.speed, -1 * self.error)
            else:
                self.position = self.get_position()
                self.x_start = self.position.x
                self.y_start = self.position.y
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
            if move_cmd.linear.y != 0.0:
                self.cmd_vel.publish(move_cmd)
            else:
                self.cmd_vel.publish(Twist())
                break
        return "succeeded"
        
    def get_position(self):
        try:
            (trans,rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.lookupException):
            rospy.loginfo("TF Exception")
            return
        return Point(*trans)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


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
        
class ChangeMode(State):
    def __init__(self,msg=None):
        State.__init__(self, outcomes=['succeeded',"succeeded",'preempted'])
#        self.pub=rospy.Publisher('serial_switch_model',xm_SerialSwitchMode,queue_size=5 )
#        self.msg=xm_SerialSwitchMode()
#        self.msg.data=msg
        self.ser=rospy.ServiceProxy("serialswitchmode",xm_SerialSwitchMode)
        self.msg = msg

    def execute(self, userdata):
        if self.msg is not None:
            rospy.sleep(0.5)
#            self.pub.publish(self.msg)
#        return 'succeeded'
            result=self.ser.call(self.msg)
            print result
            if result.switch_mode_ok ==True:
                return "succeeded"
            else :
                return 'aborted'
        pass
    
class object():
    def __init__(self):
        global target
        rospy.init_node('object', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        grasp_sm=StateMachine(outcomes=['succeeded','aborted','preempted'])
        self.target=Point
        
        gripper_goal = xm_GripperCommandGoal()
        gripper_goal.command.position = 0.10
        gripper_goal.command.torque = 0.5
#         gripper_goal.header.frame
        gripper_state = SimpleActionState('xm_move_gripper', xm_GripperCommandAction, goal=gripper_goal,
                                          result_cb=self.gripper_state_cb, exec_timeout=rospy.Duration(10),
                                          server_wait_timeout=rospy.Duration(10))
#         
#         self.lift=actionlib.SimpleActionClient('xm_move_arm_height', xm_ArmHeightAction)
#         arm_goal = xm_ArmHeightGoal()
#         arm_goal.distance_x = 0.5
#         arm_goal.distance_z = 0.1
#         self.arm_state=SimpleActionState('xm_move_gripper',xm_ArmHeightAction,goal=arm_goal,
#                                          result_cb=self.arm_state_cb, exec_timeout=rospy.Duration(10),
#                                           server_wait_timeout=rospy.Duration(10))
        change2arm=ChangeMode("arm")
        change2base=ChangeMode("base")
        rospy.loginfo("ready")
        with grasp_sm:
#             StateMachine.add("INIT",ServiceState('xm_move_arm_position/move_position', xm_ArmPosition,"init",response_cb=self.responce_cb),transitions={'succeeded':'FIND_OBJECT'})
            StateMachine.add("FIND_OBJECT", ServiceState("Find_Object", xm_ObjectDetect,1, response_cb=self.find_object_cb), transitions={"succeeded":"BACK",'aborted':'FIND_OBJECT'})
            StateMachine.add("BACK", Forword_BacK(dis= -0.5), transitions={'succeeded':'CHANGE_2_ARM1'})
            StateMachine.add("CHANGE_2_ARM1",change2arm,transitions={'succeeded':"READY"})
            StateMachine.add("READY",ServiceState('xm_move_arm_position/move_position', xm_ArmPosition,"prepare",response_cb=self.responce_cb),transitions={'succeeded':'CHANGE_2_BASE1'})
            StateMachine.add("CHANGE_2_BASE1",change2base,transitions={"succeeded":"ALIGN_Y"})
            StateMachine.add("ALIGN_Y",Left_Right(dis=target.y-0),transitions= {"succeeded" :"CHANGE_2_ARM2"})
            StateMachine.add("CHANGE_2_ARM2",change2arm,transitions={'succeeded':"OPEN_GRIPPER"})
            StateMachine.add("OPEN_GRIPPER",gripper_state,transitions={'succeeded':'CHANGE_2_BASE2'})
            StateMachine.add("CHANGE_2_BASE2",change2base,transitions={'succeeded':""})
            
        outcome=grasp_sm.execute("FIND_OBJECT")
    

    pass

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        rospy.sleep(1)
        
    def find_object_cb(self,userdata,response):
        global target
        target=response.object.pos
        print response
        return "succeeded"
    
    def gripper_state_cb(self, userdata, status, result):
        if result:
            rospy.loginfo("Complete!")
            
    def arm_state_cb(self,userdata,status,result):
        
        pass
    def responce_cb(self,userdata,response):
        return 'succeeded'
if __name__ == '__main__':
#     try :
    object()
#     except:
#         rospy.loginfo("Operate and recognize object")
    
