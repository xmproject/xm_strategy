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
from math import sqrt,copysign,atan2
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


target=Point(0,0,0)

# class Forword_BacK(State):
#     def __init__(self,dis=0):
#         State.__init__(self, outcomes=['succeeded','preempted','aborted'],input_keys=["distance_in"])
#         rospy.on_shutdown(self.shutdown)
#         
#         self.rate = 50
#         self.r = rospy.Rate(self.rate)
#         self.speed = rospy.get_param('~speed',0.1)
#         self.tolerance = rospy.get_param('~tolerance', 0.01)
#         self.cmd_vel = rospy.Publisher('/mobile_base_controller/cmd_vel',Twist,queue_size=5)
#         self.base_frame = rospy.get_param('~base_frame', '/base_link')
#         self.odom_frame = rospy.get_param('~odom_frame', '/odom')
#         self.tf_listener = tf.TransformListener()
#         rospy.sleep(1)
#         self.tf_listener.waitForTransform(self.odom_frame,self.base_frame,rospy.Time(),rospy.Duration(60.0))
#         self.flag = rospy.get_param('~flag', True)
#         #tf get position
#         self.position = Point()
#         self.dis=dis
#         
#         #publish cmd_vel
#     def execute(self,userdata):
# #         print len(userdata.distance_in)
#         self.dis=userdata.distance_in
#         print self.dis
#         self.position = self.get_position()
#         print self.position
#         self.y_start = self.position.y
#         self.x_start = self.position.x
#         if self.dis!=0:
#             self.test_distance = self.dis
#         else:
#             print userdata.distance_in
#             self.test_distance = float(userdata.distance_in)
#             
#         while not rospy.is_shutdown():
#             move_cmd = Twist()
#             print self.test_distance
#             if self.flag:
#                 self.position = self.get_position()
#                 distance = float(sqrt(pow((self.position.x -self. x_start), 2) +
#                                 pow((self.position.y - self.y_start), 2)))
#                 if self.test_distance > 0:
#                     self.error = float(distance - self.test_distance)
#                 else:
#                     self.error = -float(distance + self.test_distance)
#                 if not self.flag or abs(self.error) < self.tolerance:
#                     self.flag = False
#                 else:
#                     move_cmd.linear.x = copysign(self.speed, -1 * self.error)
#             else:
#                 self.position = self.get_position()
#                 self.x_start = self.position.x
#                 self.y_start = self.position.y
#             self.cmd_vel.publish(move_cmd)
#             print move_cmd
#             if move_cmd.linear.x != 0.0:
#                 self.cmd_vel.publish(move_cmd)
#             else:
#                 break
#             self.r.sleep()
#         self.cmd_vel.publish(Twist())
#         return "succeeded"
#     def get_position(self):
#         try:
#             (trans,rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
#         except (tf.Exception, tf.ConnectivityException, tf.lookupException):
#             rospy.loginfo("TF Exception")
#             return
#         return Point(*trans)
#     def shutdown(self):
# #         rospy.loginfo("Stopping the robot...")
#         self.cmd_vel.publish(Twist())
#         rospy.sleep(1)
#     pass

class Left_Right(State):
    def __init__(self,dis=0.0):
        State.__init__(self, outcomes=['succeeded','preempted','aborted'],input_keys=['target'])
        rospy.on_shutdown(self.shutdown)
        self.test_distance = target.y-dis
        self.rate = 200
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
#         rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


class ChangeAngular(State):
    def __init__(self,angle=0):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'],input_keys=["angle_in"])
        self.speed = rospy.get_param('~speed', 0.10)
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
        
        self.tf_listener = tf.TransformListener()
        rospy.sleep(0.5)
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60))
    def execute(self,userdata):
        self.angle = userdata.angle_in
        print self.angle
        move_cmd = Twist()
        while not rospy.is_shutdown():
            move_cmd = Twist()
            self.odom_angle = self.get_odom_angle()
            last_angle = self.odom_angle
            turn_angle = 0
            angular_speed = self.speed
            while abs(float(turn_angle)) < abs(float(self.angle)):
                if rospy.is_shutdown():
                    return "succeeded"
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
                self.cmd_vel.publish(Twist())
            return "succeeded"
    def get_odom_angle(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return "succeeded"
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
#         rospy.loginfo("Stopping the robot...")
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
    
     
class Calculate_angle(State):
    def __init__(self):
        State.__init__(self, outcomes=["succeeded"], input_keys=["waypoint_in"], output_keys=["angle"])
        pass
     
    def execute(self, userdata):
        angle=atan2(userdata.waypoint_in.y,userdata.waypoint_in.x)
        return "succeeded"
         
class Lift(State):
    def __init__(self):
        State.__init__(self, outcomes=["succeeded","aborted","preempted"], input_keys=["position"],output_keys=["length_out"])
        self.lift=actionlib.SimpleActionClient('xm_move_arm_height', xm_ArmHeightAction)
        
    def execute(self, userdata):
        self.arm_goal = xm_ArmHeightGoal()
        self.arm_goal.distance_x = userdata.position.x
        self.arm_goal.distance_z = userdata.position.z-0.03
        self.lift.send_goal(self.arm_goal)
        self.lift.wait_for_result(rospy.Duration(20))
        result=self.lift.get_result()
        print result
#         userdata.length_out=result.distance
        return "succeeded"
class linearMove(object):
    def __init__(self):
        rospy.loginfo('the linear srv move init ok')

    def execute(self):
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('the distance you want to move is %s'%self.test_distance)
        self.test_distance = float(self.test_distance)
        self.rate = 100
        r = rospy.Rate(self.rate)
        self.speed = rospy.get_param('~speed',0.10)
        self.tolerance = rospy.get_param('~tolerance', 0.01)
        self.cmd_vel = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=5)
        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1)
        self.tf_listener.waitForTransform(self.odom_frame,self.base_frame,rospy.Time(),rospy.Duration(60.0))
        rospy.loginfo("start test!!!!")
        #define a bianliang
        self.flag = rospy.get_param('~flag', True)
        #tf get position
        self.position = Point()
        self.position = self.get_position()
        x_start = self.position.x
        y_start = self.position.y

        #publish cmd_vel
        move_cmd = Twist()
        while not rospy.is_shutdown():
            move_cmd = Twist()
            if self.flag:
                self.position = self.get_position()
                distance = float(math.sqrt(math.pow((self.position.x - x_start), 2) +
                                math.pow((self.position.y - y_start), 2)))
                if self.test_distance > 0:
                    error = float(distance - self.test_distance)
                else:
                    error = - float(distance + self.test_distance)
                if not self.flag or abs(error) < self.tolerance:
                    self.flag = False
                else:
                    move_cmd.linear.x = math.copysign(self.speed, -1 * error)
            else:
                self.position = self.get_position()
                x_start = self.position.x
                y_start = self.position.y
            if move_cmd.linear.x != 0:
                self.cmd_vel.publish(move_cmd)
            else:
                break
            r.sleep()
        self.cmd_vel.publish(Twist())
        
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

    def move(self,dis):
        rospy.loginfo('the distance you want to move is : %s'%dis)
        self.test_distance = dis
        self.execute()

class Forword_BacK(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'], input_keys=['distance_in'])
        self.__move_client = linearMove()
        rospy.loginfo("init ok!!")

    def execute(self, userdata):
        rospy.loginfo('the distance'
                      ' is init ok!!, please input your distance!!')
        goal_x = userdata.distance_in
        self.__move_client.move(dis=goal_x)
        return "succeeded"
    
class object():
    def __init__(self):
        global target
        rospy.init_node('object', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        grasp_sm=StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        grasp_sm.userdata.target=Point(0,0,0)
        grasp_sm.userdata.object=xm_Object
        
        gripper_goal = xm_GripperCommandGoal()
        gripper_goal.command.position = 0.15
#         gripper_goal.command.torque = 0.5
#         gripper_goal.header.frame
        gripper_open_state = SimpleActionState('xm_move_gripper', xm_GripperCommandAction, goal=gripper_goal,
                                          result_cb=self.gripper_state_cb, exec_timeout=rospy.Duration(10),
                                          server_wait_timeout=rospy.Duration(10))
#         
        gripper_goal = xm_GripperCommandGoal()
        gripper_goal.command.position = 0.06
#         gripper_goal.command.torque = 0.5
#         gripper_goal.header.frame
        gripper_close_state = SimpleActionState('xm_move_gripper', xm_GripperCommandAction, goal=gripper_goal,
                                          result_cb=self.gripper_state_cb, exec_timeout=rospy.Duration(10),
                                          server_wait_timeout=rospy.Duration(10))
        
        
        
        wrist_goal=xm_WristPositionGoal()
        wrist_goal.angle=-1.047
        wrist_state=SimpleActionState("xm_move_wrist", xm_WristPositionAction,goal=wrist_goal,
                                          result_cb=self.gripper_state_cb, exec_timeout=rospy.Duration(20),
                                          server_wait_timeout=rospy.Duration(20))
        wrist_goal=xm_WristPositionGoal()
        wrist_goal.angle=0
        wrist_state_init=SimpleActionState("xm_move_wrist", xm_WristPositionAction,goal=wrist_goal,
                                          result_cb=self.gripper_state_cb, exec_timeout=rospy.Duration(20),
                                          server_wait_timeout=rospy.Duration(20))
        change2base=ChangeMode("base")
        change2arm=ChangeMode("arm")
        rospy.loginfo("ready")
        grasp_sm.userdata.target=Point(0.779,-0.03,-0.144)
        grasp_sm.userdata.x1=float(0.6)
        grasp_sm.userdata.x2=-0.6
        grasp_sm.userdata.x3=0.3
#         grasp_sm.userdata.distance_in=0.5
        grasp_sm.userdata.angle_out=atan2(-0.03, 0.779)
        grasp_sm.userdata.postions=Point(0.779,-0.03,-0.144)
        with grasp_sm:
            StateMachine.add("CHANGE_2_ARM0",change2arm,transitions={'succeeded':"WRIST_DOWN"})
#             StateMachine.add("INIT",ServiceState('xm_move_arm_position/move_position', xm_ArmPosition,"init",response_cb=self.responce_cb),transitions={'succeeded':'WRIST_DOWN'})
            StateMachine.add("WRIST_DOWN",wrist_state_init,transitions={"succeeded":"CHANGE_2_BASE0"})
            StateMachine.add("CHANGE_2_BASE0",change2base,transitions={"succeeded":"CALCULATE_ANGLE"})
#             StateMachine.add("Move_to_Pos", Forword_BacK(), transitions={'succeeded':'CALCULATE_ANGLE'},remapping={"distance_in":'x1'})
#             StateMachine.add("FIND_OBJECT", ServiceState("Find_Object", xm_ObjectDetect,request_cb=self.find_object_request_cb, 
#                                                          response_cb=self.find_object_cb,input_keys=['object'],output_keys=['target']), 
#                                                          transitions={"succeeded":"CALCULATE_ANGLE",'aborted':'FIND_OBJECT'},
#                                                          remapping={'target':'labal'})
            StateMachine.add("CALCULATE_ANGLE",Calculate_angle(),transitions={"succeeded":"CHANGE_ANGLE"},remapping={'waypoint_in':'target','angle':'angle_out'})
            StateMachine.add("CHANGE_ANGLE",ChangeAngular(),transitions={"succeeded":"BACK"},remapping={"angle_in":'angle_out'})
            StateMachine.add("BACK", Forword_BacK(), transitions={'succeeded':'CHANGE_2_ARM1'},remapping={"distance_in":"x2"})
            StateMachine.add("CHANGE_2_ARM1",change2arm,transitions={'succeeded':"READY"})
            StateMachine.add("READY",ServiceState('xm_move_arm_position/move_position', xm_ArmPosition,"prepare",response_cb=self.responce_cb),transitions={'succeeded':'OPEN_GRIPER'})
            StateMachine.add("OPEN_GRIPER",gripper_open_state,transitions={"succeeded":"CHANGE_HEIGET"})
            StateMachine.add("CHANGE_HEIGET",Lift(),transitions={"succeeded":"CHANGE_2_BASE1"},remapping={"position":"target"})
            StateMachine.add("CHANGE_2_BASE1",change2base,transitions={"succeeded":"ALIGN_X"})
            StateMachine.add("ALIGN_X",Forword_BacK(),transitions= {"succeeded" :"CHANGE_2_ARM2"},remapping={"distance_in":"x3"})
            StateMachine.add("CHANGE_2_ARM2",change2arm,transitions={'succeeded':"CLOSE_GRIPPER"})
            StateMachine.add("CLOSE_GRIPPER",gripper_close_state,transitions={'succeeded':'WRIST_UP'})
            StateMachine.add("WRIST_UP",wrist_state,transitions={"succeeded":"CHANGE_2_BASE2"})
            StateMachine.add("CHANGE_2_BASE2",change2base,transitions={'succeeded':"BACK2"})
            StateMachine.add("BACK2", Forword_BacK(), transitions={'succeeded':''},remapping={"distance_in":'x2'})
            
        outcome=grasp_sm.execute("CALCULATE_ANGLE")
        rospy.sleep(rospy.Duration(3))

    pass

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        rospy.sleep(1)
    
    def find_object_request_cb(self,userdata):
        request=userdata.object
        return request
        
    def find_object_cb(self,userdata,response):
        userdata.target=response.object.pos
        print userdata.target
        return "succeeded"
    
    def gripper_state_cb(self, userdata, status, result):
        if result:
            rospy.loginfo("Complete!")
            
    def arm_state_cb(self,userdata,status,result):
        
        pass
    def responce_cb(self,userdata,response):
        if response.complete==True:
            return 'succeeded'
if __name__ == '__main__':
#     try :
    object()
#     except:
#         rospy.loginfo("Operate and recognize object")
    
