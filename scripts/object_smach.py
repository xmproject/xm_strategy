#!/usr/bin/env python 
# encoding:utf8

"""
author 
    
        X_Y耀  created on 2015-10-4        

"""
import rospy
import tf
import actionlib
import math
from math import sqrt, copysign, atan2
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from rospy.exceptions import ROSInterruptException
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from people_msgs.msg import PositionMeasurementArray, PositionMeasurement
from actionlib_msgs.msg import GoalStatus
from time import sleep
from std_msgs.msg import String, Int32
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

DIS_CHANGED = 0
ANGLE_CHANGED = 0
OBJECT_DIS = 0
ARM_LENGTH = 0
OBJECT_POS = Point(0, 0, 0)
AR_POS = Point(0, 0, 0)
OBJECT_WIDTH = 0
FLAG = 0
OBJECT_ID=0
OBJECT_NAME=""
ARM_REAL=0.85


class ChangeAngular(State):
    def __init__(self, angle=0):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=["angle_in"])
        self.speed = rospy.get_param('~speed', 0.03)
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
        self.angle=angle
        self.tf_listener = tf.TransformListener()
        rospy.sleep(0.5)
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60))

    def execute(self, userdata):
        global ANGLE_CHANGED
        ANGLE_CHANGED+=userdata.angle_in
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

    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def normalize_angle(self, angle):
        res = angle
        while res > math.pi:
            res -= 2.0 * math.pi
        while res < -math.pi:
            res += 2.0 * math.pi
        return res

    def shutdown(self):
        #         rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())


class ChangeAngularGlobal(State):
    def __init__(self, angle=0):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.speed = rospy.get_param('~speed', 0.03)
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
        self.angle=angle
        self.tf_listener = tf.TransformListener()
        rospy.sleep(0.5)
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60))

    def execute(self, userdata):
        global ANGLE_CHANGED
        self.angle =-0.5*ANGLE_CHANGED
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

    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def normalize_angle(self, angle):
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
    def __init__(self, msg=None):
        State.__init__(self, outcomes=['succeeded', "aborted", 'preempted'])
#         self.pub = rospy.Publisher('serial_switch_mode', xm_Serialswitchmode, queue_size=5)
        self.ser=rospy.ServiceProxy("serialswitchmode",xm_SerialSwitchMode)
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


class Calculate_angle(State):
    def __init__(self):
        State.__init__(self, outcomes=["succeeded"], output_keys=["angle"])
        pass

    def execute(self, userdata):
        global OBJECT_POS
        #test
        userdata.angle = atan2(OBJECT_POS.y, OBJECT_POS.x)*1.05
        rospy.sleep(1)
        return "succeeded"


class Lift_HEIGHT(State):
    def __init__(self, height=0):
        State.__init__(self, outcomes=["succeeded", "aborted", "preempted"])
        self.lift = actionlib.SimpleActionClient('xm_move_arm_height', xm_ArmHeightAction)
        self.height = height

    def execute(self, userdata):
        global ARM_REAL
        print self.height
        self.arm_goal = xm_ArmHeightGoal()
        self.arm_goal.height = self.height
        self.lift.send_goal(self.arm_goal)
        self.lift.wait_for_result(rospy.Duration(20))
        result=self.lift.get_result()
        ARM_REAL=result.distance
        print self.lift.get_result()
        return "succeeded"
    
class CLOSE_GRIPPER(State):
    def __init__(self):
        State.__init__(self, outcomes=["succeeded", "aborted", "preempted"])
        self.gripper = actionlib.SimpleActionClient('xm_move_gripper', xm_GripperCommandAction)
        
    def execute(self, userdata):
        global OBJECT_WIDTH
        print OBJECT_WIDTH
        gripper_goal = xm_GripperCommandGoal()
        gripper_goal.command.position = OBJECT_WIDTH/100
        self.gripper.send_goal(gripper_goal)
        self.gripper.wait_for_result(rospy.Duration(20))
        print self.gripper.get_result()
        return "succeeded"

class linearMove(object):
    def __init__(self):
        rospy.loginfo('the linear srv move init ok')

    def execute(self):
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('the distance you want to move is %s' % self.test_distance)
        self.test_distance = float(self.test_distance)
        self.rate = 100
        r = rospy.Rate(self.rate)
        self.speed = rospy.get_param('~speed', 0.05)
        self.tolerance = rospy.get_param('~tolerance', 0.01)
        self.cmd_vel = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=5)
        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1)
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
        rospy.loginfo("start test!!!!")
        # define a bianliang
        self.flag = rospy.get_param('~flag', True)
        # tf get position
        self.position = Point()
        self.position = self.get_position()
        x_start = self.position.x
        y_start = self.position.y

        # publish cmd_vel
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
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.lookupException):
            rospy.loginfo("TF Exception")
            return

        return Point(*trans)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def move(self, dis):
        rospy.loginfo('the distance you want to move is : %s' % dis)
        self.test_distance = dis
        self.execute()


class Forword_BacK(State):
    def __init__(self, dis=0):
        State.__init__(self, outcomes=['succeeded'])
        self.__move_client = linearMove()
        rospy.loginfo("init ok!!")
        self.dis = dis

    def execute(self, userdata):
        rospy.loginfo('the distance'
                      ' is init ok!!, please input your distance!!')
        goal_x = self.dis
        self.__move_client.move(dis=goal_x)
        return "succeeded"


class Forword_BacK_Global(State):
    def __init__(self, dis=0):
        State.__init__(self, outcomes=['succeeded'])
        self.__move_client = linearMove()
        rospy.loginfo("init ok!!")
        self.dis = dis

    def execute(self, userdata):
        global OBJECT_DIS
        if self.dis == 0:
            goal_x = OBJECT_DIS
        else:
            goal_x = 0 - OBJECT_DIS
        self.__move_client.move(dis=goal_x)
        return "succeeded"


class Calculate_length(State):
    def __init__(self):
        State.__init__(self, outcomes=["succeeded"])

    def execute(self, userdata):
        global OBJECT_POS, OBJECT_DIS, AR_POS
        print OBJECT_POS, AR_POS
        x = OBJECT_POS.x
        z = OBJECT_POS.z
        x1 = AR_POS.x
        z1 = AR_POS.z
        #test
        OBJECT_DIS = sqrt(x * x + z * z - 0.55 * 0.55) - sqrt(x1 * x1 + z1 * z1 - 0.55 * 0.55) + 0.5 - 0.17
        rospy.sleep(1)
        return "succeeded"

class Calculate_length_place(State):
    def __init__(self):
        State.__init__(self, outcomes=["succeeded"])

    def execute(self, userdata):
        global OBJECT_POS, OBJECT_DIS,ARM_REAL
        compensate = 0.865-ARM_REAL+0.1
        #test
        OBJECT_DIS = OBJECT_DIS+compensate
        rospy.sleep(1)
        return "succeeded"
    
class Find_Object(State):
    def __init__(self, msg=None):
        State.__init__(self, outcomes=['succeeded', "aborted", 'preempted'])
#         self.pub = rospy.Publisher('serial_switch_mode', xm_Serialswitchsode, queue_size=5)
        self.ser=rospy.ServiceProxy("Find_Object",xm_ObjectDetect)
#         self.msg = xm_SerialSwitchMode()
        self.msg = msg

    def execute(self, userdata):
        global OBJECT_ID,OBJECT_POS,OBJECT_WIDTH,OBJECT_NAME
        try:
            result=self.ser.call(OBJECT_ID)
            OBJECT_POS = result.object.pos
            OBJECT_NAME=result.object.name
            OBJECT_WIDTH=result.object.width
            print OBJECT_POS
            print result
            rospy.sleep(0.5)
            return "succeeded"
        except:
            print "Failed"
            return "aborted"
        pass
    
    
class Speak(State):
    def __init__(self,str="",mode=0):
        State.__init__(self, outcomes=["succeeded", "aborted", "preempted"])
        self.str=str
        self.mode=mode
        
    def execute(self, userdata):
        global OBJECT_NAME
        if self.mode==1:
            self.str="I find the "+OBJECT_NAME.data
        if self.mode==2:
            self.str="I am try to catch"+OBJECT_NAME.data
        else:
            subprocess.call(["espeak","-v","f3+en_us","-s","130",self.str])
        return "succeeded"
class main():
    def __init__(self):
        global DIS_CHANGED, ANGLE_CHANGED, OBJECT_DIS, ARM_LENGTH, OBJECT_POS, AR_POS,OBJECT_NAME,OBJECT_ID
        rospy.init_node("object", anonymous=False)
        rospy.on_shutdown(self.shutdown)
        id_list=[3,1,4]
        self.monitor_times = 0
        change2base = ChangeMode("base")
        change2arm = ChangeMode("arm")

        wrist_goal = xm_WristPositionGoal()
        wrist_goal.angle = 0.785
        wrist_up_state = SimpleActionState("xm_move_wrist", xm_WristPositionAction, goal=wrist_goal,
                                           result_cb=self.gripper_state_cb, exec_timeout=rospy.Duration(20),
                                           server_wait_timeout=rospy.Duration(20))

        wrist_goal = xm_WristPositionGoal()
        wrist_goal.angle = 0
        wrist_down_state = SimpleActionState("xm_move_wrist", xm_WristPositionAction, goal=wrist_goal,
                                             result_cb=self.gripper_state_cb, exec_timeout=rospy.Duration(20),
                                             server_wait_timeout=rospy.Duration(20))

        gripper_goal = xm_GripperCommandGoal()
        gripper_goal.command.position = 0.15
        gripper_open_state = SimpleActionState('xm_move_gripper', xm_GripperCommandAction, goal=gripper_goal,
                                               result_cb=self.gripper_state_cb, exec_timeout=rospy.Duration(10),
                                               server_wait_timeout=rospy.Duration(10))
        #
        gripper_goal = xm_GripperCommandGoal()
        gripper_goal.command.position = 0.05
        gripper_close_state = SimpleActionState('xm_move_gripper', xm_GripperCommandAction, goal=gripper_goal,
                                                result_cb=self.gripper_state_cb, exec_timeout=rospy.Duration(10),
                                                server_wait_timeout=rospy.Duration(10))

        init_sm = StateMachine(outcomes=["succeeded", "aborted", "preempted","valid","invalid"])

        with init_sm:
            StateMachine.add("CHANGE_2_ARM0", change2arm, transitions={'succeeded': "INIT","aborted":"CHANGE_2_ARM0"})
            StateMachine.add("INIT", ServiceState('xm_move_arm_position/move_position', xm_ArmPosition, "init",
                                                  response_cb=self.responce_cb),
                             transitions={'succeeded': 'WRIST_DOWN0'})
            StateMachine.add("WRIST_DOWN0", wrist_down_state, transitions={"succeeded": "CLOSE_GRIPPER0"})
            StateMachine.add("CLOSE_GRIPPER0", gripper_close_state, transitions={'succeeded': 'CHANGE_2_BASE0'})
            StateMachine.add("CHANGE_2_BASE0", change2base, transitions={"succeeded": "SPEAK_READY","aborted":"CHANGE_2_BASE0"})
#             StateMachine.add("WAIT_4_START", MonitorState("task_comming", xm_Task, self.start_cb),
#                              transitions={'invalid': 'SPEAK_READY',
#                                           'valid': 'WAIT_4_START',
#                                           'preempted': ''})
            StateMachine.add("SPEAK_READY",Speak(str="I am ready,,test start"),transitions={'succeeded': ''})
        init_sm_again=StateMachine(outcomes=["succeeded", "aborted", "preempted"])
        with init_sm_again:
            StateMachine.add("CHANGE_2_ARM0", change2arm, transitions={'succeeded': "INIT","aborted":"CHANGE_2_ARM0"})
            StateMachine.add("INIT", ServiceState('xm_move_arm_position/move_position', xm_ArmPosition, "init",
                                                  response_cb=self.responce_cb),
                             transitions={'succeeded': 'WRIST_DOWN0'})
            StateMachine.add("WRIST_DOWN0", wrist_down_state, transitions={"succeeded": "CLOSE_GRIPPER0"})
            StateMachine.add("CLOSE_GRIPPER0", gripper_close_state, transitions={'succeeded': 'CHANGE_2_BASE0'})
            StateMachine.add("CHANGE_2_BASE0", change2base, transitions={"succeeded": "SPEAK_READY","aborted":"CHANGE_2_BASE0"})
            StateMachine.add("SPEAK_READY",Speak(str=" test start again"),transitions={'succeeded': ''})
            
        find_object_sm = StateMachine(outcomes=["succeeded", "aborted", "preempted"])
        find_object_sm.userdata.object = 1
        with find_object_sm:
            StateMachine.add("NAV_2_START_POS", Forword_BacK(dis=0.5), transitions={"succeeded": "FIND_OBJECT1"})
            StateMachine.add("FIND_OBJECT1", Find_Object(),
                             transitions={"succeeded": "CALCULATE_ANGLE1", 'aborted': 'FIND_OBJECT1'}, )
            StateMachine.add("CALCULATE_ANGLE1", Calculate_angle(), transitions={"succeeded": "CHANGE_ANGLE1"},
                             remapping={'angle': 'angle_out'})
            StateMachine.add("CHANGE_ANGLE1", ChangeAngular(), transitions={"succeeded": "FIND_OBJECT2"},
                             remapping={"angle_in": 'angle_out'})
            StateMachine.add("FIND_OBJECT2", Find_Object(),
                             transitions={"succeeded": "SPEAK_OBJECT", 'aborted': 'FIND_OBJECT2'}, )
            StateMachine.add("SPEAK_OBJECT",Speak(mode=1),transitions={'succeeded': 'CALCULATE_ANGLE2'})
            StateMachine.add("CALCULATE_ANGLE2", Calculate_angle(), transitions={"succeeded": "CHANGE_ANGLE2"},
                             remapping={'angle': 'angle_out'})
            StateMachine.add("CHANGE_ANGLE2", ChangeAngular(), transitions={"succeeded": ""},
                             remapping={"angle_in": 'angle_out'})

        grasp_ready_sm = StateMachine(outcomes=["succeeded", "aborted", "preempted"])
        with grasp_ready_sm:
            StateMachine.add("SPEAK_GRASP",Speak(mode=2),transitions={'succeeded': 'BACK'})
            StateMachine.add("BACK", Forword_BacK(dis=-0.5), transitions={'succeeded': 'CHANGE_2_ARM1'})
            StateMachine.add("CHANGE_2_ARM1", change2arm, transitions={'succeeded': "READY","aborted":"CHANGE_2_ARM1"})
            StateMachine.add("READY", ServiceState('xm_move_arm_position/move_position', xm_ArmPosition, "prepare",
                                                   response_cb=self.responce_cb),
                             transitions={'succeeded': 'OPEN_GRIPER'})
            
            StateMachine.add("OPEN_GRIPER", gripper_open_state, transitions={"succeeded": "CHANGE_HEIGHT1"})
            StateMachine.add("CHANGE_HEIGHT1", Lift_HEIGHT(height=-0.06), transitions={"succeeded": "AR_TARGET"})
            StateMachine.add("AR_TARGET", MonitorState("ar_pose_marker", AlvarMarkers, self.get_mark_pos_cb),
                             transitions={"valid": "AR_TARGET", "invalid": "CHANGE_2_BASE1"})
            StateMachine.add("CHANGE_2_BASE1", change2base, transitions={"succeeded": "CALCULATE_DIS","aborted":"CHANGE_2_BASE1"})
            StateMachine.add("CALCULATE_DIS", Calculate_length(), transitions={'succeeded': ''})

        grasp_sm = StateMachine(outcomes=["succeeded", "aborted", "preempted"])
        with grasp_sm:
            
            StateMachine.add("FORWORD", Forword_BacK_Global(), transitions={"succeeded": "CLOSE_GRIPPER"})
            StateMachine.add("CLOSE_GRIPPER", CLOSE_GRIPPER(), transitions={'succeeded': 'WRIST_UP'})
            StateMachine.add("WRIST_UP", wrist_up_state, transitions={"succeeded": "CHANGE_2_BASE2"})
            StateMachine.add("CHANGE_2_BASE2", change2base, transitions={'succeeded': "BACK2","aborted":"CHANGE_2_BASE2"})
            StateMachine.add("BACK2", Forword_BacK_Global(dis=1), transitions={'succeeded': ''})

        place_sm = StateMachine(outcomes=["succeeded", "aborted", "preempted"])
        with place_sm:
            StateMachine.add("SPEAK_PLACE",Speak(str='I am try to place it'),transitions={'succeeded': 'CHANGE_2_ARM1'})
            StateMachine.add("CHANGE_2_ARM1", change2arm, transitions={'succeeded': "CHANGE_HEIGHT","aborted":"CHANGE_2_ARM1"})
            StateMachine.add("CHANGE_HEIGHT", Lift_HEIGHT(height=0.38), transitions={"succeeded": "CHANGE_2_BASE2"})
            StateMachine.add("CHANGE_2_BASE2", change2base, transitions={"succeeded": "CALCULATE_DIS2","aborted":"CHANGE_2_BASE2"})
#             StateMachine.add("AR_TARGET2", MonitorState("ar_pose_marker", AlvarMarkers, self.get_mark_pos_cb),
#                              transitions={"valid": "AR_TARGET2", "invalid": "CALCULATE_DIS2"})
            StateMachine.add("CALCULATE_DIS2", Calculate_length_place(), transitions={"succeeded": "FORWORD2"})
            StateMachine.add("FORWORD2", Forword_BacK_Global(), transitions={"succeeded": "CHANGE_2_ARM2"})
            StateMachine.add("CHANGE_2_ARM2", change2arm, transitions={'succeeded': "WRIST_DOWN","aborted":"CHANGE_2_ARM2"})
            StateMachine.add("WRIST_DOWN", wrist_down_state, transitions={"succeeded": "GRIPER_OPEN"})
            StateMachine.add("GRIPER_OPEN", gripper_open_state, transitions={"succeeded": "CHANGE_2_BASE3"})
            StateMachine.add("CHANGE_2_BASE3", change2base, transitions={"succeeded": "","aborted":"CHANGE_2_BASE3"})

        reinit_sm = StateMachine(outcomes=["succeeded", "aborted", "preempted"])
        with reinit_sm:
            StateMachine.add("BACK3", Forword_BacK_Global(dis=1), transitions={"succeeded": "INIT_ANGLE"})
            StateMachine.add("INIT_ANGLE", ChangeAngularGlobal(), transitions={"succeeded": ""})

        sm = StateMachine(outcomes=["succeeded", "aborted", "preempted"])
        sm.userdata.object_id=0
        with sm:
            StateMachine.add("FIND_OBJECT", find_object_sm, transitions={"succeeded": "READY"},remapping={"object":"object_id "})
            StateMachine.add("READY", grasp_ready_sm, transitions={"succeeded": "GRASP"})
            StateMachine.add("GRASP", grasp_sm, transitions={"succeeded": "PLACE"})
            StateMachine.add("PLACE", place_sm, transitions={"succeeded": "REINIT"})
            StateMachine.add("REINIT", reinit_sm, transitions={"succeeded": "INIT"})
            StateMachine.add("INIT", init_sm_again, transitions={"succeeded": ""})
        output=init_sm.execute()
        rospy.sleep(1)
        for id in id_list:
            OBJECT_ID=id
            output = sm.execute()
            ANGLE_CHANGED = 0
            rospy.sleep(1)
        rospy.sleep(rospy.Duration(3))

    def get_mark_pos_cb(self, userdata, msg):
        global AR_POS
        if self.monitor_times >= 15:
            self.monitor_times = 0
            return True
        if len(msg.markers) == 0:
            print msg.markers
            self.monitor_times += 1
            print "aaa"
            return True
        print msg
        AR_POS = msg.markers[0].pose.pose.position
        print AR_POS
        self.monitor_times = 0
        return False

    def start_cb(self, userdata, msg):
        if msg.task == 0x01:
            return False
        else:
            return True

    def object_request_cb(self, userdata, request):
        global OBJECT_ID
        request = OBJECT_ID
        return request

    def find_object_cb(self, userdata, response):
        global OBJECT_POS
        if response.object.pos is None:
            return "aborted"
        OBJECT_POS = response.object.pos
        OBJECT_NAME=response.object.name
        OBJECT_WIDTH=response.object.width
        print OBJECT_POS
        return "succeeded"

    def responce_cb(self, userdata, response):
        if response.complete == True:
            return 'succeeded'

    def gripper_state_cb(self, userdata, status, result):
        if result:
            rospy.loginfo("Complete!")

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)

    pass


if __name__ == '__main__':
    main()
