#!/usr/bin/env python 
#encoding:utf8 

"""
author 
       Luke Liao
       
        X_Y耀  created by 2015-4-30

"""
import rospy
import tf
import actionlib
import math
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from rospy.exceptions import ROSInterruptException
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from people_msgs.msg import PositionMeasurementArray,PositionMeasurement
from actionlib_msgs.msg import GoalStatus
from time import sleep
from std_msgs.msg import String,Int16
import threading
from smach.concurrence import Concurrence
from bson.json_util import default
from lxml import get_include
from roslib.manifestlib import VALID
from std_srvs.srv._Empty import EmptyRequest


current_people_pose = Point(0,0,0)
updata_flag=0
waring_scope=9
# REMEMBER='0x01'
# FOLLOW='0x02'
# GET_IN='0X03'
# GET_OUT='0X04'
# WAIT='0X05'
REMEMBER=0x01
FOLLOW=0x02
GET_IN=0x03
GET_OUT=0x04
WAIT=0x05
# start_time = rospy.get_time()

class Nav2Waypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.pub = rospy.Publisher('speech', String)
        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))    
        
        rospy.loginfo("Connected to move_base action server")

    def execute(self, userdata):
        global current_people_pose, start_time,updata_flag,waring_scope,WAIT
        self.move_base.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
#         self.kinect_client.wait_for_service()
        lock = threading.Lock()
        update=-1
        self.flag=0
        while 1:
            with lock:
                if not(update == updata_flag):
                    self.flag=1
                    x=current_people_pose.x
                    y=current_people_pose.y
            if self.flag==1:
                theta = math.atan2(y, x)
                q_angle = quaternion_from_euler(0, 0,0)
                self.q=Quaternion(*q_angle)
                goal.target_pose.pose=(Pose(Point(x-0.2,y,0),self.q))
                #rospy.ServiceProxy('clear_costmaps',Empty)
#                 rospy.ServiceProxy('move_base/clear_costmaps',EmptyRequest)
                self.move_base.send_goal(goal)
                update= updata_flag
                self.flag=0
                if(x*x+y*y > waring_scope):
                    self.pub.publish(WAIT)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
#         
#                 # Allow 1 minute to get there
#             finished_within_time = self.move_base.wait_for_result(rospy.Duration(30)) 
#         
#                 # If we don't get there in time, abort the goal
#             if not finished_within_time:
#                 self.move_base.cancel_goal()
#                 rospy.loginfo("Timed out achieving goal")
#                 return 'aborted'  
# #             else:
# #                     # We made it!
# #                     state = self.move_base.get_state()
# #                     if state == GoalStatus.SUCCEEDED:
# #                         rospy.loginfo("Goal succeeded!")
# #                         return 'succeeded'
class Get_in(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])            
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    def execute(self, userdata):
        global current_people_pose
        self.move_base.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        lock = threading.Lock()
        update=-1
        self.flag=0       
        while 1:
            with lock:
                if not(update == updata_flag):
                    self.flag=1
                    x=current_people_pose.x
                    y=current_people_pose.y
            if self.flag==1:
                theta = math.atan2(x, y)
                q_angle = quaternion_from_euler(0, 0,theta, axes='sxyz')
                self.q=Quaternion(*q_angle)
                goal.target_pose.pose=(Pose(Point(x-1,y,0),self.q))
                #rospy.ServiceProxy('clear_costmaps',Empty)
                self.move_base.send_goal(goal)
                update= updata_flag
                self.flag=0
                if(x*x+y*y > waring_scope):
                    self.pub.publish(WAIT)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
         
class Get_out(State):         
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])            
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server()
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'base_link'
    def execute(self, userdata):
        q_angle = quaternion_from_euler(0, 0,0, axes='sxyz')
        self.q=Quaternion(*q_angle)
        self.goal.target_pose.pose=(Pose(Point(-2,0,0),self.q))
        #rospy.ServiceProxy('clear_costmaps',Empty)
        self.move_base.send_goal(self.goal)
        self.move_base.wait_for_result(rospy.Duration(30)) 
        self.goal.target_pose.pose=(Pose(Point(-0.5,0,0),self.q))
        #rospy.ServiceProxy('clear_costmaps',Empty)
        self.move_base.send_goal(self.goal)
        self.move_base.wait_for_result(rospy.Duration(30))         
        return 'succeeded'
class main():
    def __init__(self):
        rospy.init_node("follow")
        rospy.on_shutdown(self.shutdown)
        self.state = None
        
        self.sm_follow=StateMachine(outcomes=['succeeded','aborted','preempted'])
#    总状态机follow
        with self.sm_follow: 
            self.co_follow=Concurrence(outcomes=['succeeded','preempted','aborted'],
                                default_outcome='succeeded', 
#                              outcome_cb = self.co_follow_outcome_cb ,
#                                   child_termination_cb=self.co_follow_child_cb
                                    )
            with self.co_follow:
                Concurrence.add('mo_L',MonitorState('people_position_estimation',PositionMeasurement,
                                              self.pos_M_cb))
                
                self.sm_nav=StateMachine(outcomes=('succeeded','aborted','preempted'))
                with self.sm_nav:
#                    StateMachine.add('wait',MonitorState('sr_data',Int16,self.wait_cb),transitions={'valid':'wait','invalid':'nav_speech','preempted':'nav_speech'})
                    
                    self.speech_nav=Concurrence(outcomes=['get_in','succeeded'],
                                               default_outcome='succeeded',
                                                outcome_cb=self.speech_nav_outcome_cb,
                                                child_termination_cb=self.speech_nav_child_termination_cb
                                                )
                    with self.speech_nav:
                        Concurrence.add('speech',MonitorState('sr_data',Int16,self.nav_speech_cb))
                        Concurrence.add('nav', Nav2Waypoint())
                    StateMachine.add('nav_speech',self.speech_nav,transitions={'get_in':'Get_in',})
                    
                    self.get_in_speech=Concurrence(outcomes=['get_out','succeeded'],
                                                default_outcome='succeeded',
                                                outcome_cb=self.speech_get_in_outcome_cb,
                                                child_termination_cb=self.speech_get_in_child_termination_cb
                                                   
                                                   )
                    with self.get_in_speech:
                        Concurrence.add('speech',MonitorState('sr_data',Int16,self.get_in_speech_cb))
                        Concurrence.add('get_in',Get_in())
                    StateMachine.add('Get_in',self.get_in_speech,transitions={'get_out':'Get_out'})
                    StateMachine.add('Get_out',Get_out(), transitions={'succeeded':'nav_speech','preempted':'','aborted':'Get_out' })
                Concurrence.add('Nav',self.sm_nav)
            StateMachine.add('Nav_top',self.co_follow)
            
        a=self.sm_follow.execute()
        
    def pos_M_cb(self,userdata,msg):
        global current_people_pose,updata_flag
#         lock= threading.Lock()
#         with lock:
        if msg.pos is not None:
                current_people_pose=msg.pos
                updata_flag+=1
                return True
            
    def wait_cb(self,userdata,msg):
        global FOLLOW
        if(msg.data==FOLLOW):
            return False
        else:
            return True
    def nav_speech_cb(self,userdata,msg):
        global GET_IN,GET_OUT
        if msg.data==GET_IN:
            self.state='get_in'
            return False
        else:
            return True
    def get_in_speech_cb(self,userdata,msg):
        global GET_IN,GET_OUT
        if msg.data==GET_OUT:
            self.state='get_out'
            return False
        else:
            return True    
    def speech_nav_outcome_cb(self,outcome_map):
        if outcome_map['speech']=='invalid':
            if self.state=='get_in':
                return 'get_in'
        
    def speech_nav_child_termination_cb(self,outcome_map):
        if outcome_map['speech']=='invalid':
            return True
        
    def speech_get_in_outcome_cb(self,outcome_map):
        if outcome_map['speech']=='invalid':
            if self.state =='get_out':
                return 'get_out'
        
    def speech_get_in_child_termination_cb(self,outcome_map):
        if outcome_map['speech']=='invalid':
            return True
                 
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        self.sm_follow.request_preempt()
        
        rospy.sleep(1)
    pass    
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("follow")
