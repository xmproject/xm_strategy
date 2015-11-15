#!/usr/bin/env python 
#encoding:utf8 

"""
author 
    
        X_Y耀  created on 2015-9-21

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
from people_msgs.msg import PositionMeasurementArray, PositionMeasurement
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
from opt_msgs.msg import *

current_people_pose = Point(0,0,0)
updata_flag=0
waring_scope=9

class Nav2Waypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))    
        
        rospy.loginfo("Connected to move_base action server")

    def execute(self, userdata):
        global current_people_pose,updata_flag,waring_scope
        self.move_base.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
#         self.kinect_client.wait_for_service()
        lock = threading.Lock()
        update=0
        self.flag=0
        q_angle = quaternion_from_euler(0, 0,0)
        self.q=Quaternion(*q_angle)
        while 1:
            with lock:
                if updata_flag>=1:
                    self.flag=1
                    x=current_people_pose.x
                    y=current_people_pose.y
                    updata_flag=0
            if self.flag==1:
                goal.target_pose.pose=(Pose(Point(x-0.8,y,0),self.q))
#                 rospy.ServiceProxy('clear_costmaps',Empty)
                
#                 subprocess.call(["rosservice","call","/move_base/clear_costmaps"])
                self.move_base.send_goal(goal)
                self.move_base.wait_for_result(rospy.Duration(60))
#                 rospy.ServiceProxy('move_base/clear_costmaps',Empty)
#                 rospy.sleep(0.5)
                self.flag=0
#                 if(x*x+y*y > waring_scope):
#                     self.pub.publish(WAIT)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

class wait(State):
    def __init__(self,time=5,people_pass=False):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.speak_pub=rospy.Publisher("tts_data", String, queue_size=100)
        self.time=time
        self.speak=people_pass
    def execute(self,userdata):
        rospy.sleep(self.time)
        if self.speak:
            self.speak_pub.publish("Hey man,please let me through")
#         rospy.ServiceProxy(name, service_class, persistent, headers)
        return 'succeeded'
        pass
    
class navigation():
    def __init__(self):
        rospy.init_node("navgation_test")
        rospy.on_shutdown(self.shutdown)
        self.waypoints=[];
        Location= (Point(3.099,-1.796,0),
                            Point(6.732,-3.260,0),
                            Point(7.058,-0.119,0),
                            Point(-0.595,0.069,0),)
        quaternions=[];
        euler_angles=[0.039,0.056,-3.093,-3.133];

        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)
        for i in range(4):
            self.waypoints.append(Pose(Location[i], quaternions[i]))
            
        point_locations = (('Point1', self.waypoints[0]),
                      ('Point2', self.waypoints[1]),
                      ('Point3', self.waypoints[2]),
                      ('Point4', self.waypoints[3]))
        
        self.room_locations = OrderedDict(point_locations)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
        rospy.loginfo("Waiting for move_base action server...")
    
        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))    
        
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    
        rospy.loginfo("Connected to move_base action server")
    
        nav_states={}
        self.people_in_sight=0
        # Create a navigation task for each room
        for room in self.room_locations.iterkeys():
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = 'map'
            nav_goal.target_pose.header.stamp = rospy.Time.now()
            nav_goal.target_pose.pose = self.room_locations[room]
            move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb, 
                                                exec_timeout=rospy.Duration(100.0),
                                                server_wait_timeout=rospy.Duration(100.0))
            nav_states[room] = move_base_state
        
        sm_nav1= StateMachine(outcomes=['succeeded','aborted','preempted',"valid","invalid"])
        with sm_nav1:
            StateMachine.add('START',MonitorState("task_comming", xm_Task, self.start_cb),
                                                  transitions={'invalid':'WAIT4DOOR_OPEN',
                                                               'valid':'START',
                                                               'preempted':'WAIT4DOOR_OPEN'})
            
            StateMachine.add('WAIT4DOOR_OPEN',MonitorState("doormsg",door_msg,self.door_is_open_cb),
                                                  transitions={'invalid':'INIT',
                                                               'valid':'WAIT4DOOR_OPEN',
                                                               'preempted':'INIT'})
            StateMachine.add("INIT", ServiceState("pan_srv", xm_Pan,True, response_cb=self.init_cb),transitions={"succeeded":"NAV_1","aborted":"NAV_1"})
            StateMachine.add("NAV_1",nav_states['Point1'],transitions={'succeeded':'','aborted':'','preempted':''})
        sm_nav2=StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_nav2:
            StateMachine.add("NAV_2",nav_states['Point2'],transitions={'succeeded':'','aborted':'Check_People','preempted':''})
            StateMachine.add("Check_People",MonitorState("tracker/tracks_smoothed",TrackArray,self.check_people_cb),
                                                  transitions={'valid':'WAIT',
                                                               'invalid':'WAIT',
                                                               'preempted':'WAIT'})
            StateMachine.add('WAIT',wait(time=5,people_pass=self.people_in_sight),
                                                  transitions={'succeeded':'NAV_2_END'})
            StateMachine.add("NAV_2_END",nav_states['Point2'],transitions={'succeeded':'','aborted':'','preempted':''})
        sm_nav3=StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_nav3:
#             StateMachine.add("NAV_3",nav_states['Point3'],transitions={'succeeded':'WAIT_4_CMD','aborted':'NAV_3','preempted':'NAV_3'})
            sm_follow=Concurrence(outcomes=['succeeded', 'aborted'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.concurrence_child_termination_cb,
                                        outcome_cb=self.concurrence_outcome_cb)
            with sm_follow:
                Concurrence.add('START_FOLLOW',MonitorState("task_comming", xm_Task, self.follow_stop_cb))
                co_follow=Concurrence(outcomes=['succeeded','aborted'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.co_follow_cb,
                                        outcome_cb=self.co_follow_outcome_cb)
                with co_follow:
                    Concurrence.add('mo_L',MonitorState('people_tf_position',xm_Tracker,
                                              self.pos_M_cb))
                    Concurrence.add('nav', Nav2Waypoint())
                Concurrence.add('FOLLOW',co_follow)
            StateMachine.add('WAIT_4_CMD',MonitorState("task_comming", xm_Task, self.follow_start_cb),
                                                  transitions={'invalid':'FOLLOW_MODE',
                                                               'valid':'FOLLOW_MODE',
                                                               'preempted':'FOLLOW_MODE'})
            StateMachine.add("FOLLOW_MODE",sm_follow,transitions={'succeeded':'','aborted':''})    
            
        sm_nav4=StateMachine(outcomes=['succeeded','aborted','preempted',"valid","invalid"])
        with sm_nav4:
            StateMachine.add("WAIT_4_BACK",MonitorState("task_comming",xm_Task,self.cmd_back_cb),transitions={"invalid":"NAV_4","valid":"WAIT_4_BACK","preempted":""})
            StateMachine.add("NAV_4",nav_states['Point4'],transitions={'succeeded':'','aborted':'','preempted':''})
        
        sm_nav_test=StateMachine(outcomes=['succeeded','aborted','preempted',"valid","invalid"])
        
        with sm_nav_test:
#             StateMachine.add("NAV_1",sm_nav1,transitions={'succeeded':'NAV_2','aborted':'NAV_2','preempted':'NAV_2'})
#             StateMachine.add("NAV_2",sm_nav2,transitions={'succeeded':'NAV_3','aborted':'NAV_3','preempted':'NAV_3'})
            StateMachine.add("NAV_3",sm_nav3,transitions={'succeeded':'NAV_4','aborted':'NAV_4','preempted':'NAV_4'})
            StateMachine.add("NAV_4",sm_nav4,transitions={'succeeded':'','aborted':'','preempted':''})
        
        sm_outcome = sm_nav_test.execute()
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        rospy.sleep(1)
    pass
    def cmd_back_cb(self,userdata,msg):
        if msg.task==0x04:
            pub=rospy.Publisher("tts_data", String, queue_size=100)
            pub.publish("OK,I am going back. Bye~")
            return False
        else:
            return True
    def init_cb(self,userdata,result): 
        return "succeeded"
    def move_base_result_cb(self, userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            pass
    
    def start_cb(self,userdata,msg):
        if msg.task==0x01 :
            pub=rospy.Publisher("tts_data", String, queue_size=100)
            pub.publish("yes, my lord")
            return False
        else:
            return True
        
    def follow_start_cb(self,userdata,msg):
        if msg.task==0x02  :
#             pub=rospy.Publisher("tts_data", String, queue_size=100)
#             pub.publish("Got it ,I will try my best to follow you")
            task=xm_Task()
            task.status=0x00
            task.place=0x00
            task.task=0x05
            pub=rospy.Publisher("task_comming", xm_Task,task)
            return False
        else:
            return True
        
    def follow_stop_cb(self,userdata,msg):
        if msg.task==0x03 :
            pub=rospy.Publisher("tts_data", String, queue_size=100)
            pub.publish("OK,I am stopping")
            return False
        else:
            return True
        
    def door_is_open_cb(self,userdata,msg):
        if msg.door_open_msg==False:
            return True
        elif msg.door_open_msg==True:
            rospy.sleep(3)
            subprocess.call(["rosservice","call","/move_base/clear_costmaps"])
            rospy.sleep(1)
            return False
        else :
            return True         
    # Gets called when ANY child state terminates
    def concurrence_child_termination_cb(self, outcome_map):
        # If the current navigation task has succeeded, return True
        if outcome_map['START_FOLLOW'] == 'valid':
            return False
        else:
            return True
    
    # Gets called when ALL child states are terminated
    def concurrence_outcome_cb(self, outcome_map):
        # If the battery is below threshold, return the 'recharge' outcome
        if outcome_map['START_FOLLOW'] == 'invalid':
            return 'succeeded'
        else :
            return 'aborted'
    def co_follow_cb(self,outcome_map):
        if outcome_map['nav']=='preempted':
            return True
        pass
    
    def co_follow_outcome_cb(self,outcome_map):
        if outcome_map['nav']=='preempted':
            return 'succeeded'        
        pass
    
    def pos_M_cb(self,userdata,msg):
        global current_people_pose,updata_flag
#         lock= threading.Lock()
#         with lock:
        if msg.pos is not None:
            print msg.pos
            current_people_pose=msg.pos
            updata_flag+=1
            return True
    def check_people_cb(self,userdata,msg):
        if msg.tracks is not None:
            self.people_in_sight=True
            return False
        else:
            return True
            
if __name__ == '__main__':
    try:
        navigation()
    except KeyboardInterrupt:
        pass
