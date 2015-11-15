#!/usr/bin/env python 
#encoding:utf8 

"""
author 
    
        X_Y耀  created on 2015-8-15

"""
import rospy
import actionlib
from actionlib import GoalStatus
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist,Point,Quaternion,Pose
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal,MoveBaseActionFeedback
from pi_trees_ros.pi_trees_ros import *
from tf.transformations import quaternion_from_euler
from collections import OrderedDict
from math import pi, sqrt
import time
import easygui
from roscpp.srv._Empty import Empty
import std_srvs
from people_msgs.msg import PositionMeasurementArray,PositionMeasurement
import subprocess
from time import sleep
from rospy.topics import Subscriber
from Doorisopen.msg import *
import threading

PEOPLE_POSITION=Point(0,0,0)
FLAG=0

class IgnoreSuccess(Task):
    """
        Always return either RUNNING or SUCCESS.
    """
    def __init__(self, name, *args, **kwargs):
        super(IgnoreSuccess, self).__init__(name, *args, **kwargs)
 
    def run(self):
        
        for c in self.children:
            
            c.status = c.run()
            
            if c.status == TaskStatus.SUCCESS:
                return TaskStatus.FAILURE
            else:
                return c.status

        return TaskStatus.FAILURE
    
class Wait(Task):
    """
        Always return either RUNNING or SUCCESS.
    """
    def __init__(self, name,time=5 ,*args, **kwargs):
        super(Wait, self).__init__(name, *args, **kwargs)
        self.name=name
        self.time=time
    def run(self):
        sleep(time)
        return TaskStatus.SUCCESS

class Follow(Task):
    def __init__(self, name,*args, **kwargs):
        global PEOPLE_POSITION,FLAG
        super(Follow, self).__init__(name, *args, **kwargs)
        self.name=name
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.cmd_vel_msg = Twist()
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))  
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'base_link'
        q_angle = quaternion_from_euler(0, 0,0)
        self.q=Quaternion(*q_angle)
        rospy.loginfo("start follow")
        
    def run(self):
        global FLAG,PEOPLE_POSITION
        lock = threading.Lock()
#         while True:
        if FLAG!=0:
                with lock:
                    rospy.loginfo(PEOPLE_POSITION)
                    x=PEOPLE_POSITION.x-0.7
                    y=PEOPLE_POSITION.y
                self.goal.target_pose.pose=(Pose(Point(x,y,0),self.q))
                subprocess.call(["rosservice","call","/move_base/clear_costmaps"])
                self.move_base.send_goal(self.goal)
                FLAG=0
                rospy.sleep(0.1)
        return TaskStatus.RUNNING
    
class Speak(Task):
    """
        Always return either RUNNING or SUCCESS.
    """
    def __init__(self, name,word,time=5 ,*args, **kwargs):
        super(Speak, self).__init__(name, *args, **kwargs)
        self.name=name
        self.time=time
        self.word=word
    def run(self):
        subprocess.call(["espeak",self.word])
        return TaskStatus.SUCCESS
    

class navgation():
    def __init__(self):
        rospy.init_node('navgation', anonymous=False)
        
        self.waypoints=[];
        Location= (Point(2.227,1.576,0),
                            Point(0.514,-4.479,0),
                            Point(-0.415,-1.671,0),
                            Point(1.776,0.723,0),)
        quaternions=[];
        euler_angles=[1.509,1.920,1.691,1.262];

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
    
    # Create a dictionary to hold navigation tasks for getting to each room
        rospy.on_shutdown(self.shutdown)
        MOVE_BASE = {}

        # Create a navigation task for each room
        for room in self.room_locations.iterkeys():
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.room_locations[room]
            MOVE_BASE[room] = SimpleActionTask("MOVE_BASE_" + str(room.upper()), "move_base", MoveBaseAction, goal, reset_after=False)
            
         # The root node
        BEHAVE = Sequence("BEHAVE")
        
        NAV_1=Sequence("NAV_LOCATION1")
        
        with NAV_1:
            WAIT4DOOR_OPEN=MonitorTask("WAIT4DOOR_OPEN", "doormsg",door_msg, self.door_is_open_cb)
            NAV_1.add_child(WAIT4DOOR_OPEN)
            NAV_1.add_child(MOVE_BASE['Point1'])
            
        
        NAV_2=Selector("NAV_LOCATION2")
        
        with NAV_2:
            NAV_2.add_child(MOVE_BASE['Point2'])
            
            CHECK=Selector('CHECH')
            
            with CHECK:
                
                CHECK_PERSON=Sequence("CHECK_PERSON")
                with CHECK_PERSON:
                    CHECK_ING= MonitorTask("CHECK_ING", 'people_position_estimation',PositionMeasurement,self.pos_M_cb)
                    ASK4LEAVE=Speak("ASK4LEAVE","excuse me,please let me get out of here")
                    CHECK_PERSON.add_child(CHECK_ING)
                    CHECK_PERSON.add_child(ASK4LEAVE)
                CHECH_OBJECT=Wait("WAIT_PEOPLE_LEAVE")
                CHECK.add_child(CHECK_PERSON)
                CHECK.add_child(CHECH_OBJECT)
            IGNORESC=IgnoreSuccess("IGNORESUCCESS")
            IGNORESC.add_child(CHECK)
            NAV_2.add_child(IGNORESC)
            NAV_2.add_child(MOVE_BASE['Point2'])
                
        NAV_3=Sequence("NAV_LOCATION3")
        
        with NAV_3:
            #NAV_3.add_child(MOVE_BASE['Point3'])
            FOLLOW_HEALTHY=Selector("FOLLOW_HEALTHY")
            with FOLLOW_HEALTHY:
                FOLLOW_MONITOR=MonitorTask("FOLLOW_MONITOR","speech_sub",Int32,self.check_stop_cb)
                PARALLEL=ParallelOne("PARALLEL")
                SET_POS=MonitorTask("SET_POS",'people_position_estimation',PositionMeasurement,self.people_msg_cb)
                FOLLOW=Follow("FOLLOW")
                PARALLEL.add_child(SET_POS)
                PARALLEL.add_child(FOLLOW)
                FOLLOW_HEALTHY.add_child(FOLLOW_MONITOR)
                FOLLOW_HEALTHY.add_child(PARALLEL)
            NAV_3.add_child(FOLLOW_HEALTHY)
            #NAV_3.add_child(MOVE_BASE['Point3'])
            
        NAV_4=Sequence("NAV_LOCATION4")
        
        with NAV_4:
            NAV_2.add_child(MOVE_BASE['Point4'])
        
#         BEHAVE.add_child(NAV_1)
#         BEHAVE.add_child(NAV_2)
        BEHAVE.add_child(NAV_3)
#        BEHAVE.add_child(NAV_4)
        print_tree(BEHAVE)
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(0.1)
    
    def people_msg_cb(self,msg):
        global PEOPLE_POSITION,FLAG
        if msg.pos is None:
            return TaskStatus.RUNNING
        else :
            lock = threading.Lock()
            with lock:
                PEOPLE_POSITION=msg.pos
                FLAG+=1
            rospy.loginfo(PEOPLE_POSITION)
            return TaskStatus.FAILURE
            
    def check_stop_cb(self,msg):
        if msg.data==None:
            return TaskStatus.RUNNING
        elif msg.data==7:
            subprocess.call(['espeak',"I am ready, Please stand in front of me."])
            return TaskStatus.FAILURE
        elif msg.data==8:
            subprocess.call(['espeak',"OK, I am stoping, ,I will go back"])
            return TaskStatus.SUCCESS
        
    def door_is_open_cb(self,msg):
        if msg.door_open_msg==False:
            return TaskStatus.RUNNING
        elif msg.door_open_msg==True:
            rospy.sleep(3)
            subprocess.call(["rosservice","call","/move_base/clear_costmaps"])
            return TaskStatus.SUCCESS       
        else :
            return TaskStatus.RUNNING         
            
    def pos_M_cb(self,msg):
        if msg.pos==None:
            return TaskStatus.FAILURE
        else :
            return TaskStatus.SUCCESS
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        self.move_base.cancel_all_goals()
        
        self.cmd_vel_pub.publish(Twist())
        
        rospy.sleep(1)
        
        
if __name__ == '__main__':
    try:
        tree = navgation()
    except KeyboardInterrupt:
        pass
