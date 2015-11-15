#!/usr/bin/env python 
#encoding:utf8 

"""
author 
    
        X_Y耀  created on 2015-8-15

"""
import rospy
from std_msgs.msg import Int32
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



class PrintMessage(Task):
    def __init__(self, name, message, *args, **kwargs):
        super(PrintMessage, self).__init__(name, *args, **kwargs)
        
        self.name = name
        self.message = message
        self.words = message.split()

        print "Creating Print Message task for", self.message
 
    def run(self):
        try:
            word = self.words.pop(0)
            rospy.loginfo( word)
            time.sleep(0.1)
            if self.words == []:
                return TaskStatus.SUCCESS
            return TaskStatus.RUNNING
        except:
            return TaskStatus.SUCCESS
        
    def reset(self):
        self.words = self.message.split()
        
class Count(Task):
    def __init__(self, name, number, *args, **kwargs):
        super(Count, self).__init__(name, *args, **kwargs)
        
        self.name = name
        self.number = number
        self.count = 0
        print "Creating counting task to", self.number
 
    def run(self):
        if self.count == self.number:
            return TaskStatus.SUCCESS
        else:
            time.sleep(0.1)
            self.count += 1
            rospy.loginfo( self.count)
            if self.count == self.number:
                        return TaskStatus.SUCCESS
            return TaskStatus.RUNNING

    
    def reset(self):
        self.count = 0
        
task_list =[]
Task_dic= OrderedDict([
                       (1, [Count(name="COUNT",number=10)]),
                       (2,[PrintMessage(name="Print",message="Take me to your leader!")])
                      
                      
                      
                      ])


class Stop(Task):
    def __init__(self, room=None, timer=3, *args):
        print "Stoping"
        name = "VACUUM_" + room.upper()
        super(Vacuum, self).__init__(name)    
        self.name = name
        self.room = room
        self.counter = timer
        self.finished = False
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.cmd_vel_msg = Twist()
        self.cmd_vel_msg.linear.x = 0.05

    def run(self):
        if self.finished:
            return TaskStatus.SUCCESS
        else:
            rospy.loginfo('Vacuuming the floor in the ' + str(self.room))
            
            while self.counter > 0:
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                self.cmd_vel_msg.linear.x *= -1
                rospy.loginfo(self.counter)
                self.counter -= 1
                rospy.sleep(1)
                return TaskStatus.RUNNING
            
            self.finished = True
            self.cmd_vel_pub.publish(Twist())
            message = "Finished vacuuming the " + str(self.room) + "!"
            rospy.loginfo(message)
        
class Check_Cmd(Task):
    def __init__(self,  *args, **kwargs):
        name='CHECK_CMD'
        super(Check_Cmd, self).__init__(name)
        self.name=name
        print "wait for command"
 
    def run(self):
        try:
            if len(task_list)==0:
                TaskStatus.SUCCESS
            else :
                rospy.loginfo("running")
                TaskStatus.FAILED
        except:
            return TaskStatus.SUCCESS
        
class ExecuteTask(Task):
#记得加线程锁
    def __init__(self,*args,**kwargs):
        name="EXECUTIVE_TASK"
        super(ExecuteTask,self).__init__(name,*args,**kwargs)
        self.name=name
        print "execute the task"
    def run(self):
        try:
            if len(task_list)!=0:
#                 rospy.loginfo("running")
                ExecuteTask.add_child(Task_dic[task_list[0]])
                rospy.loginfo("running")
                del task_list[0]
#                 rospy.loginfo("running")
                for c in self.children:
            
                    c.status = c.run()
#                     rospy.loginfo("running")  
                    if c.status != TaskStatus.SUCCESS:
                        return c.status   
            else :
                return TaskStatus.RUNNING
            
        except:
            pass
#     def reset(self):
#         for task in ExecuteTask.children:
#                 ExecuteTask.remove_child(task)
#         task_list=[]
#         return TaskStatus.SUCCESS
             

class xm_trees():
    def __init__(self):
        rospy.init_node('xm_tree', anonymous=False)
        # The root node
        BEHAVE = Sequence("BEHEAVE")
        
         # Initialize the ParallelAll task
        PARALLEL = ParallelAll("Listen_AND_CHECK")
        
        BEHAVE.add_child(PARALLEL)
        
        SPEECH_CMD = MonitorTask("SPEECH", "speech_sub", Int32, self.task_check_cb)
        
        CMD_EXE=Selector("CMD_LIST")
        
        PARALLEL.add_child(SPEECH_CMD)
        PARALLEL.add_child(CMD_EXE)
        
        with CMD_EXE:
            #CMD_EXE.add_child(Check_Cmd())
            CMD_EXE.add_child(ExecuteTask())
            pass
        print "Behavior Tree\n"
        print_tree(BEHAVE)

        rospy.loginfo("Starting simulated house cleaning test")
            
        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(0.1)
            BEHAVE.reset()
            #rospy.loginfo("running")
        
    
    def task_check_cb(self,msg):
        if msg.data is None:
            rospy.loginfo("no msg")
            return TaskStatus.RUNNING
        task_list.append(msg.data)
        print task_list
        if msg.data == -1:
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.RUNNING

if __name__ == '__main__':
    try:
        tree = xm_trees()
    except KeyboardInterrupt:
        pass
    
