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


class SpeechMonitor(Task):
    def __init__(self, name, message, *args, **kwargs):
        super(PrintMessage, self).__init__(name, *args, **kwargs)
        
        self.name = name
        self.message = message
        self.words = message.split()

        print "Creating Print Message task for", self.message
 
    def run(self):
        try:
            word = self.words.pop(0)
            print word
            time.sleep(0.1)
            if self.words == []:
                return TaskStatus.SUCCESS
            return TaskStatus.RUNNING
        except:
            return TaskStatus.SUCCESS
        
    def reset(self):
        self.words = self.message.split()
class main():
    pass

if __name__  == 'main':
    try:
        main()
    except:
        rospy.loginfo("speech_monitor finished")