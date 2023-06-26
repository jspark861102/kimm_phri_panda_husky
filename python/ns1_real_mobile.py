#! /usr/bin/python3 

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import Int16
import numpy as np
import importlib, pkgutil
import threading
import cmd, sys, os
import copy


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class ControlSuiteShell(cmd.Cmd):
    intro = bcolors.OKBLUE + "Welcome to the control suite shell.\nType help or ? to list commands.\n" + bcolors.ENDC
    prompt = "(csuite) "

    def __init__(self):
        cmd.Cmd.__init__(self)
        rospy.init_node('phri_real_mobile_controlsuite')        
        
        self.ctrl_type = 0
        self.ctrl_type_pub = rospy.Publisher('/ns1/real_robot/ctrl_type', Int16, queue_size=1)

    def do_gravity(self, arg):
        'Gravity Ctrl mode'
        self.ctrl_type = 0
        self.ctrl_type_pub.publish(self.ctrl_type)
    
    def do_home(self, arg):
        'Go to the home position using joint posture ctrl'                
        self.ctrl_type = 1
        self.ctrl_type_pub.publish(self.ctrl_type) 
    

    def do_quit(self, arg):
        return True

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()