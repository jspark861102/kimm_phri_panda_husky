#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal, GraspEpsilon

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    # client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    # goal = MoveGoal(speed=0.1,width=0.07)
    
    goal = GraspGoal(speed=0.1, width=0.03, force=80, epsilon=GraspEpsilon(inner=0.01, outer=0.01))

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fdsafdsa')
        result = fibonacci_client()
    except:
        pass