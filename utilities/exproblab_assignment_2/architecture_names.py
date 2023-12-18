#!/usr/bin/env python

"""
.. module:: architecture_names
   :platform: Ubuntu 20.04
   :synopsis: Python module which implements a 'dictionary' that defines names and a function for logging in the architecture.

.. moduleauthor:: Gabriele Russo <gabriele.russo117@gmail.com>

it implements a 'dictionary' that defines names and a function for logging used among all the components of the architecture of this assignment.

"""

import rospy

# ---------------------------------------------------------

# The name of the robot state node which represent the shared knowledge.
NODE_ROBOT_STATE = 'robot_state'

# The name of the server to set the actual robot position. 
SERVER_SET_POSE = 'state/set_pose'

# The name of the topic where the battery state is published.
TOPIC_BATTERY_LOW = 'state/battery_low'

# -------------------------------------------------

# The name of the controller node.
NODE_CONTROLLER = 'controller'

# The name of the action server solving the motion control problem.
ACTION_CONTROLLER = 'motion/controller'

# -------------------------------------------------

# the name of the state machine node
NODE_STATEMACHINE = 'state_machine'

# the name of the server used to start the robot charging process
SERVER_CHARGING = 'inteface/start_charging' 

# the name of the topic where the state machine takes the
# marker id detected by the camera
MARKER_ID = '/marker_publisher/detected_marker_id'

# the name of the server where the state machine sends the
# marker id and gets the location informations associated 
# to that marker id
MARKER_SERVER = '/room_info'

# the name of the topic where the state machine sends the 
# joint position commands
JOINT_1_COMMAND = '/robot/joint1_position_controller/command'
JOINT_2_COMMAND = '/robot/joint2_position_controller/command'

# Function used to label each log with a producer tag.
def tag_log(msg, producer_tag):
    return f'@{producer_tag}>> {msg}'
