#!/usr/bin/env python

"""
.. module:: controller
  :platform: Ubuntu 20.04
  :synopsis: Python module to move of the robot towards the target location

.. moduleauthor:: Gabriele Russo <gabriele.russo117@gmail.com>

This node implements an Action server that receives the path plan from the State Machine node
and moves the robot from the current position into the target position using move base.
Then it returns, as result, the final location reached by the robot,
to the Action Client which is the State Machine node.

Service:
  **/state/set_pose** set the new robot position in the Robot State node. \n

Action Server:
  **/motion/controller** given the path plan as a goal, simulate the robot motion in order to reach the final position. \n
  **/move_base** used to move the robot autonomously towards the goal position \n
"""
import random
import rospy

# Import constant name defined to structure the architecture.
from exproblab_assignment_2 import architecture_names as anm

# Import the simple ActionServer.
from actionlib import SimpleActionServer, SimpleActionClient

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

# Import custom message, actions and services.
from exproblab_assignment_2.msg import ControlFeedback, ControlResult
from exproblab_assignment_2.srv import NewPosition
import exproblab_assignment_2 # This is required to pass the `ControlAction` type for 
                                            # instantiating the `SimpleActionServer`.

# used to color the output text
from colorama import Fore

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_CONTROLLER




class ControllingAction(object):
    """
    This class represents the controller and its /motion/controller
    Action server used to move the robot towards a desired position.
    Given a target location, it moves the robot towards the target location 
    using move base. Then it updates the current robot position stored in 
    the 'robot-state' node.
    """

    def __init__(self):
        """
        define the action server and starts it, 
        and define the action client to move base.
        """

        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
                                      exproblab_assignment_2.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        
        self._as.start()

        # Log information.
        log_msg = (Fore.LIGHTCYAN_EX + f'`{anm.ACTION_CONTROLLER}` Action Server initialised.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print(Fore.WHITE)

        # Create a SimpleActionClient for the move_base action
        self.move_base_cli = SimpleActionClient('move_base', MoveBaseAction)

    def execute_callback(self, goal):
        """
        The callback executed when the client set the goal to the 'controller' server, 
        in this case the goal is the plan computed in the Planning state of the state machine.
        This function requires a target location (i.e., the plan), and it moves the robot
        towards this target location using move base.
        Once a new position is reached, the related robot position is updated
        in the `robot-state` node, through the /state/set_pose service.

        Args:
          goal (ControlGoal): the plan, composed of a list of locations

        """

        # Wait for the action server to come up
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_cli.wait_for_server()

        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.target_location is None:
          rospy.logerr(anm.tag_log(Fore.LIGHTRED_EX + 'No locations provided! This service will be aborted!', LOG_TAG))
          print(Fore.WHITE)
          self._as.set_aborted()
          return
        
        # Create a goal to send to the move_base action server
        move_goal = MoveBaseGoal()

        # Set the move base goal target pose 
        move_goal.target_pose.header.frame_id = "map"
        move_goal.target_pose.header.stamp = rospy.Time.now()
        move_goal.target_pose.pose = Pose(
            position=Point(x=goal.target_location.x, y=goal.target_location.y, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        
        # Send the goal to the action server
        rospy.loginfo(Fore.LIGHTBLUE_EX + "Sending goal to move_base...")
        print(Fore.WHITE)
        self.move_base_cli.send_goal(move_goal)

        # Check that the client did not cancel this service.
        if self._as.is_preempt_requested():
            rospy.loginfo(anm.tag_log(Fore.LIGHTRED_EX + 'Service has been cancelled by the client!', LOG_TAG))
            print(Fore.WHITE)

            # Actually cancel this service.
            self._as.set_preempted()
            
            return

        # Log current robot position.
        log_msg = Fore.LIGHTCYAN_EX + f'Reaching location ({goal.target_location.name}).'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print(Fore.WHITE)

        

        # Wait for the result (you can set a timeout if needed)
        self.move_base_cli.wait_for_result()
      
        # Publish the results to the client (the state machine).
        res = ControlResult()

        # Check if the goal was successful or canceled
        if self.move_base_cli.get_state() == GoalStatus.SUCCEEDED:
            
            rospy.loginfo(Fore.LIGHTBLUE_EX + 'Goal achieved!')
            print(Fore.WHITE)
            
            # Set the new current position into the `robot-state` node.
            _set_client_position(goal.target_location)

            res.final_location = goal.target_location

            log_msg = Fore.GREEN + 'Motion control successes.'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            print(Fore.WHITE)

            # Succeeded.
            self._as.set_succeeded(res)

            return
         
        elif self.move_base_cli.get_state() == GoalStatus.PREEMPTED:
            
            rospy.loginfo(Fore.LIGHTRED_EX + 'Goal canceled by user!')
            print(Fore.WHITE)

            # Actually cancel this service.
            self._as.set_preempted()

            return
        else:
            
            rospy.loginfo(Fore.LIGHTRED_EX + 'Goal failed!')
            print(Fore.WHITE)

            # Failed
            self._as.set_aborted(res)

            return 
            
def _set_client_position(position):
    """
    Update the current robot position stored in the `robot-state` node.
    This method is performed for each position provided in the action's server feedback.

    Args:
      position (Location): the new robot position

    """
    # wait for the server to be initialized.
    rospy.wait_for_service(anm.SERVER_SET_POSE)
    try:

        # Log service call.
        log_msg = f'Set current robot position to the `{anm.SERVER_SET_POSE}` node.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Call the service and set the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_SET_POSE, NewPosition)
        service(position)  # The response is not used.

    except rospy.ServiceException as e:

        log_msg = Fore.LIGHTRED_EX + f'Server cannot set current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))


if __name__ == '__main__':
    # Initialize the node, its action server, and wait.   
    rospy.init_node(anm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()