#!/usr/bin/env python
import rospy

# The name of the parameter to define the environment size.
# It should be a list `[max_x, max_y]` such that x:[0, `max_x`) and y:[0, `max_y`).
PARAM_ENVIRONMENT_SIZE = 'config/environment_size'

# The name of parameter to set the initial robot position.
PARAM_INITIAL_POSE = 'state/initial_pose'

# The name of the node representing the shared knowledge required for this scenario.
NODE_ROBOT_STATE = 'robot-state'

# The name of the server to get the current robot pose.
SERVER_GET_POSE = 'state/get_pose'

# The name of the server to set the current robot pose. 
SERVER_SET_POSE = 'state/set_pose'

# The name of the server to get the current robot battery level.
SERVER_GET_BATTERY_LEVEL = 'state/get_battery_level'

# The name of the server to set the current robot battery level. 
SERVER_SET_BATTERY_LEVEL = 'state/set_battery_level'

# The name of the planner node.
NODE_PLANNER = 'planner'

# The name of the action server solving the motion planning problem.
ACTION_PLANNER = 'motion/planner'

# The name of the controller node.
NODE_CONTROLLER = 'controller'

# The name of the action server solving the motion control problem.
ACTION_CONTROLLER = 'motion/controller'

# The name of the finite state machine node.
NODE_FINITE_STATE_MACHINE = 'fsm'

# Function used to label each log with a producer tag.
def tag_log(msg, producer_tag):
    return '@%s>> %s' % (producer_tag, msg)
