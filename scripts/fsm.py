#!/usr/bin/env python
"""
.. module:: fsm
    :platform: Unix
    :synopsis: the fsm python script in assignment1 package

.. moduleauthor:: Saeed Kalateh <s4982001@studenti.unige.it>

Uses Service:
    /state/set_battery_level

    /state/get_pose

Uses Action:
    /motion/planner
    
    /motion/controller
    
Uses helper script:

    /utilities/assignment1/helper.py

Defines the states and transitions for the finite state machine of this rospackage. In the initial state
robot builds the semantic map of environment after loading the ``topological_map.owl`` file. This node
uses ``helper.py`` script to update the ontology while the process is running, and retreives
the target room based on last visit times and robot battey state. In the next state it moves to the target room
and if battery level gets lower than threshold, it goes to charger, and charges the battery.
"""
import rospy
import smach
import smach_ros
import math
import actionlib
from assignment1.msg import PlanGoal
from assignment1.srv import GetPose, SetBatteryLevel
from assignment1 import architecture_name_mapper as anm
from assignment1.helper import TopologicalMap
import assignment1  # This is required to pass the `PlanAction` and `ControlAction` type for instantiating the `SimpleActionServer`.


# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_FINITE_STATE_MACHINE
LOOP_TIME = 5

planner_client = None
tm = None
mutex = None

def move_to_room(room):
    """
    Sends target room's corresponding position by calling ``move_to_pose(x,y)`` function. 
    Subsecuently, it would initiate the planner server, and once the path is found, the controller
    server moves the robot.

    Args:
        room(string)
    """
    log_msg = 'Received request for robot to move to ' + room
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    def move_to_pose(x, y):
        goal = PlanGoal()
        goal.target.x = x
        goal.target.y = y
        # Sends the goal to the planner server.
        log_msg = 'Request sent to planner server'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        planner_client.send_goal(goal)
        log_msg = 'Waiting for planner to find the path'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        finished_before_timeout = planner_client.wait_for_result(timeout=rospy.Duration(30))
        # detects if the action is done before timeout
        if finished_before_timeout:
            log_msg = 'Plan found!'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            path = planner_client.get_result()
            # Sends the path to the controller server.
            log_msg = 'Request sent to controller server'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            controller_client.send_goal(path)
            log_msg = 'Waiting for controller to move robot to target'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            finished_before_timeout = controller_client.wait_for_result(timeout=rospy.Duration(30))
            # detects if the action is done before timeout
            if finished_before_timeout:
                log_msg = 'Target Reached!'
                rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            else:
                log_msg = 'Action did not finish before time out!'
                rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
                controller_client.cancel_all_goals()
        else:
            log_msg = 'Action did not finish before time out!'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            planner_client.cancel_all_goals()

    if room   == 'E':
        move_to_pose(3.5, 1.0)
    elif room == 'R1':
        move_to_pose(1.0, 4.5)
    elif room == 'R2': 
        move_to_pose(1.0, 7.5)
    elif room == 'R3':
        move_to_pose(9.0, 4.5) 
    elif room == 'R4':
        move_to_pose(9.0, 7.5) 
    elif room == 'C1':
        move_to_pose(3.0, 4.5) 
    elif room == 'C2':
        move_to_pose(6.5, 4.5)

def check_room_reached(room):
    """
        Checks if the robot has reached to a room 

        Args:
            room(string)

        Returns:
            target_reached(Bool)
        """

    def check_pose_reached(x, y):
        """
        Checks if the robot has reached to a specific pose by computing the eucledian distance between
        robot current pose and target pose 

        Args:
            pose(Point)

        Returns:
            target_reached(Bool)
        """
        rospy.wait_for_service(anm.SERVER_GET_POSE)
        try:
            service = rospy.ServiceProxy(anm.SERVER_GET_POSE, GetPose)
            response = service()
            pose = response.pose
            if math.sqrt((pose.x - x)**2 + (pose.y - y)**2) < 1:
                target_reached = True
            else:
                target_reached = False
            log_msg = 'target reached state: ' + str(target_reached)
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            return target_reached
        except rospy.ServiceException as e:
            log_msg = f'Server cannot get current robot position: {e}'
            rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
    
    if room   == 'E':
        target_reached = check_pose_reached(3.5, 1.0)
    elif room == 'R1':
        target_reached = check_pose_reached(1.0, 4.5)
    elif room == 'R2': 
        target_reached = check_pose_reached(1.0, 7.5)
    elif room == 'R3':
        target_reached = check_pose_reached(9.0, 4.5) 
    elif room == 'R4':
        target_reached = check_pose_reached(9.0, 7.5) 
    elif room == 'C1':
        target_reached = check_pose_reached(3.0, 4.5) 
    elif room == 'C2':
        target_reached = check_pose_reached(6.5, 4.5)
    
    return target_reached

    

def set_battery_level(battery_level):
    """
    Service client function for ``/state/set_battery_level`` Update the current robot battery level
    stored in the ``robot-state`` node

    Args:
        battery_level(int)
    """
    rospy.wait_for_service(anm.SERVER_SET_BATTERY_LEVEL)
    try:
        log_msg = f'Set current robot battery level to the `{anm.SERVER_SET_BATTERY_LEVEL}` node.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        service = rospy.ServiceProxy(anm.SERVER_SET_BATTERY_LEVEL, SetBatteryLevel)
        service(battery_level)
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current robot battery level: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))


class CreateTopologicalMap(smach.State):
    """
    Defines the initial state in which the robot creates the topological map, by loading the ``topological_map.owl`` file
    and adding the properties 
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['map_created'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex

        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            mutex.acquire()
            try:
                tm.build_semantic_map()
                rospy.sleep(LOOP_TIME)
                return 'map_created'
            finally:
                mutex.release()


class MoveToRoom(smach.State):
    """
    Defines the state when robot moves to the target room found by the ontology, using 
    ``move_to_room(room)`` function. Additionally, it updates the ontology while it moves to target room
    using ``update_ontology(now)`` function, until it reaches the target room, then exits the state by
    returning ``room_reached``. If the battery level gets lower than threshold, it returns ``battery_low``
    and target room will be canceled.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['room_reached', 'battery_low'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global tm
        now = rospy.get_rostime()
        [target_room, battery_low] = tm.update_ontology(now)
        move_to_room(target_room)
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            mutex.acquire()
            try:
                now = rospy.get_rostime()
                [next_target_room, battery_low] = tm.update_ontology(now)
                log_msg = 'target room: ' + target_room
                rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
                if battery_low:
                    return 'battery_low'
                else:
                    target_reached = check_room_reached(target_room)
                    if target_reached:
                        return 'room_reached'
            finally:
                mutex.release()
            rospy.sleep(LOOP_TIME)

class VisitRoom(smach.State):
    """
    Defines the state when robot has reached the target room and then visits it. 
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['room_visited'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global tm
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            mutex.acquire()
            try:
                rospy.sleep(LOOP_TIME)
                return 'room_visited'
            finally:
                mutex.release()


class MoveToCharger(smach.State):
    """
    Defines the state when battery level is low and moves to charger, using 
    ``move_to_room('E')`` function. Additionally, it updates the ontology while it moves to charger
    using ``update_ontology(now)`` function, until it reaches the charger.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['docked'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global tm
        now = rospy.get_rostime()
        tm.update_ontology(now)
        move_to_room('E')
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            mutex.acquire()
            try:
                now = rospy.get_rostime()
                tm.update_ontology(now)
                target_reached = check_room_reached('E')
                if target_reached:
                    return 'docked'              
            finally:
                mutex.release()
            rospy.sleep(LOOP_TIME)


class Recharging(smach.State):
    """
    Defines the state when robot has reached the charger and chargers battery after some time using
    ``set_battery_level(battery_level)`` function and then returns ``robot_charged`` transition.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['recharged'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global tm
        
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            mutex.acquire()
            try:
                now = rospy.get_rostime()
                tm.update_ontology(now)
                rospy.sleep(10)
                set_battery_level(20)
                return 'recharged'
            finally:
                mutex.release()


def main():
    """
    The main function for `fsm` node, initialises the node and takes an instance of
    ``TopologicalMap`` class in the time instance now, defines the states and transitions of 
    the finite state machine for topological map and finally starts the finite state machine process
    """
    # Initialise this node.
    global tm
    global planner_client
    global controller_client
    global mutex

    rospy.init_node(anm.NODE_FINITE_STATE_MACHINE, log_level=rospy.INFO)
    planner_client = actionlib.SimpleActionClient(anm.ACTION_PLANNER, assignment1.msg.PlanAction)
    controller_client = actionlib.SimpleActionClient(anm.ACTION_CONTROLLER, assignment1.msg.ControlAction)
    now = rospy.get_rostime()
    tm = TopologicalMap(LOG_TAG, now)

    # Get or create a new mutex.
    if mutex is None:
        mutex = Lock()
    else:
        mutex = mutex

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('CREATE_TOPOLOGICAL_MAP', CreateTopologicalMap(), transitions={'map_created':'MOVE_TO_ROOM'})
        smach.StateMachine.add('MOVE_TO_ROOM', MoveToRoom(), transitions={'battery_low':'MOVE_TO_CHARGER', 'room_reached':'VISIT_ROOM'})
        smach.StateMachine.add('MOVE_TO_CHARGER', MoveToCharger(), transitions={'docked':'RECHARGING'})
        smach.StateMachine.add('VISIT_ROOM', VisitRoom(), transitions={'room_visited':'MOVE_TO_ROOM'})
        smach.StateMachine.add('RECHARGING', Recharging(), transitions={'recharged':'MOVE_TO_ROOM'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
