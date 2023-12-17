#! /usr/bin/env python
"""
.. module:: robot_state
    :platform: Unix
    :synopsis: the robot state python script in assignment1 package

.. moduleauthor:: Saeed Kalateh <s4982001@studenti.unige.it>

Uses Service:
    /state/get_pose

    /state/set_pose

    /state/get_battery_level
    
    /state/set_battery_level

Simulates the robot by defining a battery level, and pose. Gets controled by the controller server.
"""

import rospy
from assignment1 import architecture_name_mapper as anm
from assignment1.msg import Point
from assignment1.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse, GetBatteryLevel, SetBatteryLevel, GetBatteryLevelResponse, SetBatteryLevelResponse

LOG_TAG = anm.NODE_ROBOT_STATE


class RobotState:
    """
    This class defines two services to get and set the current 
    robot pose, and a publisher to notify that the battery is low.
    """

    def __init__(self):
        # Initialise this node.
        rospy.init_node(anm.NODE_ROBOT_STATE, log_level=rospy.INFO)
        # Initialise robot position.
        self._pose = Point()
        start_point_param = rospy.get_param("/state/initial_pose")
        self._pose.x = start_point_param[0]
        self._pose.y = start_point_param[1]
        # Initialise robot battery level.
        self._battery_level = 20
        # Define services.
        rospy.Service(anm.SERVER_GET_POSE, GetPose, self.get_pose)
        rospy.Service(anm.SERVER_SET_POSE, SetPose, self.set_pose)
        rospy.Service(anm.SERVER_GET_BATTERY_LEVEL, GetBatteryLevel, self.get_battery_level)
        rospy.Service(anm.SERVER_SET_BATTERY_LEVEL, SetBatteryLevel, self.set_battery_level)
        # Log information.
        log_msg = (f'Initialise node `{anm.NODE_ROBOT_STATE}` with services `{anm.SERVER_GET_POSE}` and '
                   f'`{anm.SERVER_SET_POSE}`, `{anm.SERVER_GET_BATTERY_LEVEL}` and `{anm.SERVER_SET_BATTERY_LEVEL}`.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    def set_pose(self, request):
        """
        The ``robot/set_pose`` service implementation.
        The ``request`` input parameter is the current robot pose to be set,
        as given by the client. This server returns an empty ``response``.
        """
        if request.pose is not None:
            # Store the new current robot position.
            self._pose = request.pose
            # Log information.
            log_msg = (f'Set current robot position through `{anm.SERVER_SET_POSE}` '
                             f'as ({self._pose.x}, {self._pose.y}).')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        else:
            rospy.logerr(anm.tag_log('Cannot set an unspecified robot position', LOG_TAG))
        # Return an empty response.
        return SetPoseResponse()

    def get_pose(self, request):
        """
        The ``robot/get_pose`` service implementation.
        The ``request`` input parameter is given by the client as empty. Thus, it is not used.
        The ``response`` returned to the client contains the current robot pose.
        """
        # Log information.
        if self._pose is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot position', LOG_TAG))
        else:
            log_msg = f'Get current robot position through `{anm.SERVER_GET_POSE}` as ({self._pose.x}, {self._pose.y})'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # Create the response with the robot pose and return it.
        response = GetPoseResponse()
        response.pose = self._pose
        return response

    def set_battery_level(self, request):
        """
        The ``robot/set_battery_level`` service implementation.
        The ``request`` input parameter is given by the client as empty. Thus, it is not used.
        The ``response`` returned to the client contains the current robot battery level.
        """
        if request.battery_level is not None:
            # Store the new current robot battery level.
            self._battery_level = request.battery_level
            # Log information.
            log_msg = (f'Set current robot battery level through `{anm.SERVER_SET_BATTERY_LEVEL}` '
                             f'as ({self._battery_level}).')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        else:
            rospy.logerr(anm.tag_log('Cannot set an unspecified robot battery level', LOG_TAG))
        # Return an empty response.
        return SetBatteryLevelResponse()

    def get_battery_level(self, request):
        """
        The ``robot/get_battery_level`` service implementation.
        The ``request`` input parameter is given by the client as empty. Thus, it is not used.
        The ``response`` returned to the client contains the current robot battery level.
        """
        # Log information.
        if self._battery_level is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot battery level', LOG_TAG))
        else:
            log_msg = f'Get current robot battery level through `{anm.SERVER_GET_BATTERY_LEVEL}` as ({self._battery_level})'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # Create the response with the robot battery level and return it.
        response = GetBatteryLevelResponse()
        response.battery_level = self._battery_level
        return response

if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()

