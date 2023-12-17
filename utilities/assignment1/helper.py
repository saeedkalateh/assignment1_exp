#!/usr/bin/env python
"""
.. module:: helper
    :platform: Unix
    :synopsis: the helper python script in assignment1 package

.. moduleauthor:: Saeed Kalateh <s4982001@studenti.unige.it>

Uses Service:
    /state/get_battery_level  
    
    /state/get_pose

Uses helper script:
    /utilities/armor_api/armor_client.py

Defines the topological map using the ``armor_client.py`` helper script methods and the
ontology file ``topological_map.owl``, which is not complete (only contatins class definitions)
"""
import rospy
from os.path import dirname, realpath
from assignment1 import architecture_name_mapper as anm
from assignment1.srv import GetPose, GetBatteryLevel
from assignment1.msg import Point
from armor_api.armor_client import ArmorClient

class TopologicalMap:
    """
        Class for implemnting the topological map. When an instance is taken from this class,
        it loads the ontology file using ``armor_client`` loading method and tries to complete it by 
        adding rooms, doors and robot individuals, disjointing them, predefining the last visit 
        times and placing the robot to room "E" by calling the corresponding functions which use
        ``armor_client`` methods.
    """
    def __init__(self, log_tag, init_time):
        self.log_tag = log_tag
        self.init_time = init_time
        # Build topological map
        self.path = dirname(realpath(__file__))
        self.path = self.path + "/../../ontology/"
        self.client = ArmorClient("ontology", "ontology_reference")
        self.client.utils.load_ref_from_file(self.path + "topological_map.owl", "", True, "PELLET", False)
        self.client.utils.set_log_to_terminal(True)
        
    def build_semantic_map(self):
        self.add_individuals_to_classes()
        self.disjoint_individuals()
        self.assign_doors_to_rooms()
        self.add_last_visit_times()
        self.add_robot()

    def add_individuals_to_classes(self):
        """
        Adds rooms, doors and robot in to the topological map using ``armor_client``
        add individual to class method and finaly syncs the reasoner
        """
        # Add indiviuals to class --> ROOM
        self.client.manipulation.add_ind_to_class("E", "ROOM")
        self.client.manipulation.add_ind_to_class("C1", "ROOM")
        self.client.manipulation.add_ind_to_class("C2", "ROOM")
        self.client.manipulation.add_ind_to_class("R1", "ROOM")
        self.client.manipulation.add_ind_to_class("R2", "ROOM")
        self.client.manipulation.add_ind_to_class("R3", "ROOM")
        self.client.manipulation.add_ind_to_class("R4", "ROOM")
        self.client.utils.sync_buffered_reasoner()  

        #Add indiviuals to class --> DOOR
        self.client.manipulation.add_ind_to_class("D1", "DOOR")
        self.client.manipulation.add_ind_to_class("D2", "DOOR")
        self.client.manipulation.add_ind_to_class("D3", "DOOR")
        self.client.manipulation.add_ind_to_class("D4", "DOOR")
        self.client.manipulation.add_ind_to_class("D5", "DOOR")
        self.client.manipulation.add_ind_to_class("D6", "DOOR")
        self.client.manipulation.add_ind_to_class("D7", "DOOR")
        self.client.utils.sync_buffered_reasoner()  

        #Add indiviuals to class --> ROBOT
        self.client.manipulation.add_ind_to_class("Robot", "ROBOT")   
        self.client.utils.sync_buffered_reasoner()   

    def disjoint_individuals(self):
        """
        Disjoints every individual in each class using ``armor_client`` disjoint 
        individuals of class method and finally syncs the reasoner
        """
        self.client.manipulation.disj_inds_of_class("ROOM")
        self.client.manipulation.disj_inds_of_class("DOOR")
        self.client.utils.sync_buffered_reasoner()

    def assign_doors_to_rooms(self):
        """
        Assigns the doors to the rooms using ``armor_client`` add object to individual 
        method and finally syncs the reasoner
        """
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "R1", "D1")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "R2", "D2")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "R3", "D3")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "R4", "D4")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "R4", "D4")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D6")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D7")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D1")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D2")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D5")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D7")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D3")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D4")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D5")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D6")
        self.client.utils.sync_buffered_reasoner()

    def add_last_visit_times(self):
        """
        Defines the initial last visit times using ``armor_client`` add data to individual 
        method and finally syncs the reasoner
        """
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "R1", "Int", str(self.init_time.secs - 35))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "R2", "Int", str(self.init_time.secs - 40))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "R3", "Int", str(self.init_time.secs - 25))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "R4", "Int", str(self.init_time.secs - 15))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "E", "Int", str(self.init_time.secs))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "C1", "Int", str(self.init_time.secs - 5))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "C2", "Int", str(self.init_time.secs - 10))  
        self.client.utils.sync_buffered_reasoner()

    def add_robot(self):
        """
        Places the robot in room "E" and sets its initial time instance and battery level
        and defines its urgency threshold using ``armor_client`` add data  and object to individual 
        methods and finally syncs the reasoner
        """ 
        self.client.manipulation.add_dataprop_to_ind("now", "Robot", "Int", str(self.init_time.secs))
        self.client.utils.sync_buffered_reasoner()  
        self.client.manipulation.add_objectprop_to_ind("isIn", "Robot", self.get_location())
        self.client.utils.sync_buffered_reasoner()  
        self.client.manipulation.add_dataprop_to_ind("batteryLvl", "Robot", "Int", str(self.get_battery_level_client()))
        self.client.utils.sync_buffered_reasoner()  
        self.client.manipulation.add_dataprop_to_ind("urgencyThreshold", "Robot", "Int", "7")
        self.client.utils.sync_buffered_reasoner()       

    def cut_dataprop(self, data_prop):
        """ 
        Cuts the data prop from a string received from armor.

        Args:
            data_prop(string)
        """
        start = 0
        end = data_prop.rfind('^') - 2
        data_prop = data_prop[(start+1) : end]
        return data_prop

    def cut_dataprop_list(self, data_prop_list):
        """ 
        Cuts the data prop from a list of strings received from armor.
        
        Args:
            data_prop_list(string[])
        """
        for i in range(len(data_prop_list)):
            data_prop_list[i] = self.cut_dataprop(data_prop_list[i])
        return data_prop_list

    def get_battery_level_client(self):
        """
        Retrieve the current robot battery level by the ``state/battery_level`` server of the 
        ``robot-state`` node.

        Returns:
            battery_level(int)
        """
        # Eventually, wait for the server to be initialised.
        rospy.wait_for_service(anm.SERVER_GET_BATTERY_LEVEL)
        try:
            # Call the service and get a response with the current robot battery level.
            service = rospy.ServiceProxy(anm.SERVER_GET_BATTERY_LEVEL, GetBatteryLevel)
            response = service()
            battery_level = response.battery_level
            # Log service response.
            log_msg = f'Retrieving current robot battery level from the `{anm.NODE_ROBOT_STATE}` node as: {battery_level}.'
            rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
            return battery_level
        except rospy.ServiceException as e:
            log_msg = f'Server cannot get current robot battery level: {e}'
            rospy.logerr(anm.tag_log(log_msg, self.log_tag))

    def get_pose_client(self):
        """
        Retrieve the current robot pose by the ``state/get_pose`` server of the 
        ``robot-state`` node.

        Returns:
            pose(Point)
        """
        # Eventually, wait for the server to be initialised.
        rospy.wait_for_service(anm.SERVER_GET_POSE)
        try:
            # Call the service and get a response with the current robot position.
            service = rospy.ServiceProxy(anm.SERVER_GET_POSE, GetPose)
            response = service()
            pose = response.pose
            # Log service response.
            log_msg = f'Retrieving current robot position from the `{anm.NODE_ROBOT_STATE}` node as: ({pose.x}, {pose.y}).'
            rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
            return pose
        except rospy.ServiceException as e:
            log_msg = f'Server cannot get current robot position: {e}'
            rospy.logerr(anm.tag_log(log_msg, self.log_tag))

    def get_location(self):
        """
        Detects robot current position using ``get_pose_client()`` function and then checks in which room
        it is, it also updates robot location in the ontology using ``armor_client`` replace data belonging
        to an individual and finally syncs the reasoner.

        Returns:
            is_in(string)
        """
        pose = Point()
        pose = self.get_pose_client()
        now = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("now", "Robot"))[0]

        if pose.y <= 2:
            is_in = "E"
            prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "E"))[0]
            self.client.manipulation.replace_dataprop_b2_ind("visitedAt", "E", "Int", now, prev_time)
            
        elif pose.x <= 2 and pose.y > 2 and pose.y <= 6:
            is_in = "R1"
            prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R1"))[0]
            self.client.manipulation.replace_dataprop_b2_ind("visitedAt", "R1", "Int", now, prev_time)
            
        elif pose.x <= 2 and pose.y > 6:
            is_in = "R2"
            prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R2"))[0]
            self.client.manipulation.replace_dataprop_b2_ind("visitedAt", "R2", "Int", now, prev_time)

        elif pose.x > 2 and pose.x <= 5 and pose.y > 2:
            is_in = "C1"
            prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "C1"))[0]
            self.client.manipulation.replace_dataprop_b2_ind("visitedAt", "C1", "Int", now, prev_time)

        elif pose.x > 5 and pose.x <= 8 and pose.y > 2:
            is_in = "C2"
            prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "C2"))[0]
            self.client.manipulation.replace_dataprop_b2_ind("visitedAt", "C2", "Int", now, prev_time)

        elif pose.x > 8 and pose.y > 2 and pose.y <= 6:
            is_in = "R3"
            prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R3"))[0]
            self.client.manipulation.replace_dataprop_b2_ind("visitedAt", "R3", "Int", now, prev_time)

        elif pose.x > 8 and pose.y > 6:
            is_in = "R4"
            prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R4"))[0]
            self.client.manipulation.replace_dataprop_b2_ind("visitedAt", "R4", "Int", now, prev_time)

        self.client.utils.sync_buffered_reasoner()
        return is_in


    def update_ontology(self, now):
        """
        The function which is called in ``finite_state_machine`` node, it gets current time instance as
        an argument, it gets robot current location using ``get_location()`` function, it gets robot current battery
        level using ``get_battery_level_client()`` function, and updates them in the ontology. It sorts the last visit
        times and detects which room is the most behind and sets it as the target room. It detects the urgent rooms
        considering the last visit times and robot urgeny threshold and updates them in the ontology, finally, it 
        returns the target room as it is found if the battery level is high enough otherwise it returns room "E" as
        target room.

        Returns:
            target_room(string)
        """
        # Update robot time instance
        prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("now", "Robot"))[0]
        self.client.manipulation.replace_dataprop_b2_ind("now", "Robot", "Int", str(now.secs), prev_time)
        self.client.utils.sync_buffered_reasoner()
        # Update battery level
        prev_battery_level = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]
        battery_level = str(self.get_battery_level_client())
        self.client.manipulation.replace_dataprop_b2_ind("batteryLvl", "Robot", "Int", battery_level, prev_battery_level)
        self.client.utils.sync_buffered_reasoner()
        log_msg = 'battery level: ' + self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]
        rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        # Update robot location
        loc = self.get_location()
        log_msg = 'current location: ' + loc
        rospy.loginfo(anm.tag_log(log_msg, self.log_tag))

        visitedAt_E = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "E"))[0]
        visitedAt_R1 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R1"))[0]
        visitedAt_R2 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R2"))[0]
        visitedAt_R3 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R3"))[0]
        visitedAt_R4 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R4"))[0]
        visitedAt_C1 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "C1"))[0]
        visitedAt_C2 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "C2"))[0]

        visitedAt_dict = {visitedAt_R1: "R1", visitedAt_R2: "R2", visitedAt_R3: "R3", visitedAt_R4: "R4", visitedAt_C1: "C1", visitedAt_C2: "C2", visitedAt_E: "E"}
        visitedAt_dict = dict(sorted(visitedAt_dict.items()))
        room_list = list(visitedAt_dict.values())
        target_room = room_list[0]

        urgency_threshold = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("urgencyThreshold", "Robot"))[0]

        if now.secs - int(visitedAt_E) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("E", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("E", "URGENT")

        if now.secs - int(visitedAt_R1) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("R1", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("R1", "URGENT")
        
        if now.secs - int(visitedAt_R2) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("R2", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("R2", "URGENT")
        
        if now.secs - int(visitedAt_R3) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("R3", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("R3", "URGENT")
        
        if now.secs - int(visitedAt_R4) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("R4", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("R4", "URGENT")
        
        if now.secs - int(visitedAt_C1) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("C1", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("C1", "URGENT")
        
        if now.secs - int(visitedAt_C2) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("C2", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("C2", "URGENT")
        self.client.utils.sync_buffered_reasoner()

        urgent_rooms = self.client.query.ind_b2_class("URGENT")
        log_msg = 'urgent locations: ' 
        rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        for i in range(0,len(urgent_rooms)):
            log_msg = urgent_rooms[i]
            rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        
        battery_lvl = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]
        if int(battery_lvl) > int(urgency_threshold):
            log_msg = 'target room: ' + target_room
            rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
            return target_room, False
        else:
            log_msg = 'target room: "E"' 
            rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
            return "E", True

