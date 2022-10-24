#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
# import tf
import numpy as np
# from nav_msgs.msg import Path, Odometry
# from geometry_msgs.msg import PoseStamped, Point, Quaternion
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
from nav_msgs.msg import Path

# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Vector3
# from std_msgs.msg import ColorRGBA

class RosManager:
    def __init__(self, cams_planner):
        rospy.init_node("cams_planner", anonymous=True)

        self.cams_planner = cams_planner
        
        self.sampling_rate = 30
        self.ros_rate = rospy.Rate(self.sampling_rate)

        self.ego_vehicle_status = EgoVehicleStatus()
        self.object_status_list = ObjectStatusList()
        self.global_path = Path()
        self.local_path = Path()

        self.is_ego_vehicle_status = False
        self.is_object_status_list = False
        self.is_global_path = False
        self.is_local_path = False

    def execute(self):
        print("start cams planner")
        self._set_protocol()
        while not rospy.is_shutdown():
            if self.is_ego_vehicle_status and self.is_object_status_list and self.is_local_path:
                cams_path, cams_path_candidates = self.cams_planner.execute(
                    self.ego_vehicle_status, self.object_status_list, self.global_path, self.local_path
                    )
                self._send_data(cams_path, cams_path_candidates)
        print("end cams planner")

    def _set_protocol(self):
        # publisher
        self.cams_path_pub = rospy.Publisher('/cams_path', Path, queue_size=1)

        # subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.cb_ego_vehicle_status)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.cb_object_status_list)
        rospy.Subscriber('/global_path', Path, self.cb_global_path)
        rospy.Subscriber('/local_path', Path, self.cb_local_path)
    
    def _send_data(self, cams_path, cams_path_candidates):
        self.cams_path_pub.publish(cams_path)

        for i in range(len(cams_path_candidates)):
            globals()['cams_path_{}'.format(i+1)] = rospy.Publisher('/cams_path_{}'.format(i+1), Path, queue_size = 1)
            globals()['cams_path_{}'.format(i+1)].publish(cams_path_candidates[i])
        
        self.ros_rate.sleep()

    def cb_global_path(self, data):
        self.global_path = data
        self.is_global_path = True
    
    def cb_local_path(self, data):
        self.local_path = data
        self.is_local_path = True

    def cb_ego_vehicle_status(self, data):
        '''ego vehicle status callback

        Parameters
        ---------------------
        data : morai_msgs.msg.EgoVehicleStatus
            topic   : /Ego_topic
            msg     : EgoVehicleStatus
                position    : x,y,z position in global coordintae
                velocity    : x(longitudinal), y(lateral), z, velocity [m/s] in vehicle coordintae
                accleration : x, y, z, acceleration [m/s^2] in vehicle coordintae

                heading     : vehicle heading [deg]
                accel       : gas pedal [0~1]
                brake       : brake     [0~1]
                wheel_angle : steering wheel angle [deg]


        Set Instance Variables
        ------------------------------
        self.ego_vehicle_status : morai_msgs.msg.EgoVehicleStatus
                                    save the original ego vehicle status data (heading -> rad으로 변경함)
        '''     
        # Ego Vehicle Status heading deg -> rad
        self.ego_vehicle_status = data      # class instance의 경우 복사가 이루어지지 않고, 객체를 참조함 (data와 self.ego_vehicle_status가 동일해짐)
        self.ego_vehicle_status.heading = np.deg2rad(self.ego_vehicle_status.heading) # data의 heading을 rad으로 변환

        self.is_ego_vehicle_status = True

    def cb_object_status_list(self, data):
        '''object info callback

        Parameters
        ---------------------
        data : morai_msgs.msg.ObjectStatusList
            topic   : /Object_topic, 
            msg     : ObjectStatusList
                header              : std_msgs.msg.Header
                num_of_npcs         : int32
                num_of_pedestrian   : int32
                num_of_obstacle     : int32

                npc_list            : list of morai_msgs.msg.ObjectStatus
                pedestrian_list     : list of morai_msgs.msg.ObjectStatus
                obstacle_list       : list of morai_msgs.msg.ObjectStatus

            
                morai_msgs.msg.ObjectStatus
                    unique_id   : int32
                    type        : int32
                                    object_type (0: Pedestrian, 1: NPC vehicle, 2: Static object (obstacle), -1: Ego-vehicle)
                    name        : string

                    size        : width, length, hight [m]
                    position    : x, y, z [m] (global)
                    velocity    : x(longitudinal), y(lateral), z, velocity [km/h] in vehicle coordintae
                    accleration : x, y, z, acceleration [m/s^2] in vehicle coordinate
                    heading     : vehicle heading [deg]

        Set Instance Variables
        ------------------------------
        self.object_info_list : lists of perception.object_info.ObjectInfo
                                position : localization.point.Point(x, y)
                                velocity : x(longitudinal) velocity [m/s] in vehicle coordintae
                                type : object_type
                                        (0: Pedestrian, 1: NPC vehicle, 2: Static object (obstacle), -1: Ego-vehicle)
                                

        self.object_status_list  : morai_msgs.msg.ObjectStatusList
                                    save the object status list data 
                                    (velocity -> m/s, heading -> rad으로 변환)
        '''     
        self.object_status_list = data

        # Object_status_list m/s 로 변환
        for i in range(data.num_of_npcs):
            self.object_status_list.npc_list[i].velocity.x = self.object_status_list.npc_list[i].velocity.x / 3.6
            self.object_status_list.npc_list[i].velocity.y = self.object_status_list.npc_list[i].velocity.y / 3.6
            self.object_status_list.npc_list[i].velocity.z = self.object_status_list.npc_list[i].velocity.z / 3.6
            self.object_status_list.npc_list[i].heading = np.deg2rad(self.object_status_list.npc_list[i].heading)

        for i in range(data.num_of_pedestrian):
            self.object_status_list.pedestrian_list[i].velocity.x = self.object_status_list.pedestrian_list[i].velocity.x / 3.6
            self.object_status_list.pedestrian_list[i].velocity.y = self.object_status_list.pedestrian_list[i].velocity.y / 3.6
            self.object_status_list.pedestrian_list[i].velocity.z = self.object_status_list.pedestrian_list[i].velocity.z / 3.6
            self.object_status_list.pedestrian_list[i].heading = np.deg2rad(self.object_status_list.pedestrian_list[i].heading)

        for i in range(data.num_of_obstacle):
            self.object_status_list.obstacle_list[i].velocity.x = self.object_status_list.obstacle_list[i].velocity.x / 3.6
            self.object_status_list.obstacle_list[i].velocity.y = self.object_status_list.obstacle_list[i].velocity.y / 3.6
            self.object_status_list.obstacle_list[i].velocity.z = self.object_status_list.obstacle_list[i].velocity.z / 3.6
            self.object_status_list.obstacle_list[i].heading = np.deg2rad(self.object_status_list.obstacle_list[i].heading)
        
        self.is_object_status_list = True