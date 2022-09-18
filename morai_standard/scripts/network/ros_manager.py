#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList, CtrlCmd, GetTrafficLightStatus, SetTrafficLight
from autonomous_driving.vehicle_state import VehicleState
from autonomous_driving.perception.object_info import ObjectInfo
from autonomous_driving.config.config import Config

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA

class RosManager:
    def __init__(self, autonomous_driving, config_file = 'config.json'):
        self.autonomous_driving = autonomous_driving

        config = Config(config_file)
        rospy.init_node("morai_standard", anonymous=True)

        self.traffic_light_control = config["map"]["traffic_light_control"]
        self.global_path = self.convert_to_ros_path(config["map"]["path"], '/map')

        self.sampling_rate = config["common"]["sampling_rate"]
        self.ros_rate = rospy.Rate(self.sampling_rate)
        self.count = 0

        self.vehicle_state = VehicleState()
        self.ego_vehicle_status = EgoVehicleStatus()
        self.object_info_list = []
        self.object_status_list = ObjectStatusList()
        self.traffic_light = []
        self.rviz_markers = MarkerArray()

        self.is_status = False
        self.is_object_info = False
        self.is_traffic_light = False

    def execute(self):
        print("start simulation")
        self._set_protocol()
        while not rospy.is_shutdown():
            self._set_rviz_visualization()
            if self.is_status and self.is_object_info:
                control_input, local_path, lattice_path = self.autonomous_driving.execute(
                    self.vehicle_state, self.ego_vehicle_status, self.object_info_list, self.object_status_list, self.traffic_light,
                )
                self._send_data(control_input, local_path, lattice_path)
        print("end simulation")

    def _set_protocol(self):
        # publisher
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.lattice_path_pub = rospy.Publisher('/lattice_path', Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.traffic_light_pub = rospy.Publisher("/SetTrafficLight", SetTrafficLight, queue_size=1)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.rviz_vehicles_pub = rospy.Publisher('/rviz_vehicles', MarkerArray, queue_size = 1)

        # subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.vehicle_status_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_info_callback)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_light_callback)
    
    def _send_data(self, control_input, local_path, lattice_path):
        self.ctrl_pub.publish(CtrlCmd(**control_input.__dict__))
        self.local_path_pub.publish(self.convert_to_ros_path(local_path, 'map'))
        self.lattice_path_pub.publish(self.convert_to_ros_path(lattice_path, 'map'))
        self.odom_pub.publish(self.convert_to_odometry(self.vehicle_state))

        if self.count == self.sampling_rate:
            self.global_path_pub.publish(self.global_path)
            self.count = 0
        self.count += 1
        self.ros_rate.sleep()

    def _set_rviz_visualization(self):
        # self.rviz_path_pub = rospy.Publisher('/rviz_path', MarkerArray, queue_size = 1)
        self.rviz_markers = MarkerArray()
        if self.is_object_info:
            if self.object_status_list.num_of_npcs > 0:
                for idx, npc_object_status in enumerate(self.object_status_list.npc_list):
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.ns = 'npc'
                    marker.id = idx+1
                    marker.type = marker.CUBE
                    marker.action = marker.ADD      # 항상 새로 만드는 것은 아니고 ADD와 MODIFY는 동일하다 
                                                    # http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html                                                    
                    marker.pose.position = Point(x = npc_object_status.position.x, 
                                                 y = npc_object_status.position.y,
                                                 z = 0
                                                )
                    quaternion = tf.transformations.quaternion_from_euler(0, 0, npc_object_status.heading)
                    marker.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
                    marker.scale = npc_object_status.size
                    marker.color = ColorRGBA(1.0, 0.67, 0.11, 1.0)
                    
                    self.rviz_markers.markers.append(marker)

        if self.is_status:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns = 'ego'
            marker.id = 0
            marker.type = marker.CUBE
            marker.action = marker.ADD      # 항상 새로 만드는 것은 아니고 ADD와 MODIFY는 동일하다 
                                            # http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html                                                    
            marker.pose.position = Point(x = self.ego_vehicle_status.position.x, 
                                         y = self.ego_vehicle_status.position.y,
                                         z = 0
                                        )
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.ego_vehicle_status.heading)
            marker.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
            marker.scale = Vector3(4.65, 1.88, 1.51)    # IONIQ 기준
            marker.color = ColorRGBA(0.427, 0.965, 0.918, 1.0)
            
            self.rviz_markers.markers.append(marker)

        
        self.rviz_vehicles_pub.publish(self.rviz_markers)

    @staticmethod
    def convert_to_ros_path(path, frame_id):
        ros_path = Path()
        ros_path.header.frame_id = frame_id
        for point in path:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = Point(x=point.x, y=point.y, z=0)
            pose_stamped.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
            ros_path.poses.append(pose_stamped)

        return ros_path

    @staticmethod
    def convert_to_odometry(vehicle_state):
        odometry = Odometry()
        odometry.header.frame_id = 'map'
        odometry.child_frame_id = 'gps'

        quaternion = tf.transformations.quaternion_from_euler(0, 0, vehicle_state.yaw)

        odometry.pose.pose.position = Point(x=vehicle_state.position.x, y=vehicle_state.position.y, z=0)
        odometry.pose.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        return odometry

    def vehicle_status_callback(self, data):
        '''vehicle status callback

        Parameters
        ---------------------
        data : msg(EgoVehicleStatus)
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
        self.vehicle_state : autonomous_driving.vehicle_state.VehicleState
                                position : localization.point.Point(x, y)
                                yaw : vehicle global yaw[rad]
                                velcoity : vehicle longitudinal velocity [m/s]

        self.ego_vehicle_status : morai_msgs.msg.EgoVehicleStatus
                                    save the original ego vehicle status data (heading -> rad으로 변경함)
        '''     
        self.vehicle_state = VehicleState(data.position.x, data.position.y, np.deg2rad(data.heading), data.velocity.x)
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (self.vehicle_state.position.x, self.vehicle_state.position.y, 0),
            tf.transformations.quaternion_from_euler(0, 0, self.vehicle_state.yaw),
            rospy.Time.now(),
            "gps",
            "map"
        )

        # Ego Vehicle Status heading deg -> rad
        self.ego_vehicle_status = data      # class instance의 경우 복사가 이루어지지 않고, 객체를 참조함 (data와 self.ego_vehicle_status가 동일해짐)
        self.ego_vehicle_status.heading = np.deg2rad(self.ego_vehicle_status.heading) # data의 heading을 rad으로 변환

        self.is_status = True

    def object_info_callback(self, data):
        '''object info callback

        Parameters
        ---------------------
        data : morai_msgs.msg.EgoVehicleStatus
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
        self.object_info_list = [
            ObjectInfo(object_status.position.x, object_status.position.y, object_status.velocity.x / 3.6, object_status.type)
            for object_status in data.npc_list + data.obstacle_list + data.pedestrian_list
        ]
        
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
        
        self.is_object_info = True

    def traffic_light_callback(self, data):
        # traffic_control (차량 신호 Green Light(16) 변경)
        if self.traffic_light_control:
            traffic_light_status = 16
            set_traffic_light = SetTrafficLight(
                trafficLightIndex=data.trafficLightIndex,
                trafficLightStatus=traffic_light_status
            )
            self.traffic_light_pub.publish(set_traffic_light)
        else:
            traffic_light_status = data.trafficLightStatus

        self.traffic_light = [data.trafficLightIndex, traffic_light_status]
        self.is_traffic_light = True
