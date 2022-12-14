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
        self.local_path = Path()

        self.is_status = False
        self.is_ego_vehicle_status = False
        self.is_object_info = False
        self.is_object_status_list = False
        self.is_traffic_light = False
        self.is_local_path = False

    def execute(self):
        print("start simulation")
        self._set_protocol()
        while not rospy.is_shutdown():
            if self.is_status and self.is_object_info and self.is_ego_vehicle_status and self.is_object_status_list:
                control_input, local_path, optimal_lattice_path = self.autonomous_driving.execute(
                    self.vehicle_state, self.ego_vehicle_status, self.object_info_list, self.object_status_list, self.traffic_light
                )
                self._send_data(control_input, local_path, optimal_lattice_path)
        print("end simulation")

    def _set_protocol(self):
        # publisher
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.optimal_lattice_path_pub = rospy.Publisher('/optimal_lattice_path', Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.traffic_light_pub = rospy.Publisher("/SetTrafficLight", SetTrafficLight, queue_size=1)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)

        # subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.vehicle_status_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_info_callback)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_light_callback)

    def _send_data(self, control_input, local_path, optimal_lattice_path):
        """publsh control input, local path, optimal lattice path"""
        self.ctrl_pub.publish(CtrlCmd(**control_input.__dict__))
        self.local_path_pub.publish(self.convert_to_ros_path(local_path, 'map'))
        self.optimal_lattice_path_pub.publish(optimal_lattice_path)
        self.odom_pub.publish(self.convert_to_odometry(self.ego_vehicle_status))

        if self.count == self.sampling_rate:
            self.global_path_pub.publish(self.global_path)
            self.count = 0
        self.count += 1
        self.ros_rate.sleep()

    @staticmethod
    def convert_to_ros_path(path, frame_id):
        """ path to ros msg 'Path'
            Args :
                path : localization.point??? Point ????????? ???????????? path
                frame_id : frame_ id
            Returns :
                ros_path : rosmsg 'Path'??? ????????? ??????
        """
        ros_path = Path()
        ros_path.header.frame_id = frame_id
        for point in path:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = Point(x=point.x, y=point.y, z=0)
            pose_stamped.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
            ros_path.poses.append(pose_stamped)

        return ros_path

    @staticmethod
    def convert_to_odometry(ego_vehicle_status):
        odometry = Odometry()
        odometry.header.frame_id = 'map'
        odometry.child_frame_id = 'gps'

        quaternion = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(ego_vehicle_status.heading))

        odometry.pose.pose.position = Point(x=ego_vehicle_status.position.x, y=ego_vehicle_status.position.y, z=ego_vehicle_status.position.z)
        odometry.pose.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        return odometry

    def vehicle_status_callback(self, data):
        '''vehicle status callback
        topic   : /Ego_topic
        msg     : EgoVehicle Status
            position    : x,y,z position in global coordintae
            velocity    : x(longitudinal), y(lateral), z, velocity [m/s] in vehicle coordintae
            accleration : x, y, z, acceleration [m/s^2] in vehicle coordintae

            heading     : vehicle heading [deg]
            accel       : gas pedal [0~1]
            brake       : brake     [0~1]
            wheel_angle : steering wheel angle [deg]
        '''
        self.vehicle_state = VehicleState(data.position.x, data.position.y, np.deg2rad(data.heading), data.velocity.x)
        self.ego_vehicle_status = data
        # print(self.ego_vehicle_status)
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (self.vehicle_state.position.x, self.vehicle_state.position.y, 0),
            tf.transformations.quaternion_from_euler(0, 0, self.vehicle_state.yaw),
            rospy.Time.now(),
            "gps",
            "map"
        )
        self.is_status = True
        self.is_ego_vehicle_status = True

    def object_info_callback(self, data):
        self.object_info_list = [
            ObjectInfo(data.position.x, data.position.y, data.velocity.x, data.type)
            for data in data.npc_list + data.obstacle_list + data.pedestrian_list
        ]
        self.object_status_list = data

        self.is_object_info = True
        self.is_object_status_list = True

    def traffic_light_callback(self, data):
        # traffic_control (?????? ?????? Green Light(16) ??????)
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
