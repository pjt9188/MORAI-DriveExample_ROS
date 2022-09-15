#!/usr/bin/env python
# -*- coding: utf-8 -*-
from .perception.forward_object_detector import ForwardObjectDetector
from .localization.point import Point
from .localization.path_manager import PathManager
from .planning.adaptive_cruise_control import AdaptiveCruiseControl
from .planning.lattice_planner import LATTICE_BEHAVIOR_DICT, LatticePlanner
from .control.pure_pursuit import PurePursuit
from .control.pid import Pid
from .control.control_input import ControlInput
from .config.config import Config

from .mgeo.calc_mgeo_path import mgeo_dijkstra_path

import matplotlib.pyplot as plt

class AutonomousDriving:
    def __init__(self, config_file = 'config.json'):
        config = Config(config_file = config_file)

        if config["map"]["use_mgeo_path"]:
            mgeo_path = mgeo_dijkstra_path(config["map"]["name"])
            self.path = mgeo_path.calc_dijkstra_path(config["map"]["mgeo"]["start_node"], config["map"]["mgeo"]["end_node"])
            self.path_manager = PathManager(
                self.path, config["map"]["is_closed_path"], config["map"]["local_path_size"]
            )
        else:
            self.path = config["map"]["path"]
            self.path_manager = PathManager(
                self.path , config["map"]["is_closed_path"], config["map"]["local_path_size"]
            )
        self.path_manager.set_velocity_profile(**config['planning']['velocity_profile'])
        self.lattice_planner = LatticePlanner()

        self.forward_object_detector = ForwardObjectDetector(config["map"]["traffic_light_list"])

        self.adaptive_cruise_control = AdaptiveCruiseControl(
            vehicle_length=config['common']['vehicle_length'], **config['planning']['adaptive_cruise_control']
        )
        self.pid = Pid(sampling_time=1/float(config['common']['sampling_rate']), **config['control']['pid'])
        self.pure_pursuit = PurePursuit(
            wheelbase=config['common']['wheelbase'], **config['control']['pure_pursuit']
        )

        self.behavior_state = 0
        self.prev_acc_cmd = 0
        self.prev_target_velocity = 0
    
    def execute(self, vehicle_state, ego_vehicle_status, dynamic_object_list, object_status_list, current_traffic_light):
        # 현재 위치 기반으로 local path과 planned velocity 추출
        local_path, planned_velocity = self.path_manager.get_local_path(vehicle_state)

        # Lattice Path에서 장애물 회피 경로 생성
        lattice_path, lattice_behavior = self.lattice_planner.get_lattice_path(ego_vehicle_status, local_path, object_status_list)
        lattice_path = convert_to_point_path(lattice_path)

        # 전방 장애물 인지
        self.forward_object_detector._dynamic_object_list = dynamic_object_list
        object_info_dic_list = self.forward_object_detector.detect_object(vehicle_state)

        # adaptive cruise control를 활용한 속도 계획
        self.adaptive_cruise_control.check_object(local_path, object_info_dic_list, current_traffic_light)
        target_velocity, acc_behavior = self.adaptive_cruise_control.get_target_velocity(vehicle_state.velocity, planned_velocity, lattice_behavior)

        # 속도 제어를 위한 PID control
        acc_cmd = self.pid.get_output(target_velocity, vehicle_state.velocity)

        # print("Lattice behavior {}".format(lattice_behavior))
        # print("ACC behavior {}".format(acc_behavior))
        # print("target_velocity : {:.2f}, vehicle_state.velocity :{:.2f}".format(target_velocity, vehicle_state.velocity))
        # print("acc_cmd : {:.2f}".format(acc_cmd))    

        if acc_behavior != LATTICE_BEHAVIOR_DICT['lane_keeping']:
            print("Lattice behavior {}".format(lattice_behavior))
            print("ACC behavior {}".format(acc_behavior))
            print("target_velocity : {:.2f}, vehicle_state.velocity :{:.2f}".format(target_velocity, vehicle_state.velocity))
            print("acc_cmd : {:.2f}".format(acc_cmd))    

        # target velocity가 0이고, 일정 속도 이하일 경우 full brake를 하여 차량을 멈추도록 함.
        if round(target_velocity) == 0 and vehicle_state.velocity < 2:
            acc_cmd = -1.
        # 경로 추종을 위한 pure pursuit control
        if acc_behavior == LATTICE_BEHAVIOR_DICT['all_colliding']:
            self.pure_pursuit.path = local_path
        else:
            self.pure_pursuit.path = lattice_path
            
        self.pure_pursuit.vehicle_state = vehicle_state
        steering_cmd = self.pure_pursuit.calculate_steering_angle()

        return ControlInput(acc_cmd, steering_cmd), local_path, lattice_path


def convert_to_point_path(path):
    point_path = []
    for pose_stamped in path.poses:
        point = Point(pose_stamped.pose.position.x, pose_stamped.pose.position.y)
        point_path.append(point)

    return point_path