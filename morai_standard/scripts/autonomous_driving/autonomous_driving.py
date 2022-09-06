#!/usr/bin/env python
# -*- coding: utf-8 -*-
from .perception.forward_object_detector import ForwardObjectDetector
from .localization.path_manager import PathManager
from .planning.adaptive_cruise_control import AdaptiveCruiseControl
from .control.pure_pursuit import PurePursuit
from .control.pid import Pid
from .control.control_input import ControlInput
from .config.config import Config

from .mgeo.calc_mgeo_path import mgeo_dijkstra_path

from nav_msgs.msg import Path
from .planning.lattice_planner import LatticePlanner


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

    def execute(self, vehicle_state, ego_vehicle_status, dynamic_object_list, object_status_list, current_traffic_light):
        """ 현재 위치 기반으로 control_input(steering, acc, brake)와 local_path, optimal latticel path를 출력
            Args:
                vehicle_state : vehicle_state.VehicleState 객체 형태의 ego vehicle 차량 정보(Point(x, y), yaw, velocity)
                ego_vehicle_status : morai msg - EgoVehicleStatus 
                dynamic_object_list : perception.object_info.ObjectInfo 객체의 object info list들
                object_status_list : morai_msg - ObjectStatusList
                current_traffic_light :

            Returns:
                ControlInput : acc_cmd, steering_cmd

        """
        # 현재 위치 기반으로 local path과 planned velocity 추출
        local_path, planned_velocity = self.path_manager.get_local_path(vehicle_state)

        # TODO: Lattice Planner 기반 최적 path
        optimal_lattice_path = self.lattice_planner.get_optimal_lattice_path(ego_vehicle_status, object_status_list, local_path)

        # 전방 장애물 인지
        self.forward_object_detector._dynamic_object_list = dynamic_object_list
        object_info_dic_list = self.forward_object_detector.detect_object(vehicle_state)

        # adaptive cruise control를 활용한 속도 계획
        self.adaptive_cruise_control.check_object(local_path, object_info_dic_list, current_traffic_light)
        target_velocity = self.adaptive_cruise_control.get_target_velocity(vehicle_state.velocity, planned_velocity)

        # 속도 제어를 위한 PID control
        acc_cmd = self.pid.get_output(target_velocity, vehicle_state.velocity)
        # target velocity가 0이고, 일정 속도 이하일 경우 full brake를 하여 차량을 멈추도록 함.
        if round(target_velocity) == 0 and vehicle_state.velocity < 2:
            acc_cmd = -1.
        # 경로 추종을 위한 pure pursuit control
        self.pure_pursuit.path = local_path
        self.pure_pursuit.vehicle_state = vehicle_state
        steering_cmd = self.pure_pursuit.calculate_steering_angle()

        return ControlInput(acc_cmd, steering_cmd), local_path, optimal_lattice_path
