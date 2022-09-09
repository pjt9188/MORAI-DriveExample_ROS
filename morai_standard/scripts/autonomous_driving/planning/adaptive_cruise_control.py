#!/usr/bin/env python
# -*- coding: utf-8 -*-

from lattice_planner import LATTICE_BEHAVIOR_DICT

class AdaptiveCruiseControl:
    def __init__(self, velocity_gain, distance_gain, time_gap, vehicle_length):
        self.velocity_gain = velocity_gain
        self.distance_gain = distance_gain
        self.time_gap = time_gap
        self.vehicle_length = vehicle_length

        self.object_type = None
        self.object_distance = 0
        self.object_velocity = 0

        self.lattice_behavior = LATTICE_BEHAVIOR_DICT['lane_keeping']       # 'lane keeping', 'in_lane_change', 'lane_change'

    def check_object(self, local_path, object_info_dic_list, current_traffic_light):
        """경로상의 장애물 유무 확인 (차량, 사람, 정지선 신호)"""
        self.object_type = None
        min_relative_distance = float('inf')
        for object_info_dic in object_info_dic_list:
            object_info = object_info_dic['object_info']
            local_position = object_info_dic['local_position']

            object_type = object_info.type
            if object_type == 0:
                distance_threshold = 4.35
            elif object_type in [1, 2, -1]:     # object_type (0: Pedestrian, 1: NPC vehicle, 2: Static object (obstacle), -1: Ego-vehicle)
                distance_threshold = 2.5
            elif object_type == 3:
                if current_traffic_light and object_info.name == current_traffic_light[0] and not current_traffic_light[1] in [16, 48]:
                    distance_threshold = 9
                else:
                    continue
            else:
                continue

            for point in local_path:
                distance_from_path = point.distance(object_info.position)

                if distance_from_path < distance_threshold:     # 경로상에 있을 경우
                    relative_distance = local_position.distance()       # 자차량으로부터의 거리 확인
                    if relative_distance < min_relative_distance:       # 자차량으로부터 거리가 가장 가까운 경우
                        min_relative_distance = relative_distance       # 상대거리, type
                        self.object_type = object_type                  # type
                        self.object_distance = relative_distance - self.vehicle_length  # 차두로부터 거리
                        self.object_velocity = object_info.velocity     # object의 종방향 속도 저장

    def get_target_velocity(self, ego_vel, target_vel, lattice_behavior):
        out_vel = target_vel

        if self.object_type == 0:
            print("ACC ON_Person")
            default_space = 8
        elif self.object_type in [1, 2, -1]:        # object_type (0: Pedestrian, 1: NPC vehicle, 2: Static object (obstacle), -1: Ego-vehicle)
            print("ACC ON_Vehicle")
            default_space = 5
        elif self.object_type == 3:
            print("ACC ON_Traffic Light")
            default_space = 3
        else:
            # print("CC ON")
            return out_vel

        velocity_error = ego_vel - self.object_velocity     # velcoity error, ego가 빠를수록 커짐

        if ego_vel >= self.object_velocity:
            safe_distance = ego_vel*self.time_gap
        else:
            safe_distance = (ego_vel-self.object_velocity) * 0.3 \
                            + (ego_vel-self.object_velocity)**2/(2*7.02) \
                            + ego_vel*self.time_gap     # 안전거리 : 시스템지연거리, 젖은 노면조건 실현 가능 감속도, 감속후 time_gap
            
        distance_error = safe_distance - self.object_distance

        acceleration = -(self.velocity_gain*velocity_error + self.distance_gain*distance_error)
        out_vel = min(ego_vel+acceleration, target_vel)

        if self.object_type == 0 and (distance_error > 0): # 안전거리는 여유있지만, 전방에 사람이 있는 경우
            out_vel = out_vel - 5.

        # default_space 미만으로 거리가 짧고 lane keeping인 경우 정지
        if self.object_distance < default_space and lattice_behavior == LATTICE_BEHAVIOR_DICT['lane_keeping']: 
            out_vel = 0.
        
        # lattice behavior가 in-lane change 혹은 lane change인 경우 현 속도 유지
        if lattice_behavior != LATTICE_BEHAVIOR_DICT['lane_keeping']: 
            out_vel = ego_vel
        
        # 디버깅용
        if lattice_behavior == LATTICE_BEHAVIOR_DICT['lane_keeping']:
            print('lane keeping')
        elif lattice_behavior == LATTICE_BEHAVIOR_DICT['in_lane_change']:
            print('In lane change')
        else:
            print('lane change')

        return out_vel
