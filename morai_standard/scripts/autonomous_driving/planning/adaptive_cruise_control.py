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
        """local_path 경로상의 장애물 유무 확인 (차량, 사람, 정지선 신호)"""
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
                        self.object_velocity = object_info.velocity    # object의 종방향 속도 저장 (object status의 속도 단위 kph ->mps로 교체 필요)

    def get_target_velocity(self, ego_vel, target_vel, lattice_behavior):
        """ 
        check_object로 수행된 local_path 경로상의 전방 object 기준 속도계획
        1. target object 종류별 default space(최소 안전 거리) 설정
        2. 안전거리 설정 및 target speed 설정 : velocity error와 distance error 이용
        3. Target 속도 설정 - 안전거리와 Lattice Behavior 기반
        """
        out_vel = target_vel
        acc_behavior = LATTICE_BEHAVIOR_DICT['lane_keeping']

        # 1. target object 종류별 default space(최소 안전 거리) 설정
        if self.object_type == 0:
            print("ACC ON_Person")
            default_space = 8
        elif self.object_type in [1, 2, -1]:        # object_type (0: Pedestrian, 1: NPC vehicle, 2: Static object (obstacle), -1: Ego-vehicle)
            print("ACC ON_Vehicle")
            default_space = 5
        elif self.object_type == 3:
            print("ACC ON_Traffic Light")
            default_space = 3
        else: # 경로상에 없는 경우
            # print("CC ON")
            return out_vel, LATTICE_BEHAVIOR_DICT['lane_keeping']

        # 2.안전거리 설정 및 target speed 설정 : velocity error와 distance error 이용
        velocity_error = ego_vel - self.object_velocity     # velcoity error, ego가 빠를수록 커짐

        if ego_vel <= self.object_velocity:     # 전방 object가 더 빠른 경우
            safe_distance = ego_vel*self.time_gap + self.vehicle_length
        else:       # 전방 object가 더 느린 경우
            # 안전거리 : 시스템지연거리, 젖은 노면조건 실현 가능 감속도, 감속후 time_gap (부분자율주행의 안전기준). 차량 길이
            safe_distance = (ego_vel-self.object_velocity) * 0.3 \
                            + (ego_vel-self.object_velocity)**2/(2*7.02) \
                            + ego_vel*self.time_gap + self.vehicle_length
            
        distance_error = safe_distance - self.object_distance

        acceleration = -(self.velocity_gain*velocity_error + self.distance_gain*distance_error)
        out_vel = min(ego_vel+acceleration, target_vel)

        # 3. Target 속도 설정 - 안전거리와 Lattice Behavior 기반
        if distance_error < 0: # 안전거리 여유있을때 및 lane keeping 일 때, 앞차와의 속도 유지
            if lattice_behavior == LATTICE_BEHAVIOR_DICT['lane_keeping']:
                out_vel = self.object_velocity
                acc_behavior = LATTICE_BEHAVIOR_DICT['lane_keeping']
            else : # 안전거리 여유가 있고, in lane change / lane change 할 경우
                out_vel = target_vel
                acc_behavior = lattice_behavior

        # # 안전거리보다 짧고 lattice behavior에서도 충돌 회피 경로가 없을 때
        # elif distance_error >= 0:
        #     acc_behavior = LATTICE_BEHAVIOR_DICT['lane_keeping']
        #     out_vel = out_vel
        #     if self.object_type == 0:   # 전방 object가 사람인 경우
        #         out_vel = out_vel - 5.
        
        # elif self.object_distance < default_space:
        #     out_vel = 0.
    

        # 안전거리보다 짧고 lattice behavior에서도 충돌 회피 경로가 없을 때
        elif distance_error >= 0 and lattice_behavior == LATTICE_BEHAVIOR_DICT['all_colliding']: 
            print("ACC: 전부 충돌 {:.2f}".format(distance_error))
            acc_behavior = LATTICE_BEHAVIOR_DICT['all_colliding']
            out_vel = out_vel
            if self.object_type == 0:   # 전방 object가 사람인 경우
                out_vel = out_vel - 5.
            
            if self.object_distance < default_space:
                out_vel = 0.
        
        # 안전거리보다 짧고 lattice behavior에서 충돌 회피 경로가 있을 때
        elif distance_error >= 0 and lattice_behavior != LATTICE_BEHAVIOR_DICT['all_colliding']:     
        # lattice behavior가 in-lane change 혹은 lane change인 경우 현 속도 유지
            acc_behavior = lattice_behavior 
            out_vel = target_vel
        
        # 디버깅용
        # if lattice_behavior == LATTICE_BEHAVIOR_DICT['lane_keeping']:
        #     print('lane keeping')
        # elif lattice_behavior == LATTICE_BEHAVIOR_DICT['in_lane_change']:
        #     print('In lane change')
        # else:
        #     print('lane change')

        return out_vel, acc_behavior
