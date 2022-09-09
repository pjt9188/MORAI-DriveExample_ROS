#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos,sin,pi,sqrt,pow,atan2
import numpy as np

from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path

'''
lattice_planner은 충돌 회피 경로 생성 및 선택 예제입니다.
차량 경로상의 장애물을 탐색하여 충돌 여부의 판단은 지역경로(/local_path) 와 장애물 정보(/Object_topic)를 받아 판단합니다.
충돌이 예견될 경우 회피경로를 생성 및 선택 하고 새로운 지역경로(/lattice_path)를 Pulish합니다.

노드 실행 순서 
1. subscriber, publisher 선언
2. 경로상의 장애물 탐색
3. 좌표 변환 행렬 생성
4. 충돌회피 경로 생성
5. 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
6. 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
7. 선택 된 새로운 지역경로 (/lattice_path) return
'''
LATTICE_BEHAVIOR_DICT = dict(lane_keeping = 0, in_lane_change = 1, lane_change = 2)

class LatticePlanner:    
    def __init__(self):
        self.ego_vehicle_status = EgoVehicleStatus()
        self.local_path = Path()
        self.object_status_list = ObjectStatusList()
        self.behavior_state = LATTICE_BEHAVIOR_DICT['lane_keeping']
        self.lane_off_set = [0, -3.5, -1.75, -1, 1, 1.75, 3.5]
        self.lattice_behavior = [0, 2, 1, 1, 1, 1, 2]

    def get_lattice_path(self, ego_vehicle_status, local_path, object_status_list):
        self.ego_vehicle_status = ego_vehicle_status
        self.local_path = convert_to_ros_path(local_path, 'map')
        self.object_status_list = object_status_list
        
        lattice_path = self.latticePlanner()
        if self.checkObject(self.local_path, self.object_status_list):      # centerline path(local path)의 경로상 장애물 탐색
            lattice_path_index = self.checkCollision(self.object_status_list, lattice_path)
            
            # TODO: (7) lattice 경로 return
            return lattice_path[lattice_path_index], self.lattice_behavior[lattice_path_index]
        else:
            return lattice_path[0], self.lattice_behavior[0]

    def checkObject(self, ref_path, object_status_list):
        #TODO: (2) 경로의 waypoint 별 장애물 탐색
        is_crash = False
        for object_status in object_status_list.npc_list + object_status_list.obstacle_list + object_status_list.pedestrian_list:
            for path in ref_path.poses: 
                distance = np.hypot(object_status.position.x - path.pose.position.x, object_status.position.y - path.pose.position.y)
                if distance < 2.35: # 장애물의 좌표값이 지역 경로 상의 좌표값과의 직선거리가 2.35 미만일때 충돌이라 판단.
                    is_crash = True
                    break
        
        return is_crash
    
    def checkCollision(self, object_status_list, lattice_path):
        #TODO: (6) 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
        selected_lane_idx = -1
        lane_weight = [0,3,2,1,1,2,3] # reference path

        # object의 현재 위치 기준 path의 위험도 계산
        for object in object_status_list.npc_list + object_status_list.obstacle_list + object_status_list.pedestrian_list:
            for lattice_path_idx in range(len(lattice_path)):
                for path_pose in lattice_path[lattice_path_idx].poses:
                    distance = np.hypot(object.position.x - path_pose.pose.position.x, object.position.y - path_pose.pose.position.y)
                    if distance < 2.35:
                        lane_weight[lattice_path_idx] += 100

        selected_lane_idx = lane_weight.index(min(lane_weight))
        return selected_lane_idx
        


    def latticePlanner(self):
        ref_path = self.local_path
        out_path = []
        vehicle_pose_x = self.ego_vehicle_status.position.x
        vehicle_pose_y = self.ego_vehicle_status.position.y
        vehicle_velocity = self.ego_vehicle_status.velocity.x

        look_distance = int(vehicle_velocity * 1) *2     # look ahead distance(차량속도 기준 1초 간의 거리[m]의) 0.5m 간격 waypoint 개수

        
        if look_distance < 20 : # 최소 20개 / min 10m   
            look_distance = 20                    

        if len(ref_path.poses) > look_distance :  
            #TODO: (3) 좌표 변환 행렬 생성
            """
            # 좌표 변환 행렬을 만듭니다.
            # Lattice 경로를 만들기 위해서 경로 생성을 시작하는 Point 좌표에서 
            # 경로 생성이 끝나는 Point 좌표의 상대 위치를 계산해야 합니다.
            
            """          

            # 고쳐야할 TODO: ref Path에 중복되는 waypoint 있음 아래 방법 수정 필요
            global_ref_start_point      = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)

            global_ref_end_point = (ref_path.poses[look_distance].pose.position.x, ref_path.poses[look_distance].pose.position.y)   # look ahead distance 2배 만큼
            
            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]
            
            # local_path2global
            trans_matrix    = np.array([    [cos(theta),                -sin(theta),                                                                      translation[0]], 
                                            [sin(theta),                 cos(theta),                                                                      translation[1]], 
                                            [         0,                          0,                                                                                  1 ]     ])

            # global2local_path
            det_trans_matrix = np.array([   [trans_matrix[0][0], trans_matrix[1][0],        -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                            [trans_matrix[0][1], trans_matrix[1][1],        -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                            [                 0,                  0,                                                                                   1]     ])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])     # ref_path의 endpoint
            local_end_point = det_trans_matrix.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)
            local_lattice_points = []
            
            for i in range(len(self.lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + self.lane_off_set[i], 1])
            
            #TODO: (4) Lattice 충돌 회피 경로 생성 -> cubit spline으로 변경 필요
            '''
            # Local 좌표계로 변경 후 3차곡선계획법에 의해 경로를 생성한 후 다시 Map 좌표계로 가져옵니다.
            # Path 생성 방식은 3차 방정식을 이용하며 lane_change_ 예제와 동일한 방식의 경로 생성을 하면 됩니다.
            # 생성된 Lattice 경로는 out_path 변수에 List 형식으로 넣습니다.
            # 충돌 회피 경로는 기존 경로를 제외하고 좌 우로 3개씩 총 6개의 경로를 가지도록 합니다.

            '''
                
            for end_point in local_lattice_points :
                lattice_path = Path()
                lattice_path.header.frame_id = 'map'
                x = []
                y = []
                x_interval = 0.5
                xs = 0
                xf = end_point[0]
                ps = local_ego_vehicle_position[1][0]
                pf = end_point[1]
                x_num = xf / x_interval

                # x_interval 간격 x 위치
                for i in range(xs,int(x_num)) : 
                    x.append(i*x_interval)
                
                # 3차 곡선 계획의 계수 [B.C. : y(x=0)=0, y(xf) = yf, dydx(x=0) = 0, dydx(xf)=0]
                a = [0.0, 0.0, 0.0, 0.0]
                a[0] = ps
                a[1] = 0
                a[2] = 3.0 * (pf - ps) / (xf * xf)
                a[3] = -2.0 * (pf - ps) / (xf * xf * xf)
                
                # 
                for i in x:
                    result = a[3] * i * i * i + a[2] * i * i + a[1] * i + a[0]
                    y.append(result)

                for i in range(0,len(y)) :
                    local_result = np.array([[x[i]], [y[i]], [1]])
                    global_result = trans_matrix.dot(local_result)

                    read_pose = PoseStamped()
                    read_pose.pose.position.x = global_result[0][0]
                    read_pose.pose.position.y = global_result[1][0]
                    read_pose.pose.position.z = 0
                    read_pose.pose.orientation.x = 0
                    read_pose.pose.orientation.y = 0
                    read_pose.pose.orientation.z = 0
                    read_pose.pose.orientation.w = 1
                    lattice_path.poses.append(read_pose)

                out_path.append(lattice_path)

            # look_distance의 2배 길이(2초 길이) 혹은 ref_path 길이 만큼 뒤에 lattice planner 추가
            add_point_size = min(look_distance*2, len(ref_path.poses))           
            
            for i in range(look_distance, add_point_size):
                if i+1 < len(ref_path.poses): # 앞의 waypoint가 존재한다면
                    tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y, \
                        ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)                    
                    tmp_translation = [ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],\
                                      [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]],\
                                      [0, 0, 1]])

                    for lane_num in range(len(self.lane_off_set)) :
                        local_result = np.array([[0], [self.lane_off_set[lane_num]], [1]])
                        global_result = tmp_t.dot(local_result)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = global_result[0][0]
                        read_pose.pose.position.y = global_result[1][0]
                        read_pose.pose.position.z = 0
                        read_pose.pose.orientation.x = 0
                        read_pose.pose.orientation.y = 0
                        read_pose.pose.orientation.z = 0
                        read_pose.pose.orientation.w = 1
                        out_path[lane_num].poses.append(read_pose)
                        
            #TODO: (5) 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
            '''
            # 생성된 모든 Lattice 충돌회피 경로는 ros 메세지로 송신하여
            # Rviz 창에서 시각화 하도록 합니다.

            '''
            for i in range(len(out_path)):          
                globals()['lattice_pub_{}'.format(i+1)] = rospy.Publisher('/lattice_path_{}'.format(i+1),Path,queue_size=1)
                globals()['lattice_pub_{}'.format(i+1)].publish(out_path[i])
        
        return out_path


def convert_to_ros_path(path, frame_id):
    ros_path = Path()
    ros_path.header.frame_id = frame_id
    for point in path:
        pose_stamped = PoseStamped()
        pose_stamped.pose.position = Point(x=point.x, y=point.y, z=0)
        pose_stamped.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
        ros_path.poses.append(pose_stamped)

    return ros_path