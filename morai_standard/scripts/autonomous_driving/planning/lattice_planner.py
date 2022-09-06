#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos,sin,pi,sqrt,pow,atan2
import numpy as np

from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList, ObjectStatus
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path

'''
lattice_planner은 충돌 회피 경로 생성 및 선택 예제입니다.
차량 경로상의 장애물을 탐색하여 충돌 여부의 판단은 지역경로(/local_path) 와 장애물 정보(/Object_topic)를 받아 판단합니다.
충돌이 예견될 경우 회피경로를 생성 및 선택 하고 새로운 지역경로(/lattice_path)를 Pulish합니다.

노드 실행 순서 
1. 경로상의 장애물 탐색
2. 좌표 변환 행렬 생성
3. 충돌회피 경로 생성
4. 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
5. 생성된 충돌회피 경로 중 낮은 비용의 경로 선택
6. 선택 된 새로운 지역경로 self.optimal_lattice_path 출력
'''
class LatticePlanner:
    """ Lattice Planner
        object vars:
            ego_vehicle_status
            object_status_list
            local_path : localization.point의 Point 객체의 list로 이루어진 local path
    """
    def __init__(self):
        self.ego_vehicle_status = EgoVehicleStatus()
        self.object_status_list = ObjectStatusList()
        self.local_path = []
        self.lattice_path_list = []
        self.optimal_lattice_path = None

        self.is_lattice_path_list = False
    
    def set_object_variables(self, ego_vehicle_status, object_status_list, local_path):
        self.ego_vehicle_status = ego_vehicle_status
        self.object_status_list = object_status_list
        
        # localization.point의 Point 객체의 list로 이루어진 local path를 nav_msg의 Path 객체 형태로 변환
        self.local_path = Path()
        self.local_path.header.frame_id = 'map'
        for point in local_path:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = Point(x=point.x, y=point.y, z=0)
            self.local_path.poses.append(pose_stamped)
    
    def get_optimal_lattice_path(self, ego_vehicle_status, object_status_list, local_path):
        """ local path와 ego status, object status를 통해서 최적의 lattice Path 계산
            Args: 
                ego_vehicle_status : morai msg - EgoVehicleStatus
                object_status_list : morai msg - ObjectStatusList
                local_path         : localization.point의 Point 객체의 list로 이루어진 local path

            return :
                optimal_lattice_path : nav_msgs의 Path 객체 형태의 최적 lattice path
        """
        self.set_object_variables(ego_vehicle_status, object_status_list, local_path)
        self.calc_lattice_path_list()
        
        # TODO : 생성한 Path list들 중에서 optimal path 선택
        if self.is_lattice_path_list:
            self.optimal_lattice_path = self.lattice_path_list[0]
        else:
            self.optimal_lattice_path = local_path

        return self.optimal_lattice_path
    
    def calc_lattice_path_list(self):
        # Input variable 받기
        ref_path = self.local_path
        lattice_path_list = []
        vehicle_pose_x = self.ego_vehicle_status.position.x
        vehicle_pose_y = self.ego_vehicle_status.position.y
        vehicle_velocity = self.ego_vehicle_status.velocity.x

        look_distance = int(vehicle_velocity * 1) *2     # look ahead distance(차량속도 기준 1초 간의 거리의) waypoint 개수

        
        if look_distance < 20 : # 최소 20개 / min 10m   
            look_distance = 20                    

        if len(ref_path.poses) > look_distance :  
            #TODO: (3) 좌표 변환 행렬 생성
            """
            # 좌표 변환 행렬을 만듭니다.
            # Lattice 경로를 만들기 위해서 경로 생성을 시작하는 Point 좌표에서 
            # 경로 생성이 끝나는 Point 좌표의 상대 위치를 계산해야 합니다.
            
            """          

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
            lane_off_set = [-3.0, -1.75, -1, 1, 1.75, 3.0]
            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
            
            #TODO: (4) Lattice 충돌 회피 경로 생성
            '''
            # Local 좌표계로 변경 후 3차곡선계획법에 의해 경로를 생성한 후 다시 Map 좌표계로 가져옵니다.
            # Path 생성 방식은 3차 방정식을 이용하며 lane_change_ 예제와 동일한 방식의 경로 생성을 하면 됩니다.
            # 생성된 Lattice 경로는 lattice_path_list 변수에 List 형식으로 넣습니다.
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

                lattice_path_list.append(lattice_path)

            # look_distance의 2배 길이(2초 길이) 혹은 ref_path 길이 만큼 뒤에 lattice planner 추가
            add_point_size = min(look_distance*2, len(ref_path.poses))           
            
            for i in range(look_distance, add_point_size):
                if i+1 < len(ref_path.poses):
                    tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y, \
                        ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)                    
                    tmp_translation = [ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],\
                                      [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]],\
                                      [0, 0, 1]])

                    for lane_num in range(len(lane_off_set)) :
                        local_result = np.array([[0], [lane_off_set[lane_num]], [1]])
                        global_result = tmp_t.dot(local_result)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = global_result[0][0]
                        read_pose.pose.position.y = global_result[1][0]
                        read_pose.pose.position.z = 0
                        read_pose.pose.orientation.x = 0
                        read_pose.pose.orientation.y = 0
                        read_pose.pose.orientation.z = 0
                        read_pose.pose.orientation.w = 1
                        lattice_path_list[lane_num].poses.append(read_pose)
                        
            #TODO: (5) 생성된 모든 Lattice 충돌 회피 경로 메시지 Publish
            '''
            # 생성된 모든 Lattice 충돌회피 경로는 ros 메세지로 송신하여
            # Rviz 창에서 시각화 하도록 합니다.

            '''
            for i in range(len(lattice_path_list)):          
                if 'lattice_pub_{}'.format(i+1) not in globals().keys():
                    globals()['lattice_pub_{}'.format(i+1)] = rospy.Publisher('/lattice_path_{}'.format(i+1),Path,queue_size=1)
                globals()['lattice_pub_{}'.format(i+1)].publish(lattice_path_list[i])
        
            self.lattice_path_list = lattice_path_list
            self.is_lattice_path_list = True
        else:
            self.is_lattice_path_list = False
    
