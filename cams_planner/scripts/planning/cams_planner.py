#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from .cubic_spline_planner import calc_spline_course

import numpy as np
from .point import Point as MoraiPoint

class CAMSPlanner:
    """
    Trajectory Planner for Collision Avoidance Mitigation System
    """

    def __init__(self):
        self.global_cs2d = None
        self.ego_state = None
        self.ego_state_frenet = None        # s, ds, dds, l, dl, ddl
        

    def execute(self, ego_vehicle_status, object_status_list, global_path, local_path):
        """
        Generate cams path

        params
        --------------------
        ego_vehicle_status : morai_msgs.msg.EgoVehicleStatus
        object_status_list : morai_msgs.msg.ObjectStatusList
        global_path        : nav_msgs.msg.Path
        local_path         : nav_msgs.msg.Path
                             (http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Path.html)
        """
        self.ego_state = ego_vehicle_status
        
        lx, ly = self.convert_to_list_path(local_path)
        rx, ry, ryaw, rk, s, self.global_cs2d = calc_spline_course(lx, ly, ds = 0.5)

        # MORAI Point Path로 변환
        ref_path = []
        for i in range(len(rx)):
            point = MoraiPoint(rx[i], ry[i])
            ref_path.append(point)

        self.ego_state_frenet = self.convert_to_frenet_state(ego_vehicle_status, rx, ry, ryaw, rk, s)

        cams_path = self.convert_to_ros_path(ref_path)
        cams_path_candidates = []

        path_candidate = self.get_cams_path_candidates(s, self.global_cs2d)
        cams_path_candidates.append(path_candidate)
        
        return cams_path, cams_path_candidates

    def convert_to_frenet_state(self, vehicle_status, ref_x, ref_y, ref_yaw, ref_curv, ref_s):
        # Need to be advanced
        """
        convert to frenet state

        parameters
        ------------------
        vehicle_status : morai_msgs.msg.EgoVehicleStatus or ObjectStatus

        """
        x_glob = vehicle_status.position.x
        y_glob = vehicle_status.position.y
        dx_bodyframe = vehicle_status.velocity.x
        dy_bodyframe = vehicle_status.velocity.y
        ddx_bodyframe = vehicle_status.acceleration.x
        ddy_bodyframe = vehicle_status.acceleration.y
        yaw_glob = vehicle_status.heading
        
        # spline sampling된 local path 중 가장 가까운 waypoint 찾기
        closest_idx = 0
        min_dist = float('inf')
        for ref_idx in range(len(ref_x)):
            dist = np.hypot(x_glob - ref_x[ref_idx], y_glob - ref_y[ref_idx])
            if dist < min_dist:
                closest_idx = ref_idx
                min_dist = dist

        s      = ref_s[closest_idx] + \
                      np.dot([x_glob - ref_x[closest_idx], y_glob - ref_y[closest_idx]],\
                             [np.cos(ref_yaw[closest_idx]), np.sin(ref_yaw[closest_idx])])
        # if s < 0:      # delay로 인해서 현재 s 위치가 음수로 나오는 경우 0로 바꿈
        #     s = 0
        ds      = np.dot([dx_bodyframe * np.cos(yaw_glob), dx_bodyframe * np.sin(yaw_glob)], \
                            [np.cos(ref_yaw[closest_idx]), np.sin(ref_yaw[closest_idx])]) + \
                    np.dot([dy_bodyframe * np.cos(yaw_glob+np.pi/2.0), dy_bodyframe * np.sin(yaw_glob+np.pi/2.0)], \
                            [np.cos(ref_yaw[closest_idx]), np.sin(ref_yaw[closest_idx])])

        dds   = np.dot([ddx_bodyframe * np.cos(yaw_glob), ddx_bodyframe * np.sin(yaw_glob)], \
                        [np.cos(ref_yaw[closest_idx]), np.sin(ref_yaw[closest_idx])]) + \
                np.dot([ddy_bodyframe * np.cos(yaw_glob+np.pi/2.0), ddy_bodyframe * np.sin(yaw_glob+np.pi/2.0)], \
                        [np.cos(ref_yaw[closest_idx]), np.sin(ref_yaw[closest_idx])])
        
        l      = np.dot([x_glob - ref_x[closest_idx], y_glob - ref_y[closest_idx]], \
                        [np.cos(ref_yaw[closest_idx]+np.pi/2.0), np.sin(ref_yaw[closest_idx]+np.pi/2.0)])

        dl    = np.dot([dx_bodyframe * np.cos(yaw_glob), dx_bodyframe * np.sin(yaw_glob)], \
                        [np.cos(ref_yaw[closest_idx]+np.pi/2.0), np.sin(ref_yaw[closest_idx]+np.pi/2.0)]) + \
                np.dot([dy_bodyframe * np.cos(yaw_glob+np.pi/2.0), dy_bodyframe * np.sin(yaw_glob+np.pi/2.0)], \
                        [np.cos(ref_yaw[closest_idx]+np.pi/2.0), np.sin(ref_yaw[closest_idx]+np.pi/2.0)])

        ddl   = np.dot([ddx_bodyframe * np.cos(yaw_glob), ddx_bodyframe * np.sin(yaw_glob)], \
                        [np.cos(ref_yaw[closest_idx]+np.pi/2.0), np.sin(ref_yaw[closest_idx]+np.pi/2.0)]) + \
                np.dot([ddy_bodyframe * np.cos(yaw_glob+np.pi/2.0), ddy_bodyframe * np.sin(yaw_glob+np.pi/2.0)], \
                        [np.cos(ref_yaw[closest_idx]+np.pi/2.0), np.sin(ref_yaw[closest_idx]+np.pi/2.0)])
        
        # Test ego status in frenet 
        # print("vehicle global status :\n{}\n{}\n".format([x_glob, dx_bodyframe, ddx_bodyframe],[y_glob, dy_bodyframe, ddy_bodyframe]))
        # print("vehicle frenet status :\n{}\n{}\n".format([s, ds, dds],[l, dl, ddl]))
        return (s, ds, dds, l, dl, ddl)


    def get_cams_path_candidates(self, s_list, cs2d):
        s_i = max(self.ego_state_frenet[0], 0)
        s_f = max(s_list)

        l_i = self.ego_state_frenet[3]
        l_f = 1.0
        
        theta_ego = self.ego_state.heading
        theta_i   = cs2d.calc_yaw(s_i)

        A = np.array([[1, s_i, s_i**2, s_i**3],
                      [0,   1, 2*s_i, 3*s_i**2],
                      [1, s_f, s_f**2, s_f**3],
                      [0, 1,   2*s_f, 3*s_f**2]
                      ])
        b = np.array([l_i, np.tan(theta_ego-theta_i), l_f, 0])
        a = np.linalg.solve(A, b)
        
        l_list = []
        for s in s_list:
            l = a[0] + a[1]*s + a[2]*s**2 + a[3]*s**3
            l_list.append(l)
        
        # for l in l_list:
        #     print("{:.2f} ".format(l), end=' ')
        # print("\n")

        path_candidate = self.calc_global_paths(s_list, l_list, cs2d)
        path_candidate = self.convert_to_ros_path(path_candidate)

        return path_candidate

        



    def calc_global_paths(self, s_list, l_list, cs2d):
        point_path = []
        for i in range(len(s_list)):
            s = s_list[i]
            l = l_list[i]

            rx, ry = cs2d.calc_position(s)
            if rx is None:
                raise ValueError("s is out of bound - calc_global_path")
            
            r_yaw = cs2d.calc_yaw(s)

            px = rx + l * np.cos(r_yaw + np.pi / 2.0)
            py = ry + l * np.sin(r_yaw + np.pi / 2.0)

            point = MoraiPoint(px, py)
            point_path.append(point)
        
        return point_path
        
        
        # ros_path = Path()
        # ros_path.header.frame_id = 'map'
        # for point in path:
        #     pose_stamped = PoseStamped()
        #     pose_stamped.pose.position = Point(x=point.x, y=point.y, z=0)
        #     pose_stamped.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
        #     ros_path.poses.append(pose_stamped)


    @staticmethod
    def convert_to_list_path(ros_path):
        list_x = []
        list_y = []

        # ros_path_poses -> 2개 간격으로 변경
        # -> 중복 waypoint 필터링
        # -> 0.5m 간격의 local path waypoint -> 1m 간격으로 변경
        for rp_pose in ros_path.poses[::2]:
            list_x.append(rp_pose.pose.position.x)
            list_y.append(rp_pose.pose.position.y)

        return list_x, list_y

    @staticmethod
    def convert_to_ros_path(path, frame_id = 'map'):
        ros_path = Path()
        ros_path.header.frame_id = frame_id
        for point in path:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = Point(x=point.x, y=point.y, z=0)
            pose_stamped.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
            ros_path.poses.append(pose_stamped)

        return ros_path