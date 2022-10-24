#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from network.ros_manager import RosManager
from planning.cams_planner import CAMSPlanner
# from autonomous_driving.planning.lattice_planner import LatticePlanner

def main(): 
    cams_planner = CAMSPlanner()
    ros_manager = RosManager(cams_planner)
    ros_manager.execute()


if __name__ == '__main__':
    main()