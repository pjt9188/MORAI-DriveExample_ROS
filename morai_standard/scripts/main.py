#!/usr/bin/env python
# -*- coding: utf-8 -*-
from network.ros_manager import RosManager
from autonomous_driving.autonomous_driving import AutonomousDriving


def main():
    # K-city 
    # autonomous_driving = AutonomousDriving()
    
    # Scenarios
    autonomous_driving = AutonomousDriving(config_file='map/V_RHT_HighwayJunction_1/CCRB4/config.json')
    
    ros_manager = RosManager(autonomous_driving)
    ros_manager.execute()


if __name__ == '__main__':
    main()