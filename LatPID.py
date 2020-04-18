#!/usr/bin/env python

#this will contain our Longitudinal controller

from collections import deque
import math

import glob
import os
import sys
import numpy as np
try:
    #hardcoded this file path because it wasnt working for me before
    sys.path.append(glob.glob('../carla/dist/carla-0.9.8-py3.5-linux-x86_64.egg')[0])
except IndexError:
    print('fix your file path around line 14')
import carla
from agents.tools.misc import get_speed
import random
import time

# SPECIAL note: this is the same lateral PID as LongAndLatPID.py 
# this method is just used to return a steering value that will control the car

# this PID needs to optimized too!



################# function for lat PID control ###########################################
err_buffer = deque(maxlen=10)
def latPID(vehicle, world, laneChange, count, x_desiredPath, y_desiredPath, x_actualPath, y_actualPath):
    #set gain values
    Kp = .3 #.2
    Kd = .001  #1
    Ki = .05 #.03
    #differential time
    dt = 0.03
   

    #get the vehicle's current location
    vehicle_transform = vehicle.get_transform()
    v_begin = vehicle_transform.location
    v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
                                         y=math.sin(math.radians(vehicle_transform.rotation.yaw)))

    v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])

    ## Here we get the map and 'wypt' is set here only to pass into the 'get_left_lane' and 'get_right_lane'
    ## if a lane change is set to occur. The 'wypt' just gets the nearest waypoint in the center of the current
    ## driving lane. That way the 'get_left_lane' and 'get_right_lane' get the nearest waypoint in the 
    ## adjacent lane. 
    map = world.get_map()
    wypt = map.get_waypoint(vehicle.get_location(),project_to_road=True, lane_type=carla.LaneType.Driving)
    
    if laneChange == False:
        #if no lane change is set to occur, then get the location of the nearest waypoint that's in the center 
        #of the nearest driving lane
        waypoint = map.get_waypoint(vehicle.get_location(),project_to_road=True, lane_type=carla.LaneType.Driving)

        #need to collect the coordinates for the desired and actual paths for plotting
        x_desiredPath.append(waypoint.transform.location.x)
        y_desiredPath.append(waypoint.transform.location.y)
        x_actualPath.append(v_begin.x)
        y_actualPath.append(v_begin.y)


    elif laneChange == True:
        #if want to change to the left lane, get the location of the nearest waypoint that's in the center of the
        #adjacent driving lane to the left
        waypoint = carla.Waypoint.get_left_lane(wypt)

        #need to collect the coordinates for the desired and actual paths for plotting
        x_desiredPath.append(waypoint.transform.location.x)
        y_desiredPath.append(waypoint.transform.location.y)
        x_actualPath.append(v_begin.x)
        y_actualPath.append(v_begin.y)

        count  = count + 1
        if count == 40:
            laneChange = False

    w_vec = np.array([waypoint.transform.location.x -
                          v_begin.x, waypoint.transform.location.y -
                          v_begin.y, 0.0])
    
    dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

    cross = np.cross(v_vec, w_vec)
    if cross[2] < 0:
        dot *= -1.0

    err_buffer.append(dot)
    if len(err_buffer) >= 2:
        diff_error = (err_buffer[-1] - err_buffer[-2]) / dt
        integral_error = sum(err_buffer) * dt
    else:
        diff_error = 0.0
        integral_error = 0.0

    controlled_steering = np.clip((Kp * dot) + (Kd * diff_error / dt) + (Ki * integral_error * dt), -1.0, 1.0)
  
    #time.sleep(.3)

    return controlled_steering, laneChange, count, x_desiredPath, y_desiredPath, x_actualPath, y_actualPath