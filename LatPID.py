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

def latPID(vehicle, world):
    #set gain values
    Kp = 0.2
    Kd = 0.0
    Ki = 0.0
    #differential time
    dt = 0.03
    #this is the error array.. 
    err_buffer = deque(maxlen=10)

    #get the vehicle's current location
    vehicle_transform = vehicle.get_transform()
    v_begin = vehicle_transform.location
    v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
                                         y=math.sin(math.radians(vehicle_transform.rotation.yaw)))

    v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])

    #get the location of the nearest waypoint that's in the center of the nearest driving lane
    map = world.get_map()
    waypoint = map.get_waypoint(vehicle.get_location(),project_to_road=True, lane_type=carla.LaneType.Driving)
    
    
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
  
    time.sleep(.3)

    return controlled_steering