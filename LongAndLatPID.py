#!/usr/bin/env pytho

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


go = True #boolean for running program

######### Begin our setUp ############################################
    
actor_list = [] #list of actors for properly destroying

client = carla.Client('localhost', 2000) #contact server at this port
client.set_timeout(30.0) #large timeout time for slower machines
print('connection to client was a success!')
print('loading town2.......')
client.load_world('Town02') #load town2 or any other town
world = client.get_world() #get the world
spectator = world.get_spectator() #get spectator 
bp_lib = world.get_blueprint_library() #get the bp lib
print('loading tesla model 3')
model_3 = bp_lib.filter('model3')[0] #filter for the model 3
#get a random spawn point for the vehicle 
transform = random.choice(world.get_map().get_spawn_points())
#spawn the vehcile 
vehicle = world.try_spawn_actor(model_3, transform)
actor_list.append(vehicle) #add vehicle to list of actors
world.tick() #wait for car to be placed
world_snapshot = world.wait_for_tick() #''
actor_snapshot = world_snapshot.find(vehicle.id) #align world view and actor view
#put camera there
spectator.set_transform(actor_snapshot.get_transform())
print('can you smell the success?? cuz I can! :)')
time.sleep(3) #wait for everything to settle 

################# function for long PID control ###########################################

def longPID(vehicle):
    #now we will set our gain values
    Kp = 1.0
    Kd = 0.0
    Ki = 0.0
    #differential time
    dt = 0.03
    #this is an error array of some sort, should look more into this
    #just using it because other people did 
    error_buffer = deque(maxlen=30)
    
    #we need to generate a target speed
    #in Carla this is a 3D vector 
    #so we must get a carla 3D vector then do some math on its components

    target_speed = vehicle.get_velocity() #get current velocity of vehicle (3D vector)
    #this is zero because the car is not moving 
    #so add some values to it
    target_speed.x = 2.0
    target_speed.y = 2.0
    target_speed.z = 2.0
    ##print(target_speed)
    #now do some math to convert this into a float value
    target_speed = 3.6 * math.sqrt(target_speed.x ** 2 + target_speed.y ** 2 + target_speed.z ** 2)
    ##print('This is your target speed: {}'.format(target_speed))
    #now we will do something similar to the current speed of the vehicle 
    current_speed = vehicle.get_velocity()
    current_speed = 3.6 * math.sqrt(current_speed.x ** 2 + current_speed.y ** 2 + current_speed.z ** 2)
    ##print('This is your current speed: {}'.format(current_speed))

    #now we will begin our PID control
    error = (target_speed-current_speed) #the error
    error_buffer.append(error)#add it to the list

    if len(error_buffer) >= 2: #if we have more than two error values
        de = (error_buffer[-1] - error_buffer[-2]) / dt #math for differential error
        ie = sum(error_buffer) * dt #math for integral error
    else: #otherwise set these values to zero
        de = 0.0
        ie = 0.0
    #we do the above so we dont start off right away with crazy errors

    #now the math for outputting a throttle value between zero and one
    controlled_throttle = np.clip((Kp * error) + (Kd * de / dt) + (Ki * ie * dt), 0.0, 1.0)
    ##print('Your throttle value: {}'.format(controlled_throttle))
    #now we apply the throttle
    vehicle.apply_control(carla.VehicleControl(throttle=controlled_throttle, brake = 0.0))   
    #now ill add a sleep time just to see the number outputs better
    # this will hinder performance of the PID controller though 
    # can always change it later
    time.sleep(.3) 

def latPID(vehicle):
    #set gain values
    Kp = 1.0
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
    vehicle.apply_control(carla.VehicleControl(steer=controlled_steering))
    

while go == True:
    longPID(vehicle)
    latPID(vehicle)


