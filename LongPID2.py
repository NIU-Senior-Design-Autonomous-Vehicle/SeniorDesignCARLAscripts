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
from misc import get_speed
import random
import time


go = True #boolean for running program

error_buffer = deque(maxlen=30)
    




################# function for long PID control ###########################################

def longPID(vehicle):
    
    #now we will set our gain values
    Kp = .3
    Kd = .001
    Ki = .05
    #differential time
    dt = 0.03
    #this is an error array of some sort, should look more into this
    #just using it because other people did 
    
    #we need to generate a target speed
    #in Carla this is a 3D vector 
    #so we must get a carla 3D vector then do some math on its components

    target_speed = vehicle.get_velocity() #get current velocity of vehicle (3D vector)
    #this is zero because the car is not moving 
    #so add some values to it
    target_speed.x = 2.0
    target_speed.y = 2.0
    target_speed.z = 2.0
    print(target_speed)
    #now do some math to convert this into a float value
    target_speed = 3.6 * math.sqrt(target_speed.x ** 2 + target_speed.y ** 2 + target_speed.z ** 2)
    print('This is your target speed: {}'.format(target_speed))
    #now we will do something similar to the current speed of the vehicle 
    current_speed = vehicle.get_velocity()
    current_speed = 3.6 * math.sqrt(current_speed.x ** 2 + current_speed.y ** 2 + current_speed.z ** 2)
    print('This is your current speed: {}'.format(current_speed))

    #now we will begin our PID control
    error = (target_speed-current_speed) #the error
    error_buffer.append(error)#add it to the list
    print(error_buffer)

    if len(error_buffer) >= 2: #if we have more than two error values
        de = (error_buffer[-1] - error_buffer[-2]) /dt #math for differential error
        ie = sum(error_buffer)*dt #math for integral error
    else: #otherwise set these values to zero
        de = 0.0
        ie = 0.0
    #we do the above so we dont start off right away with crazy errors
    print(de)
    print(ie)
    #now the math for outputting a throttle value between zero and one
    controlled_throttle = np.clip((Kp * error) + (Kd * de/dt) + (Ki * ie*dt ), 0.3, 0.5)
    print('Your throttle value: {}'.format(controlled_throttle))
    return controlled_throttle
    #now we apply the throttle
    #vehicle.apply_control(carla.VehicleControl(throttle=controlled_throttle, brake = 0.0))   
    #now ill add a sleep time just to see the number outputs better
    # this will hinder performance of the PID controller though 
    # can always change it later
   # time.sleep(.3) 




  #  while go == True:
        #longPID(vehicle)


