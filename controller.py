#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module contains PID controllers to perform lateral and longitudinal control. """

from collections import deque
import math

import glob
import os
import sys
import numpy as np
try:
    #fixed per '/spawn_npc.py'
    sys.path.append(glob.glob('../carla/dist/carla-0.9.8-py3.5-linux-x86_64.egg')[0])
except IndexError:
    pass
import carla
from misc import get_speed
import random
import time


class CarEnv:
    actor_list = [] #actor list like before for cleaning server
   
    #our init method with everything that we have seen from previous files.
    def __init__(self, Kp=1.0, Kd=0.0, Ki=0.0, dt=0.03 ):
        self.client = carla.Client('localhost', 2000) #port to communicate
        self.client.set_timeout(30.0) #increase timeout time for slower machines
        print('connection to client was a success!')
        print('loading town2.......')
        self.client.load_world('Town02')
        # Once we have a client we can retrieve the world that is currently
        # running.
        self.world =  self.client.get_world()
        self.spectator =  self.world.get_spectator()
        blueprint_library =  self.world.get_blueprint_library()
        print('loading telsa.....')
        self.model_3 = blueprint_library.filter('model3')[0]
        
        self.actor_list = [] #clear this array 
        self.transform = random.choice( self.world.get_map().get_spawn_points())

        self.vehicle =  self.world.try_spawn_actor(self.model_3, self.transform)
        
        self.actor_list.append(self.vehicle)
        self.world.tick()
        self.world_snapshot =  self.world.wait_for_tick()
        self.actor_snapshot =  self.world_snapshot.find(self.vehicle.id)
        self.spectator.set_transform(self.actor_snapshot.get_transform())

        print('success baby!!')
        time.sleep(3)
        #self.vehicle.get_velocity()
       # controlled = PIDLongitudinalController(self)
        #self.vehicle.apply_control(carla.VehicleControl(throttle = controlled, brake=0.0))

        
        
        #a_speed = math.sqrt((zero_speed.x + 10)**2 + (zero_speed.y + 10)**2 (zero_speed.z + 10)**2)
        
        
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.dt = dt
        self.error_buffer = deque(maxlen=30)
        go = True
        while go == True:
            target_speed = self.vehicle.get_velocity()
            target_speed.x = 2.0
            target_speed.y = 2.0
            target_speed.z = 2.0
            print(target_speed)
            target_speed = 3.6 * math.sqrt(target_speed.x ** 2 + target_speed.y ** 2 + target_speed.z ** 2)
            print(target_speed)
            current_speed = self.vehicle.get_velocity()
            current_speed = 3.6 * math.sqrt(current_speed.x ** 2 + current_speed.y ** 2 + current_speed.z ** 2)
            print(current_speed)
            error = (target_speed - current_speed)
            self.error_buffer.append(error)

            if len(self.error_buffer) >= 2:
                de = (self.error_buffer[-1] - self.error_buffer[-2]) / self.dt
                ie = sum(self.error_buffer) * self.dt
            else:
                de = 0.0
                ie = 0.0
            
            controlled_throttle = np.clip((self.Kp * error) + (self.Kd * de / self.dt) + (self.Ki * ie * self.dt), 0.0, 1.0)
            print(controlled_throttle)
            self.vehicle.apply_control(carla.VehicleControl(throttle = controlled_throttle, brake=0.0))
            time.sleep(1)



class VehiclePIDController():
    """
    VehiclePIDController is the combination of two PID controllers (lateral and longitudinal) to perform the
    low level control a vehicle from client side
    """

    def __init__(self, vehicle, args_lateral=None, args_longitudinal=None):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param args_lateral: dictionary of arguments to set the lateral PID controller using the following semantics:
                             K_P -- Proportional term
                             K_D -- Differential term
                             K_I -- Integral term
        :param args_longitudinal: dictionary of arguments to set the longitudinal PID controller using the following
        semantics:
                             K_P -- Proportional term
                             K_D -- Differential term
                             K_I -- Integral term
        """
        if not args_lateral:
            args_lateral = {'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0}
        if not args_longitudinal:
            args_longitudinal = {'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0}
        
        self._lon_controller = PIDLongitudinalController(self.vehicle, **args_longitudinal)
        self._lat_controller = PIDLateralController(self.vehicle, **args_lateral)

    def run_step(self, target_speed, waypoint):
        """
        Execute one step of control invoking both lateral and longitudinal PID controllers to reach a target waypoint
        at a given target_speed.

        :param target_speed: desired vehicle speed
        :param waypoint: target location encoded as a waypoint
        :return: distance (in meters) to the waypoint
        """
        throttle = self._lon_controller.run_step(target_speed)
        steering = self._lat_controller.run_step(waypoint)

        control = carla.VehicleControl()
        control.steer = steering
        control.throttle = throttle
        control.brake = 0.0
        control.hand_brake = False
        control.manual_gear_shift = False

        return control


class PIDLongitudinalController():
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, vehicle, K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        :param dt: time differential in seconds
        """
        target_speed = 50
        self.vehicle = vehicle
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self._dt = dt
        self._e_buffer = deque(maxlen=30)
        self.run_step(target_speed)

    def run_step(self, target_speed, debug=True):
        """
        Execute one step of longitudinal control to reach a given target speed.

        :param target_speed: target speed in Km/h
        :return: throttle control in the range [0, 1]
        """
        v = self.vehicle.get_velocity()
        current_speed = int(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))

        if debug:
            print('Current speed = {}'.format(current_speed))

        return self._pid_control(target_speed, current_speed)

    def _pid_control(self, target_speed, current_speed):
        """
        Estimate the throttle of the vehicle based on the PID equations

        :param target_speed:  target speed in Km/h
        :param current_speed: current speed of the vehicle in Km/h
        :return: throttle control in the range [0, 1]
        """
        
        _e = (target_speed - current_speed)
        self._e_buffer.append(_e)

        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0
        
        return np.clip((self._K_P * _e) + (self._K_D * _de / self._dt) + (self._K_I * _ie * self._dt), 0.0, 1.0)
        

class PIDLateralController():
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, vehicle, K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        :param dt: time differential in seconds
        """
        self.vehicle = vehicle
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self._dt = dt
        self._e_buffer = deque(maxlen=10)
        
    def run_step(self, waypoint):
        """
        Execute one step of lateral control to steer the vehicle towards a certain waypoin.

        :param waypoint: target waypoint
        :return: steering control in the range [-1, 1] where:
            -1 represent maximum steering to left
            +1 maximum steering to right
        """
        return self._pid_control(waypoint, self.vehicle.get_transform())

    def _pid_control(self, waypoint, vehicle_transform):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param waypoint: target waypoint
        :param vehicle_transform: current transform of the vehicle
        :return: steering control in the range [-1, 1]
        """
        v_begin = vehicle_transform.location
        v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
                                         y=math.sin(math.radians(vehicle_transform.rotation.yaw)))

        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array([waypoint.transform.location.x -
                          v_begin.x, waypoint.transform.location.y -
                          v_begin.y, 0.0])
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        self._e_buffer.append(_dot)
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._K_P * _dot) + (self._K_D * _de /
                                             self._dt) + (self._K_I * _ie * self._dt), -1.0, 1.0)
CarEnv()