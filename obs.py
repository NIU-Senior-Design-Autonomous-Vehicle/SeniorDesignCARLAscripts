#!/usr/bin/env python


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

try:
    #fix per '/spawn_npc.py'
    sys.path.append(glob.glob('../carla/dist/carla-0.9.8-py3.5-linux-x86_64.egg')[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla
import random
import time



def obs():
        client = carla.Client('localhost', 2000)
        world = client.get_world()
        spectator = world.get_spectator()

        vehicle_bp = random.choice(world.get_blueprint_library().filter('vehicle.lincoln.*'))
        sp = world.get_map().get_spawn_points()
        spawn = sp[3]
        i = 1
        for i in range(1,5):
            spawn.location.x += 20*i

            vehicle = world.try_spawn_actor(vehicle_bp, spawn)
        
        # Wait for world to get the vehicle actor
        world.tick()

        #world_snapshot = world.wait_for_tick()
        #actor_snapshot = world_snapshot.find(vehicle.id)

        # Set spectator at given transform (vehicle transform)
        #spectator.set_transform(actor_snapshot.get_transform())

if __name__ == '__main__':

    obs()