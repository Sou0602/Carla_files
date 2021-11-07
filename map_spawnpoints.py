from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla
import sys


from carla import ColorConverter as cc
#from global_route_planner import GlobalRoutePlanner
#from global_route_planner_dao import GlobalRoutePlannerDAO
import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import time

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

from global_route_planner import GlobalRoutePlanner
from global_route_planner_dao import GlobalRoutePlannerDAO

actor_list = []
try:

    client = carla.Client('127.0.0.1',2000)
    client.set_timeout(5.0)

    #destroy existing actors.
    #world = client.load_world('Town06')
    world = client.get_world()
    actors = world.get_actors()
    for a in actors:
        #print(a.type_id)
        if 'vehicle' in a.type_id:
            a.destroy()

    blueprint_lib = world.get_blueprint_library()
    amap = world.get_map()

    sampling_resolution = 0.05
    dao = GlobalRoutePlannerDAO(amap,sampling_resolution)
    grp = GlobalRoutePlanner(dao)
    grp.setup()

    spawn_points = amap.get_spawn_points()
    a = carla.Location(spawn_points[430].location)
    b = carla.Location(spawn_points[416].location)
    '''''
    for k in range(len(spawn_points)):
        world.debug.draw_string(spawn_points[k].location,str(k),draw_shadow=False,color = carla.Color(r=255,g=0,b=0),life_time = 600.0,persistent_lines = True)
    '''''
    world.debug.draw_string(spawn_points[430].location, str(430), draw_shadow=False, color=carla.Color(r=255, g=0, b=0),
                            life_time=600.0, persistent_lines=True)
    world.debug.draw_string(spawn_points[416].location, str(416), draw_shadow=False, color=carla.Color(r=255, g=0, b=0),
                            life_time=600.0, persistent_lines=True)
#######################################################################################################################
#Define and Spawn Actors
    veh = blueprint_lib.filter('model3')[0]
    veh.set_attribute('role_name','autopilot')
    vehicle = world.spawn_actor(veh,spawn_points[430])
    vehicle.set_autopilot = True
    #vehicle.enable_constant_velocity(carla.Vector3D(10,0,0))
    actor_list.append(vehicle)

    vehicle_1 = world.spawn_actor(veh,spawn_points[357])
    vehicle_1.set_autopilot = True
    #vehicle_1.enable_constant_velocity(carla.Vector3D(10,0,0))
    actor_list.append(vehicle_1)

########################################################################################################################
    #Waypoints
    def single_lane(waypoint_list, lane_id):
        waypoints = []
        for i in range(len(waypoint_list) - 1):
            if waypoint_list[i].lane_id == lane_id:
                waypoints.append(waypoint_list[i])
        return waypoints


    #Generate way_points at a distance of 2
    #Plot Waypoints at a distance of 2
    '''''
    way_list = amap.generate_waypoints(2)
    way_list = single_lane(way_list,-3)
    for w in way_list:
        world.debug.draw_string(w.transform.location, 'O', draw_shadow=False,
                                color=carla.Color(r=0, g=255, b=0), life_time=600.0,
                                persistent_lines=True)
    '''''
    w1 = grp.trace_route(a, b)
    w1 = w1[:3600]
    vwaypoint = amap.get_waypoint(vehicle.get_location())
    '''''
    waypoint = amap.get_waypoint(vehicle.get_location())
    next_waypoint = list(waypoint.next(1.0))
    world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                    color=carla.Color(r=0, g=0, b=255), life_time=600.0,
                                    persistent_lines=True)
    world.debug.draw_string(next_waypoint[0].transform.location, 'O', draw_shadow=False,
                            color=carla.Color(r=0, g=0, b=255), life_time=600.0,
                            persistent_lines=True)
    '''''


    # Observing collisions with this velocity.
    # Write a vehicle control on lane id of ramp and generate more waypoints along that for navigation
    ## Figure how to move the carla vehicle in any directoin of choice, either by defining lane id or using a planner.1
    #vehicle.apply_control(carla.VehicleControl(throttle = 1.0,steer = 0.0))
    '''''
    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)
    settings = world.get_settings()

    tm_port = tm.get_port()
    tm.global_percentage_speed_difference(30.0)

    for v in actor_list:
        v.set_autopilot = True
        v.set_simulate_physics(True)
        tm.auto_lane_change(v,False)
    '''''
    #while True:
    #    world.tick()
    #    print(vehicle.get_control())
    def control_pure_pursuit(vehicle_tr, waypoint_tr, max_steer, wheelbase):
        # TODO: convert vehicle transform to rear axle transform
        wp_loc_rel = relative_location(vehicle_tr, waypoint_tr.location) + carla.Vector3D(wheelbase, 0, 0)
        wp_ar = [wp_loc_rel.x, wp_loc_rel.y]
        d2 = wp_ar[0] ** 2 + wp_ar[1] ** 2
        steer_rad = math.atan(2 * wheelbase * wp_loc_rel.y / d2)
        steer_deg = math.degrees(steer_rad)
        steer_deg = np.clip(steer_deg, -max_steer/2, max_steer/2)
        return steer_deg / max_steer

    def relative_location(frame, location):
        origin = frame.location
        forward = frame.get_forward_vector()
        right = frame.get_right_vector()
        up = frame.get_up_vector()
        disp = location - origin
        x = np.dot([disp.x, disp.y, disp.z], [forward.x, forward.y, forward.z])
        y = np.dot([disp.x, disp.y, disp.z], [right.x, right.y, right.z])
        z = np.dot([disp.x, disp.y, disp.z], [up.x, up.y, up.z])
        return carla.Vector3D(x, y, z)

    physics_control = vehicle.get_physics_control()
    max_steer = physics_control.wheels[0].max_steer_angle
    rear_axle_center = (physics_control.wheels[2].position + physics_control.wheels[3].position)/200
    offset = rear_axle_center - vehicle.get_location()
    wheelbase = np.linalg.norm([offset.x, offset.y, offset.z])
    vehicle.set_simulate_physics(True)
    goal = spawn_points[312]
    goal2 = spawn_points[416]
    gl = np.array([goal.location.x, goal.location.y])
    gl2 = np.array([goal2.location.x, goal2.location.y])

    j = 0
    while True:
        next_waypoint = list(vwaypoint.next_until_lane_end(1.0))
       # next_waypoint = w1[j]
        wp = next_waypoint[0]
        world.debug.draw_string(wp.transform.location, 'X', draw_shadow=False,
                                color=carla.Color(r=0, g=0, b=255), life_time=600.0,persistent_lines=True)

        #Control vehicle's throttle and steering


        vehicle_transform = vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vxy = np.array([vehicle_location.x , vehicle_location.y])
        nwxy = np.array([wp.transform.location.x,wp.transform.location.y])
        dif = vxy - nwxy
        dif_norm = np.sqrt(np.sum(np.square(dif)))
        print(dif_norm)

      #  if dif_norm < 0.03:
      #      j = j+1

        throttle = 0.1
        steer = control_pure_pursuit(vehicle_transform, wp.transform, max_steer, wheelbase)
        control = carla.VehicleControl(throttle, steer)
      #  vehicle.apply_control(control)
        vwaypoint = amap.get_waypoint(vehicle.get_location())
        #cdone = world.tick()
        world.debug.draw_string(vwaypoint.transform.location, 'X', draw_shadow=False,
                                color=carla.Color(r=255, g=0, b=0), life_time=600.0,
                                persistent_lines=True)

        wtl = np.array([vwaypoint.transform.location.x, vwaypoint.transform.location.y])
        diff = gl - wtl
        diff2 = gl2 - wtl
        dist = np.sqrt(np.sum(np.square(diff)))
        dist2 = np.sqrt(np.sum(np.square(diff2)))
        print(dist , dist2)

        #time.sleep(0.1)
        if j == len(w1):
            break


    time.sleep(60)
finally:
    for actor in actor_list:
        actor.destroy()
    print("All cleaned up")

# 300,36,40,48,52,56,60,68,81 and ego at 381
# Ego at 430 - towards 287  , 312 , Moving Obstacles at 357 ,328 - merge at 416
# Ego at 362,257 , Moving Obstacles at 8 , 275 , 12 ,279 - merge at 103