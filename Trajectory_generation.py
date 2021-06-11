#!/usr/bin/env python

import math
import time
import numpy as np
import random
from Optimization import OptPath_patient, OptPath_patient2
from shapely.geometry import Polygon, Point
from math import cos, sin, tan, radians, atan2, degrees, pi
from matplotlib import pyplot as plt

from functions_geometry import polygon_scale, polygon_center
from functions_door_related import is_between
from functions_collision_detection import rectangle_vertices, door_polygon, plot_polygon#, polygon_with_clearance


def define_obstacles(env):
    ''' This function is to define line segments for each obstacle to be used in the trajectory optimization
    and also obstacle polygons to be used for sampling start and end points that are out of obstacles.'''

    obstacles = []
    r_patient = 0.3 # Margin to avoid obstacles

    for obj in env.furnitureList:
        scaledPoly = obj.polygon
        scaledPoly = polygon_scale(scaledPoly, 1) # we scale the polygon to a bigger one here so we give the object a clearance around the object
        corners = np.asarray(scaledPoly.exterior.coords) # vertices
#        corners = np.asarray(obj.polygon.exterior.coords) # vertices
#        print('vertices for', obj.name, 'are: ', corners)
        m_box = []
        b = []
        for i in range(4):
          denominator = (corners[i+1][0]-corners[i][0])
          if denominator == 0:
            denominator = 0.001 # THIS NUMBER DEPENDS ON THE BIG M NUMBER IN OPTIMIZATION. IT CANNOT BE SMALLER THAN A CERTAIN VALUE
#          m_box.append((corners[i+1][1]-corners[i][1])/(corners[i+1][0]-corners[i][0]))
          m_box.append((corners[i+1][1]-corners[i][1])/(denominator))
          if m_box[-1] > 900:
#            print('m_box[-1]:', m_box[-1])
            m_box[-1] = 900
          if m_box[-1] < -900:
            m_box[-1] = -900
          
#          b.append(-corners[i][0]*m_box[i]+corners[i][1])
#        print('m_box for', obj.name,' is: ', m_box)
        
        '''Swapping x and y to deal with openCV function'''
        b = [corners[0][1] - m_box[0] * corners[0][0], corners[1][1] - m_box[1] * corners[1][0],
                    corners[2][1] - m_box[2] * corners[2][0], corners[3][1] - m_box[3] * corners[3][0]] # line = mx + b
#        print('b is: ', b)
        _, center_pose = polygon_center(scaledPoly)
#        center_pose = [obj.conf[0], obj.conf[1]]
        obstacles.append([m_box, b, center_pose, Polygon(corners)])
        
#    plt.show()
    for room_wall in env.walls:
      for wall in room_wall:          
#        print('wall is:', wall)
        wall_c = [(wall[0]+wall[2])/2, (wall[1]+wall[3])/2] # center of the wall
        wall_d = [2*r_patient + np.sqrt((wall[0]-wall[2])**2+(wall[1]-wall[3])**2),2*r_patient + 0.3]
        wall_angle = np.arctan2((wall[1]-wall[3]), (wall[0]-wall[2])) + 0.001
        m_box = [math.tan(wall_angle), math.tan(wall_angle + math.pi / 2), math.tan(wall_angle), math.tan(wall_angle + math.pi / 2)]
        corners = [[wall_c[0] - (wall_d[1]/2) * math.sin(wall_angle) - (wall_d[0]/2) * math.cos(wall_angle), wall_c[1] + (wall_d[1]/2) * math.cos(wall_angle) - (wall_d[0]/2) * math.sin(wall_angle)],
                   [wall_c[0] + (wall_d[1]/2) * math.sin(wall_angle) - (wall_d[0]/2) * math.cos(wall_angle), wall_c[1] - (wall_d[1]/2) * math.cos(wall_angle) - (wall_d[0]/2) * math.sin(wall_angle)],
                   [wall_c[0] + (wall_d[1]/2) * math.sin(wall_angle) + (wall_d[0]/2) * math.cos(wall_angle), wall_c[1] - (wall_d[1]/2) * math.cos(wall_angle) + (wall_d[0]/2) * math.sin(wall_angle)],
                   [wall_c[0] - (wall_d[1]/2) * math.sin(wall_angle) + (wall_d[0]/2) * math.cos(wall_angle), wall_c[1] + (wall_d[1]/2) * math.cos(wall_angle) + (wall_d[0]/2) * math.sin(wall_angle)]]
        b = [corners[0][1] - m_box[0] * corners[0][0], corners[1][1] - m_box[1] * corners[1][0],
                    corners[2][1] - m_box[2] * corners[2][0], corners[3][1] - m_box[3] * corners[3][0]]
        center_pose = [wall_c[0], wall_c[1]]

        ''' Check if there is a door on the wall'''
        for door in env.doorList:
          if door.room == 'bath_room':
            if not(is_between([wall[0], wall[1]], [wall[2], wall[3]], [door.conf[0], door.conf[1]])): # if  the bathroom door is NOT on the current wall
              obstacles.append([m_box, b, center_pose, Polygon(corners)])
#              plot_polygon(Polygon(corners))
            else: # if the bathroom door is on this wall
#              print('The bathroom door is on this wall')
              origWallPoly = Polygon(corners) # the polygon for the original wall
              
              # ----- DOOR VARIALBLES -----
              door_c = [door.conf[0], door.conf[1]]
              door_angle = door.conf[2] # orientation of the door, theta
              door_d = [door.length , 2*r_patient + 1.2] # length of the door, width of the wall
              doorCorners = [[door_c[0] - (door_d[1]/2) * sin(radians(door_angle)) - (door_d[0]/2) * cos(radians(door_angle)), door_c[1] + (door_d[1]/2) * cos(radians(door_angle)) - (door_d[0]/2) * sin(radians(door_angle))],
                         [door_c[0] + (door_d[1]/2) * sin(radians(door_angle)) - (door_d[0]/2) * cos(radians(door_angle)), door_c[1] - (door_d[1]/2) * cos(radians(door_angle)) - (door_d[0]/2) * sin(radians(door_angle))],
                         [door_c[0] + (door_d[1]/2) * sin(radians(door_angle)) + (door_d[0]/2) * cos(radians(door_angle)), door_c[1] - (door_d[1]/2) * cos(radians(door_angle)) + (door_d[0]/2) * sin(radians(door_angle))],
                         [door_c[0] - (door_d[1]/2) * sin(radians(door_angle)) + (door_d[0]/2) * cos(radians(door_angle)), door_c[1] + (door_d[1]/2) * cos(radians(door_angle)) + (door_d[0]/2) * sin(radians(door_angle))]]
              doorPoly = Polygon(doorCorners)
              
              # ------- SUBTRACTING DOOR POLYGON FROM THE WALL POLYGON
              diffPoly = origWallPoly.difference(doorPoly) # subtracting the door polygon from the wall polygon
              m_box = [tan(wall_angle), tan(wall_angle + pi / 2), tan(wall_angle), tan(wall_angle + pi / 2)]
              
              # ---- BREAKING DOWN THE WALL TO TWO SECTIONS, TWO OBSTACLES WITH THE DOORWAY BEING OPEN FOR TRAJECTORY SAMPLING
              for polygon in diffPoly:
                _, center_pose = polygon_center(polygon)
                corners = np.asarray(polygon.exterior.coords)
                b = [corners[0][1] - m_box[0] * corners[0][0], corners[1][1] - m_box[1] * corners[1][0],
                     corners[2][1] - m_box[2] * corners[2][0], corners[3][1] - m_box[3] * corners[3][0]]
                obstacles.append([m_box, b, center_pose, polygon])
#                plot_polygon(polygon)
#    plt.show()
#    print(obstacles)
    return obstacles

def sample_point(env, obj, obstacles):
    ''' This function samples a point around the target object. It can be a sitting zone for sittable furniture,
    a reaching zone for reachable objects such as bathroom sink, or just inside an area like main entrance door. '''
    # Recognizing the sampling zone range for sampling start and end points
#    x_min, y_min, x_max, y_max = env.sample_zones[obj].exterior.coords.xy
    x_min, y_min, x_max, y_max = env.sample_zones[obj].bounds
#    print(obj,'x_min, y_min, x_max, y_max:', x_min, y_min, x_max, y_max)
    found = False
    timeOut = 0.01
    timeStart = time.time()
    while not found:
        x = random.uniform(x_min,x_max)
        y = random.uniform(y_min,y_max)
        point = Point([x,y])
#        print('point:', point, 'object:',obj)
        # Check if the sampled point is in the sitting zone of the target object
        is_in_sample_zone = False
        if point.within(env.sample_zones[obj]):
#          print('obj:', obj,'in sample zone')
          is_in_sample_zone = True

        # Check if the sampled point is out of all the obstacles in the room
        is_out_of_obstacle = True
        for obs in obstacles:
            if point.within(obs[3]):
                is_out_of_obstacle = False
#                print('obj:', obj,'in obstacle', obs)

        if is_in_sample_zone == True and is_out_of_obstacle == True:
            found = True
            point = [x,y, 0, 0, 0]
        if time.time() > timeStart + timeOut:
#          print('time out for finding start or end point fot trajectory...')
          return []

    return point

def is_near_sitting_object(state, env, obj):
    ''' This function determines if a point is near the sitting zone of a target object. '''

    is_near = False
    if obj in ['Bed', 'Chair', 'Toilet', 'Sofa', 'Couch']:
        if state.within(env.sample_zones[obj]):
            is_near = True
    return is_near

def generate_trajectory(start, end, env, obstacles, v_max, w_max, num_points):
    ''' This is the main function that generates trajectories given a scenario. It samples the start and end point.
    Then, using optimization, we find an optimal path between these 2 points. Finally, for each point in the trajectory,
    we find a corresponding activity based on the distance to the target object. '''

    found = 0
    cc = 0 # keeps track of the number of time the Gurobi optimization tried to find a trajectory
    while found == 0:
        cc += 1
        # Sample points near the start and end locations
        patient_s = sample_point(env, start, obstacles)
        patient_g = sample_point(env, end, obstacles)
#        print('patient start:', patient_s, 'patient end:', patient_g)
        
#        print("senario: ", scenario)
        if patient_s == [] or patient_g == [] or cc > 50:
#          print('No start or end point could be sampled in 0.1 seconds for,',start,'for', cc,'th time')
          status = 3
          # Terminate the optimization if it coulnot find a trajectory after the limit sat for cc
          if  cc >= 50 :
            found = 1
            traj = []
            print('No start or ending point or trajectory were found in ', cc, ' iterations')
            break
          continue
        scenario = {'start': patient_s, 'end': patient_g, 'v_max': v_max, 'w_max': w_max}
        # Find a trajectory between sampled points
        cost, predicted_patient_traj, status = OptPath_patient2(scenario['start'], scenario['end'],  [scenario['v_max'], scenario['w_max']] , obstacles, num_points, assistive_device=False)
        
        # If the optimization was successful, find the type of activity for each point on the trajectory and add it to the resturning path  
        if status == 2 : # optimization status. 2 means it was possible. 
            found = 1
            traj = []
            print("Finding activity types...")
            for i in range(len(predicted_patient_traj)):
                if is_near_sitting_object(Point(predicted_patient_traj[i]), env, start) :
                    traj.append([predicted_patient_traj[i], 'sit_to_stand'])
                elif is_near_sitting_object(Point(predicted_patient_traj[i]), env, end):
                    traj.append([predicted_patient_traj[i], 'stand_to_sit'])
                else:
                    traj.append([predicted_patient_traj[i], 'walking'])



    return traj, status
