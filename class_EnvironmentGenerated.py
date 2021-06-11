# -*- coding: utf-8 -*-
"""
Created on Tue Jan 21 09:33:58 2020

@author: local_ergo
"""
import numpy as np
import pandas as pd
import os
import time
from math import cos, sin, radians, atan2, degrees
from shapely.geometry import Polygon, Point
from functions_object_placement import door_placement, object_placement, light_placement, room_surface_initialization
from functions_collision_detection import rectangle_vertices, door_polygon
from functions_collision_detection import collision_detection, door_room_check, door_obj_collision_detection

#############################################################################
#                           CLASSES
#############################################################################
class Environment_Generated():
    ''' This class generates an environment with doors, lights and furniture randomly placed in two rooms, bedroom and bathroom'''
    def __init__(self, mainRoomCodes, bathRoomCodes, furnMu, furnSigma, lightMu, lightSigma):
        self.status = False # This status will come from the room evaluation. If it becomes true, it means at least one of the trajectories were infeasible
        self.unit_size_m = 0.20 # Grid size in cm (X,Y).
        self.numOfRows = 22 # Number of grids in a Y direction.
        self.numOfCols = 29 # Number of grids in a X direction.
        self.sample_zones = {}
        #*********************************************
        
        mainFurnitureCodeArray = mainRoomCodes[0]
        mainDoorsCodeArray = mainRoomCodes[1]
        mainLightCodeArray = mainRoomCodes[2]
        room_name = mainRoomCodes[3]
#        mainRoomVertices = [(0,0), (8,0),(8,10),(0,10)] # Perfect square room -> for testing
        mainRoomVertices = [(0,0), (4.27,0), (4.27, 5.79), (1.93, 5.79), (1.93, 4.57), (1.12, 3.78), (0,3.78)]
        mainRoom = self.Room(room_name, mainRoomVertices)
        
        bathFurnitureCodeArray = bathRoomCodes[0]
        bathDoorsCodeArray = bathRoomCodes[1]
        bathLightCodeArray = bathRoomCodes[2]
        room_name = bathRoomCodes[3]
#        bathRoomVertices = [(8,1), (13,1),(13,7),(8,7)] # Perfect square bethroom-> for testing
        bathRoomVertices = [(0, 3.78), (1.12,3.78), (1.93, 4.57), (1.93, 5.79), (0, 5.79)]
        bathRoom = self.Room(room_name, bathRoomVertices)        
        
        mainLightList = light_installation(mainLightCodeArray, mainRoom, lightMu, lightSigma)
        mainDoorList = door_collision(mainDoorsCodeArray, mainRoom, bathRoom)
        mainFurnitureList = multiple_object_collision_detection(mainFurnitureCodeArray, mainDoorList, mainRoom, furnMu, furnSigma)
#       
        bathLightList = light_installation(bathLightCodeArray, bathRoom, lightMu, lightSigma)
        bathDoorList = door_collision(bathDoorsCodeArray, bathRoom, mainRoom)
        bathFurnitureList = multiple_object_collision_detection(bathFurnitureCodeArray, bathDoorList, bathRoom, furnMu, furnSigma)
             
        self.doorListDict = {"main":mainDoorList, "bath":bathDoorList}
        self.doorList  = create_list_of_objects(mainDoorList, bathDoorList)
        
        self.furnitureListDict = {"main":mainFurnitureList, "bath":bathFurnitureList}
        self.furnitureList  = create_list_of_objects(mainFurnitureList, bathFurnitureList)
#        
        self.lightListDict = {"main":mainLightList, "bath":bathLightList}
        self.lightList  = create_list_of_objects(mainLightList, bathLightList)
        
        self.roomList = []
        self.roomList.append(mainRoom); self.roomList.append(bathRoom)
        
        self.sample_zones = find_sample_zone(self.doorList, self.furnitureList, self.roomList)
        
        self.walls = []
        self.floor = np.zeros([self.numOfRows, self.numOfCols]) # Initializing floor_type as a zero matrix with size of number_of_rows * number_of_columns.
        for room in self.roomList:
            self.walls.append(room.wallArray)
            for row in range(self.numOfRows):
                for col in range(self.numOfCols):
                    gridCoordinate = self.grid2meter(col,row)
                    gridPoint = Point(gridCoordinate)
                    if gridPoint.within(room.polygon): # Assigning the floor type based on the room section to each grid in the space
                        self.floor[row, col] = room.surfaceRisk
#        print(self.floor.shape)
    def grid2meter(self, col, row):
        x = row*self.unit_size_m
        y = col*self.unit_size_m
        return (x,y)
        

    class Room():
        ''' 
        This class generates rooms as polygons based on the list of vertices
        "vertices" passed to the class and name it "roomName 
        '''
        def __init__(self, roomName, vertices):
            library_file_name = os.path.join(os.getcwd(), 'Object_Library.csv') # The object library file address.
            object_library = pd.read_csv(library_file_name,) # Reading the object library file
            del library_file_name
            # Room Dimensions
            xAxis = []
            yAxis = []
            for item in vertices:
                xAxis.append(item[0])
                yAxis.append(item[1])
            xmin = min(xAxis)
            xmax = max(xAxis)
            ymin = min(yAxis)
            ymax = max(yAxis)
                
            self.x = np.linspace(xmin, xmax, 4*(int(xmax-xmin))+1) # m (the ability of choosing points at each 25cm)
            self.y = np.linspace(ymin, ymax, 4*(int(xmax-xmin))+1) # m (the ability of choosing points at each 25cm)
            self.polygon = Polygon(vertices)
#            print(vertices)
            self.name = roomName
            self.code, self.surfaceType, self.surfaceRisk = room_surface_initialization(object_library)
            self.numWalls, self.walls, self.wallArray = finding_walls(self)
#            plot_polygon(self.polygon)
#            plot_polygon(self.polygon, fignum = 2)
            
        
class FurniturePlacement():
    
    def __init__(self, objects_code, room, mu, sigma):
        library_file_name = os.path.join(os.getcwd(), 'Object_Library.csv') # The object library file address.
        object_library = pd.read_csv(library_file_name,) # Reading the object library file
        del library_file_name
        orientation = np.linspace(0.0, 179.0, 180)
        # The created object should be inside the room
        self.code = objects_code
        self.room = room.name
        inside = False
#        i = 0
        while inside == False:
#            if i>100:
#                print('FURN: mu is: ', mu, '\n and sigma is: \n', sigma)
#            print('furn trial: ',i)
#            i+=1
            self.conf, self.length, self.width, self.support, self.name = object_placement(objects_code, object_library, room.x, room.y, orientation, mu, sigma)
            _, _, _, _, self.polygon = rectangle_vertices(self.conf, self.length, self.width)
            _, _, _, _, obj = rectangle_vertices(self.conf, self.length, self.width)
#            plot_polygon(obj)
            inside = room.polygon.contains(obj)


class DoorPlacement():
    
    def __init__(self, door_code, room, otherRoom):
        library_file_name = os.path.join(os.getcwd(), 'Object_Library.csv') # The object library file address.
        object_library = pd.read_csv(library_file_name,) # Reading the object library file
        del library_file_name
        self.code = door_code
        outside = True
        while outside == True:  
            self.conf, self.length, self.width, self.support = door_placement(door_code, object_library, room, otherRoom)
            self.room = room.name
            _, _, _, _, self.polygon = door_polygon(self.conf, self.length, self.width, room)
            _, _, _, _, obj = door_polygon(self.conf, self.length, self.width, room)
#            plot_polygon(obj)
            outside = door_room_check(self, room)

class Lights():
    def __init__(self, light_code, room, lightMu, lightSigma):
        library_file_name = os.path.join(os.getcwd(), 'Object_Library.csv') # The object library file address.
        object_library = pd.read_csv(library_file_name,) # Reading the object library file
        del library_file_name
        self.code = light_code
        self.room = room.name
        inside = False
#        i = 0
        while inside == False:
#            if i>100:
#                print('LGHT: mu is: ', mu, '\n and sigma is: \n', sigma)
#            print('light trial ', i)           
            self.pos, self.intensity = light_placement(light_code, object_library, room, lightMu, lightSigma)
#            print(self.pos)
            self.point = Point(self.pos)
            light_source = self.point
#            plot_lights(light_source)
            inside = room.polygon.contains(light_source)

#############################################################################
#                           FUNCTIONS/METHODS
#############################################################################
def finding_walls(room):
    '''
    This function finds the number and coordinates of the input parameter "room" as walls
    it finds each side of the rooms polygon and recognizes them as walls
    '''
    num_walls = len(room.polygon.exterior.coords[:-1])  # counts the side of the rooms polygon -> number of walls
    available_walls = (np.linspace(0, num_walls-1, num_walls)).astype(int) # create a list of walls for sampling
#    print(available_walls)
    wall_list = []
    wall_arrayType = []
    for wall in available_walls:
        # find the position of the center of the door
        if wall == num_walls-1:
    #        print("last wall")
            wall_coords = [(room.polygon.exterior.coords[:-1][wall][0], room.polygon.exterior.coords[:-1][wall][1]),
                           (room.polygon.exterior.coords[:-1][0][0], room.polygon.exterior.coords[:-1][0][1])]
            wall_array = [room.polygon.exterior.coords[:-1][wall][0], room.polygon.exterior.coords[:-1][wall][1],
                           room.polygon.exterior.coords[:-1][0][0], room.polygon.exterior.coords[:-1][0][1]]
        else:
            wall_coords = [(room.polygon.exterior.coords[:-1][wall][0], room.polygon.exterior.coords[:-1][wall][1]),
                           (room.polygon.exterior.coords[:-1][wall+1][0], room.polygon.exterior.coords[:-1][wall+1][1])]
            wall_array = [room.polygon.exterior.coords[:-1][wall][0], room.polygon.exterior.coords[:-1][wall][1],
                          room.polygon.exterior.coords[:-1][wall+1][0], room.polygon.exterior.coords[:-1][wall+1][1]]
        wall_list.append(wall_coords)
        wall_arrayType.append(wall_array)
    return num_walls, wall_list, wall_arrayType

def door_collision(door_code_array, room, otherRoom):
    '''
    This function checks the collision between multiple objects that have been sent 
    to this function as a list of objects
    :param objects_code_array: an array of the objects code of interest (the codes
    can be retrived from the corresponding object library csv file)
    :param obj_list: list of objects that are placed in the room
    '''
    #creating a list of objects
    door_list = []
    for door_code in door_code_array:
        door_list.append(DoorPlacement([door_code], room, otherRoom))    # list of the objects in the room
        restart = True
        while restart == True:  # Making sure it checks the collision between all objects even after resampling
            restart = False
            for door in door_list[:-1]:
                collision_bool = door_obj_collision_detection(door, door_list[-1])
    #            print("checking collision between ", obj.code, "and, ", obj_list[-1].code)
                if collision_bool == True:  # If the new object is colliding with any of the existing objects, recreate the latest object and assign new configuration to it
    #                print("Collision detected between objects ", obj.code, " and ", obj_list[-1].code )
                    door_list[-1] = DoorPlacement([door_code], room, otherRoom)
                    restart = True
                
    return door_list

def multiple_object_collision_detection(objects_code_array, door_list, room, mus, sigmas):
    '''
    This function checks the collision between multiple objects that have been sent 
    to this function as a list of objects
    :param objects_code_array: an array of the objects code of interest (the codes
    can be retrived from the corresponding object library csv file)
    :param obj_list: list of objects that are placed in the room
    '''
#    print('Function Called')
    ''' creating a list of objects'''
    obj_list = []
    i = 0
    for obj_code in objects_code_array:
        if mus == []:
            mu = mus
            sigma = sigmas
        else:
            mu = mus[i]
            sigma = sigmas[i]
#        print('\n and sigma is: \n, ', sigma)
        i +=1
        obj_list.append(FurniturePlacement([obj_code], room, mu, sigma))    # list of the objects in the room
        
        restart = True
        timeOut = 5
        timeStart = time.time()
        while restart == True:  # Making sure it checks the collision between all objects even after resampling
            restart = False
            for obj in obj_list[:-1]:
                collision_obj = collision_detection(obj, obj_list[-1])    
#                print("checking collision between", obj.name, "and ", obj_list[-1].name)
                if collision_obj == True:  # If the new object is colliding with any of the existing objects, recreate the latest object and assign new configuration to it
#                    print("COLLISION detected between", obj.name, " and", obj_list[-1].name )
                    obj_list[-1] = FurniturePlacement([obj_code], room, mu, sigma)
                    restart = True
                if time.time() > timeStart + timeOut:
                    print('Time out! Unable to set the room. Recreating room', room.name)
                    return multiple_object_collision_detection(objects_code_array, door_list, room, mus, sigmas)
#            timeStart = time.time()     
            for door in door_list:
                collision_door = door_obj_collision_detection(door, obj_list[-1])
#                print("checking collision between door", door.code, "and", obj_list[-1].name)
                if collision_door == True:  # If the new object is colliding with any of the existing objects, recreate the latest object and assign new configuration to it
                    obj_list[-1] = FurniturePlacement([obj_code], room, mu, sigma)
#                    print("COLLISION detected between door", door.code, " and", obj_list[-1].name)
                    restart = True
                if time.time() > timeStart + timeOut:
                    print('Time out! Unable to place the doors. Recreating room', room.name)
                    return multiple_object_collision_detection(objects_code_array, door_list, room, mus, sigmas)                   
    return obj_list

def light_installation(light_code_array, room, lightMu, lightSigma):
    light_list = []
    
    i = 0
    for light_code in light_code_array:
        if lightMu != []:
            lightMu = lightMu[i]
            lightSigma = lightSigma[i]
        i +=1
        light_list.append(Lights([light_code], room, lightMu, lightSigma))
    restart = True
    while restart == True:
        restart = False
        for light in light_list[:-1]:
#            print('light pos: ', light.pos, ' and the type is', type(light.pos))
            if light.pos.all() == light_list[-1].pos.all():
                light_list[-1] = Lights([light_code], room, lightMu, lightSigma)
                restart = True
    return light_list

def create_list_of_objects(main, bath):
    '''
    This function takes in a list of furniture for main room and bathroom 
    and save them in a list (instead of a list of lists)
    '''
    list =[]
    for item in main:
        list.append(item)
    for item in bath:
        list.append(item)
    return list

def find_sample_zone(doorList, furnitureList, roomList):
    '''
    This function chooses the side of the object that the activity happens. For example the side on the toilet that can be sit on
    or the side of the bed that the patient can use to sit and get up from (side). The side variable is hard coded for different objects
    Then the algorithm uses a predefined length, l, to define apolygon on the activity side of the object for the fall model to sample from 
    that polygon to start the trajectory (furn.samplingZone). Then a sample zone for each object is defined based on its corner sitting.
    The sample zone is the area in front of the object that is being used by the trajectory to sample from for the start and end points of 
    the trajectories
    '''
    
    l = 0.4 # The length of the clearance, or where the activity intention sampling happens
    side = []
    sample_zones = {}
    
    for door in doorList:
        if door.room == 'main_room':
            sample_zones["Main Door"] = door.polygon
    
    for furn in furnitureList:
        if furn.name == 'Toilet':
            side = [3]
        if furn.name == 'Bed':
            side = [2,4]
        if furn.name == 'Chair-Visitor' or furn.name == 'Chair-Patient' or furn.name == 'Sofa' or furn.name == 'Couch' or furn.name == 'Sink-Bath':
            side = [2]
        
        x, y = furn.polygon.exterior.coords.xy # extracting the furniture orginal polygon
        theta = furn.conf[2]
        
        # creating the points for the sample zone polygon
        if 1 in side:
            p1 = [x[1], y[1]]
            p2 = [x[2], y[2]]
            p3 = (x[2]-l*cos(radians(theta)), y[2]-l*sin(radians(theta)))
            p4 = (x[1]-l*cos(radians(theta)), y[1]-l*sin(radians(theta)))
        if 2 in side:
            p1 = (x[2], y[2])
            p2 = (x[2]+l*sin(radians(theta)), y[2]-l*cos(radians(theta)))
            p3 = (x[3]+l*sin(radians(theta)), y[3]-l*cos(radians(theta)))
            p4 = (x[3], y[3])
        if 3 in side:
            p1 = [x[0], y[0]]
            p2 = [x[3], y[3]]
            p3 = (x[3]+l*cos(radians(theta)), y[3]+l*sin(radians(theta)))
            p4 = (x[0]+l*cos(radians(theta)), y[0]+l*sin(radians(theta)))
        if 4 in side:
            p1 = (x[1], y[1])
            p2 = (x[0], y[0])
            p3 = (x[0]-l*sin(radians(theta)), y[0]+l*cos(radians(theta)))
            p4 = (x[1]-l*sin(radians(theta)), y[1]+l*cos(radians(theta)))
        '''Check the intersection between the furniture samplinf zone and the corresponding room'''
        tempSamplingZone = Polygon([p1, p2, p3, p4])
        if furn.room == 'main_room':
          furn.samplingZone = tempSamplingZone.intersection(roomList[0].polygon)
        else:
          furn.samplingZone = tempSamplingZone.intersection(roomList[1].polygon)
        sample_zones[furn.name] = furn.samplingZone

    return sample_zones