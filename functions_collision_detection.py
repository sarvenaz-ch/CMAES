"""

This code look into the collision between two polynomials

"""
import os
import shapely
import numpy as np
import pandas as pd
from math import cos, sin, radians, atan2, degrees
from math import sqrt
from matplotlib import pyplot as plt
from shapely.geometry import box, LinearRing, Polygon
from shapely.ops import cascaded_union

def door_polygon(configuration, length, width):
    '''
    This function is built on the rectangle_vertices() function
    This function takes the center of the rectangles side that lies on the wall
    and orientation of the wall and calculates the coordinates of the rectangle
    vertices. These points can be used to define the rectangle as a polygon
    and use predefined functions that work for polygons
    :param configuration: a list in form of [x, y, theta] that contains the 
    coordinates of the center of the retangle ([x,y]) and orientation of the 
    rectangle measured from the x-axis (theta)
    :param length:the length of the rectangle
    :param width: the width of the rectangle
    '''
    
    x = configuration[0]    # The center of the rectangle
    y = configuration[1]    # The center of the rectangle
    theta = configuration[2]    # The orientation of the rectangle
   
#    print("theta: ", theta)
#    print("phi: ", phi)
#    if room.name == 'main_room':
    phi = round(degrees(atan2(2*width, length))-theta, 2)
    m = (cos(radians(phi)))*sqrt(pow(width, 2) + pow((length/2), 2))
    n = (sin(radians(phi)))*sqrt(pow(width, 2) + pow((length/2), 2))
    
    x1 = round(x-m, 2)
    y1 = round(y+n, 2)
    p1 = (x1, y1)
    
    phi = round(degrees(atan2(2*width, length))+theta, 2)
    m = (cos(radians(phi)))*sqrt(pow(width, 2) + pow((length/2), 2))
    n = (sin(radians(phi)))*sqrt(pow(width, 2) + pow((length/2), 2))
    
    x2 = round(x+m, 2)
    y2 = round(y+n, 2)
    p2 = (x2, y2)
    
    m = (length/2)*(cos(radians(theta)))
    n = (length/2)*(sin(radians(theta)))
    
    x3 = round(x+m, 2)
    y3 = round(y+n, 2)
    p3 = (x3, y3)
    
    x4 = round(x-m, 2)
    y4 = round(y-n, 2)
    p4 = (x4, y4)
    
    polygon = Polygon([p1, p2, p3, p4]) # create a polygon out of the calculated points
    
    return p1, p2, p3, p4, polygon


def polygon_with_clearance(obj, obj_csv):
  polygons = []   
#  print('clearance for :', obj.name)
  
  ####### FOR TEST ######
#  obj.polygon = Polygon([(0,0), (2,0),(2,4),(0,4)])
#  theta = 45
  #######################
  x, y = obj.polygon.exterior.coords.xy # extracting the furniture orginal polygon
  theta = obj.conf[2]
  polygons.append(obj.polygon)
  # reads the clearance sides from the excel file and save them in a list
  sides = [obj_csv.iloc[0]['Sides']] 
  for i in range(0, len(sides)): sides[i] = str(sides[i])
  if sides[0].find(',')!=-1:  # if more than one side needs clearance
    sides = sides[0].split(",")
  for i in range(0, len(sides)): sides[i] = int(float(sides[i]))
#  print('sides:', sides)

  l = obj_csv.iloc[0]['Clearance Value'] # reads the clearance value from the excel file
    
  # creating the points for the sample zone polygon
  if 1 in sides:
      p1 = [x[1], y[1]]
      p2 = [x[2], y[2]]
      p3 = (x[2]-l*cos(radians(theta)), y[2]-l*sin(radians(theta)))
      p4 = (x[1]-l*cos(radians(theta)), y[1]-l*sin(radians(theta)))
      pp = Polygon([p1, p2, p3, p4])
#      print('side 1')
      polygons.append(pp)
  if 2 in sides:
      p1 = (x[2], y[2])
      p2 = (x[2]+l*sin(radians(theta)), y[2]-l*cos(radians(theta)))
      p3 = (x[3]+l*sin(radians(theta)), y[3]-l*cos(radians(theta)))
      p4 = (x[3], y[3])
      pp = Polygon([p1, p2, p3, p4])
#      print('side 2')
      polygons.append(pp)
  if 3 in sides:
      p1 = [x[0], y[0]]
      p2 = [x[3], y[3]]
      p3 = (x[3]+l*cos(radians(theta)), y[3]+l*sin(radians(theta)))
      p4 = (x[0]+l*cos(radians(theta)), y[0]+l*sin(radians(theta)))
      pp = Polygon([p1, p2, p3, p4])
#      print('side 3')
      polygons.append(pp)
  if 4 in sides:
      p1 = (x[1], y[1])
      p2 = (x[0], y[0])
      p3 = (x[0]-l*sin(radians(theta)), y[0]+l*cos(radians(theta)))
      p4 = (x[1]-l*sin(radians(theta)), y[1]+l*cos(radians(theta)))
      pp = Polygon([p1, p2, p3, p4])
#      print('side 4')
      polygons.append(pp)

###########  PLOT #############
#  for i in range(len(polygons)):
#    plot_polygon(polygons[i])

    
  finalPolygon = cascaded_union(polygons)# A polygon made up of union of the polygons for the objects and sides
#  plot_polygon(finalPolygon)
#  plt.show()
  return finalPolygon
    
def rectangle_vertices(configuration, length, width):
    '''
    This function takes in the center and orientation of a rectangle 
    (configuration) and calculates the coordinates of two oposite vertices of 
    the rectangle. These points can be used to define the rectangle as a polygon
    and use predefined functions that work for polygons
    :param configuration: a list in form of [x, y, theta] that contains the 
    coordinates of the center of the retangle ([x,y]) and orientation of the 
    rectangle measured from the x-axis (theta)
    :param length:the length of the rectangle
    :param width: the width of the rectangle
    '''
    x = configuration[0]    # The center of the rectangle
    y = configuration[1]    # The center of the rectangle
    theta = configuration[2]    # The orientation of the rectangle
    phi = round(degrees(atan2(width, length)))
   
#    print("theta: ", theta)
#    print("phi: ", phi)

    m = (cos(radians(phi+theta))/2)*sqrt(pow(width, 2) + pow(length, 2))
    n = (sin(radians(phi+theta))/2)*sqrt(pow(width, 2) + pow(length, 2))
    p = (cos(radians(theta-phi))/2)*sqrt(pow(width, 2) + pow(length, 2))
    q = (sin(radians(theta-phi))/2)*sqrt(pow(width, 2) + pow(length, 2))
    
#    print("m:", m, " n:", n, " p:", p, " q:", q)
    
    x1 = x+m
    y1 = y+n
    p1 = (x1, y1)

    x2 = x-p
    y2 = y-q
    p2 = (x2, y2)
    
    x3 = x-m
    y3 = y-n
    p3 = (x3, y3)
    
    x4 = x+p
    y4 = y+q
    p4 = (x4, y4)

    
#    print("p1: ", p1, "p2: ", p2, "p3: ", p3, "p4: ", p4)
    
#    r = sqrt(pow(length,2)+pow(width,2))    # The radius
    polygon = Polygon([p1, p2, p3, p4]) # create a polygon out of the calculated points
    
    return p1, p2, p3, p4, polygon


def plot_lights(light_source, fignum = 1, title = 'Object Placement'):
    '''
    :param light_source: light source tuple in the format of shapely,Point
    '''
    patch = light_source.buffer(0.5)
    x,y = patch.exterior.xy
    fig = plt.figure(fignum, figsize=(5,5), dpi=90)
    ax = fig.add_subplot(111)
    plt.gca().set_aspect('equal', adjustable='box')
    ax.plot(x, y)
    ax.set_title(title)

def plot_polygon(polygon, fignum = 1, title = 'Object Placement'):
    '''
    This function plots the polygons
    :param polygon: polygon object
    '''
    # Plot the found polygon
    x,y = polygon.exterior.xy
#    print(x, y)
    fig = plt.figure(fignum, figsize=(5,5), dpi=90)
    ax = fig.add_subplot(111)
    plt.gca().set_aspect('equal', adjustable='box')
    ax.plot(x, y)
    ax.set_title(title)

def door_room_check(door, room):
    '''
    This function checks if the defined door area places inside the room's perimeter
    if returns true, the door is outside the room
    '''
    if room.polygon.contains(door.polygon):
        return False
    else:
        return True

def collision_detection(obj1, obj2):
    '''
    This function uses other functions to check the collision between obj1 and
    obj2 
    :param obj1: in the form of [[x,y,theta], length, width] where (x,y) are 
    the coordinates of the center of the object, theta is the orientation of 
    the object with respect to x-axis, length and width are length and width 
    of the object. same goes for the second object, obj2
    '''
    
    library_file_name = os.path.join(os.getcwd(), 'Object_Library.csv') # The object library file address.
    object_library = pd.read_csv(library_file_name,) # Reading the object library file
    del library_file_name
    
    obj_df1 = object_library.loc[object_library['Object Code'] == obj1.code[0]]
    objClearance_1 = obj_df1.iloc[0]['Clearance']
    
    obj_df2 = object_library.loc[object_library['Object Code'] == obj2.code[0]]
    objClearance_2 = obj_df2.iloc[0]['Clearance']
    
#    print('Obj1:', objClearance_1, 'obj2:',objClearance_2)
    
    if objClearance_1 == 1:
      polygon1 = polygon_with_clearance(obj1, obj_df1)
    else:
      polygon1 = obj1.polygon
      
    if objClearance_2 == 1:
      polygon2 = polygon_with_clearance(obj2, obj_df2)
    else:
      polygon2 = obj2.polygon
#    _, _, _, _, polygon1 = rectangle_vertices(obj1.conf, obj1.length, obj1.width)

#    plot_polygon(polygon1)
#    _, _, _, _, polygon2 = rectangle_vertices(obj2.conf, obj2.length, obj2.width)
#    plot_polygon(polygon2)
    
    collision = polygon1.intersects(polygon2) # boolean variable showing the collision
    return collision

def door_obj_collision_detection(door, obj):
    '''
    This function uses other functions to check the collision between obj1 and
    obj2 
    :param obj1: in the form of [[x,y,theta], length, width] where (x,y) are 
    the coordinates of the center of the object, theta is the orientation of 
    the object with respect to x-axis, length and width are length and width 
    of the object. same goes for the second object, obj2
    '''
    collision = False
    if door.polygon.intersects(obj.polygon):
      collision = True
#      plot_polygon(door.polygon)
#      plot_polygon(obj.polygon)
#      plt.show()
#      print('Collision in the room')
    else:
#      print('Collision out of room?')
      poly1 = door.polygon
      wallPoint = [door.conf[0], door.conf[1]] # the point on the wall that we find the reflection of the polygon across
      wallPoint = [2*x for x in wallPoint]
      v1 = [] # array of the polygon vertices
      for x,y in poly1.exterior.coords:
        v1.append([x,y])
        v1[-1] = [c1-c2 for c1, c2 in zip(wallPoint, v1[-1])]
      newPoly = Polygon(v1)
#      plot_polygon(newPoly)
#      plot_polygon(door.polygon)
#      plot_polygon(obj.polygon)
#      plt.show()
      if newPoly.intersects(obj.polygon):
#        print('Collision out of the room')
        collision = True    
      
#    collision = door.polygon.intersects(obj.polygon) # boolean variable showing the collision
    return collision
