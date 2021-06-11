# -*- coding: utf-8 -*-
"""
Created on Wed Jul 22 22:50:32 2020

@author: local_ergo
"""
import numpy as np
from shapely.geometry import Polygon

def polygon_scale(polygon, scalingFactor):
  ''' This function creates a bigger polynomial with the scaling factor that is sent to 
  the function around the original polynomial, also passed to the function.
  '''
  corners = np.asarray(polygon.exterior.coords)
  newCorners = [] # new polynomial corners
  transVec = [] # translation vector for the polygons
  
  for corner in corners:
    newX = corner[0]*scalingFactor
    newY = corner[1]*scalingFactor
    newCorners.append((newX, newY))
  newPoly = Polygon(newCorners)
    
  # translating the scaled polygon
  _, polyCent = polygon_center(polygon)
  _, newPolyCent = polygon_center(newPoly)
#  print(polyCent[0])
#  print(newPolyCent[0])
  transVec.append(newPolyCent[0]-polyCent[0])
  transVec.append(newPolyCent[1]-polyCent[1])
  
  newNewCorners = []
  for corner in newCorners:
    corner = list(corner)
    tempX = (corner[0]-transVec[0])
    tempY = (corner[1]-transVec[1])
    newNewCorners.append((tempX,tempY))
  newPoly = Polygon(newNewCorners)
    
  return newPoly
  
def polygon_center(polygon):
  ''' this function takes in a polygon bject and return the center of the polygon
  as a point object (polyCenter) and as coordinate ([polyCenterX, polyCenterY])
  '''
  polyCenter = polygon.centroid.coords
  polyCenterX = polyCenter[0][0]
  polyCenterY = polyCenter[0][1]
  
  return(polyCenter, [polyCenterX, polyCenterY])
#
#poly = Polygon([(1,1),(2,0),(3,1),(2,2)])
#newPoly = polygon_scale(poly, 2)