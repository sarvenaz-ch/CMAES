# -*- coding: utf-8 -*-
"""
Created on Sat Aug  1 13:36:58 2020
This file contains functions needed for removing the section of the wall that is 
covered by a door cross section so the trajectories can go through it
@author: Sarvenaz Chaeibakhsh
"""
import numpy as np
from math import sqrt
from math import cos, sin, tan, radians, atan2, degrees, pi
from shapely.geometry import Polygon, Point

from functions_geometry import polygon_center

def distance(p1,p2):
  ''' This function measures the distance between points p1 and p2'''
  dist = sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
  return dist

def is_between(p1, p2, q):
  ''' This function checks if point q is in between points p1 and p2'''
  isBetween = distance(p1,q) + distance(p2,q) == distance(p1, p2)
  return isBetween
