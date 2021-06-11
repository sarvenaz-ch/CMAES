# -*- coding: utf-8 -*-
"""
Created on Tue Jan 21 09:30:03 2020
This file contains class and functions used for evaluating 
@author: Sarvenaz Chaeibakhsh
"""

import numpy as np
import copy
import math
import matplotlib.pyplot as plt
import os
#%matplotlib inline
import matplotlib.cbook as cbook
import matplotlib.image as image
from shapely.geometry import Polygon, Point
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib._png import read_png
class FallRiskAssesment():

	def __init__(self, env):
		self.env = env # env is an Environment class.
		self.scores = np.zeros([self.env.numOfRows,self.env.numOfCols]) # Initializing scores as a zero matrix with size of number_of_rows * number_of_columns.
		self.theta_d = [0.8,0.8,1.5] # [f_min, d_min, d_max] used in the supporting object factor
		self.theta_l = [30,70,100] # [night_min, night_max/day_min, day_max] used in the light factor
		self.theta_r = 1 #
		self.theta_v = 1 #
		self.ESP = [] # Initializing External Supporting Point list as an empty array.
		self.occupied = np.zeros([self.env.numOfRows, self.env.numOfCols])
		for obj in self.env.furnitureList:
			self.update_ESP(obj) # For each object in list of objects, the surrounding grids are updated basd on support level of the object.

	def update_ESP(self, obj):
		"""In this function, we find the effect of the input object on all the grids in the environment.
        When a grid is occupied we find from which direction it is free and add that as a External Supporting Point, 
        if it not free from any direction, it is not added as a ESP and only it is marked as an occupied cell."""
		for row in range(self.env.numOfRows):
			for col in range(self.env.numOfCols):
				coordinate = self.grid2meter([row, col]) # Changing grid to meter coordinate system
				gridPoint = Point(coordinate)
				if gridPoint.within(obj.polygon):
					coordinate = self.grid2meter([row+1, col]) # Changing grid to meter coordinate system
					rightGridPoint = Point(coordinate)
					if not rightGridPoint.within(obj.polygon):
						self.ESP.append([rightGridPoint, 3, obj.support])
					coordinate = self.grid2meter([row, col+1]) # Changing grid to meter coordinate system
					upGridPoint = Point(coordinate)
					if  not upGridPoint.within(obj.polygon):
						self.ESP.append([upGridPoint, 2, obj.support])
					coordinate = self.grid2meter([row-1, col]) # Changing grid to meter coordinate system
					leftGridPoint = Point(coordinate)
					if not leftGridPoint.within(obj.polygon):
						self.ESP.append([leftGridPoint, 1, obj.support])
					coordinate = self.grid2meter([row, col-1]) # Changing grid to meter coordinate system
					downGridPoint = Point(coordinate)
					if not downGridPoint.within(obj.polygon):
						self.ESP.append([downGridPoint, 0, obj.support])
					self.occupied[row, col] = 1

	def findFactors(self, grid):
		gridCoordinate = self.grid2meter(grid)
		gridPoint = Point(gridCoordinate)

		factors = [] # Initializing factors as an empty array
		factors.append(self.closestSupportDistance_effect(gridPoint)) # Adding the effect of ESP on this grid.
		factors.append(self.floor_effect(grid)) # Adding the effect of ESP on this grid.
		factors.append(self.lighting_effect(gridPoint)) # Adding the effect of lighting on this grid.
		factors.append(self.door_passing_effect(gridPoint)) # Adding the effect of door passing on this grid.

		return factors # returning the array of all factors

	def floor_effect(self,grid):
		floorTypeRisk = self.env.floor[grid[0], grid[1]] # Initializing the effect of floor type on this grid base on its own floor type.
		for i in [-1,1]: # Cheking if there is any transition to another type of floor or not in the surrounding grids and modifying the effect of floor type.
			if grid[0]+i < self.env.numOfRows and grid[0]+i > 0:
				if self.env.floor[grid[0]+i, grid[1]]!= self.env.floor[grid[0], grid[1]]:
					floorTypeRisk = floorTypeRisk * 1.1
			if grid[1] + i < self.env.numOfCols and grid[1] + i > 0:
				if self.env.floor[grid[0], grid[1]+i]!= self.env.floor[grid[0], grid[1]]:
					floorTypeRisk = floorTypeRisk * 1.1
		return floorTypeRisk

	def door_passing_effect(self, gridPoint):
		""" This function calculates the effect of door passing for a given grid. It needs to be updated. """
		risk = 1
		for door in self.env.doorList:
			if gridPoint.within(door.polygon) == True:
				risk = float(1)/door.support
		return risk

	def lighting_effect(self, gridPoint):
		""" This function calculates the effect of lighting for a given grid. It needs to be updated. """
		risk = 1
		light_intensity = 0
		for light in self.env.lightList:
			if self.in_the_same_room(light.point, gridPoint):
				dist = self.distance(light.point, gridPoint)
				if dist == 0:
					light_intensity += self.theta_l[2]
				else:
					light_intensity += float(1)/((dist/80)**2)*light.intensity
		if light_intensity <= self.theta_l[0]:
			risk = risk * 1.7
		elif light_intensity > self.theta_l[0] and light_intensity < self.theta_l[1]:
			risk = risk * 1.5
		elif light_intensity > self.theta_l[1] and light_intensity < self.theta_l[2]:
			risk = risk * 1.2
		return risk

	def closestSupportDistance_effect(self, gridPoint):
		''' This function calculates the effect of ESPs for a given grid. It needs to be updated. '''
		risk = 1
		min_dist = self.theta_d[2]
		support_type = 1
		for support in self.ESP:
			if self.in_the_same_room(support[0], gridPoint):
				dist = self.distance(support[0], gridPoint)
				if dist < min_dist:
					min_dist = dist
					support_type = support[2]
#		for wall in self.env.walls:
#			wall_dist = self.distance_wall(gridPoint, wall)
#			if wall_dist < min_dist:
#				min_dist = wall_dist
#				support_type = 1.1
		if min_dist <= self.theta_d[1]:
			risk = self.theta_d[0]
		elif min_dist > self.theta_d[1] and min_dist <= self.theta_d[2]:
			risk = self.theta_d[0] + (min_dist - self.theta_d[1])*(1-self.theta_d[0])/(self.theta_d[2]-self.theta_d[1])
		risk = float(risk) / support_type
		return risk

	def in_the_same_room(self, grid1, grid2):
		""" This function determines whether two grids are in the same room or not. It is mostly used for lights to make sure it doesn't pass through walls. """
		room_grid_1 = "out"
		room_grid_2 = "out"
		for room in self.env.roomList:
			if grid1.within(room.polygon) == 1:
				room_grid_1 = room.name
			if grid2.within(room.polygon) == 1:
				room_grid_2 = room.name
		if room_grid_1 == room_grid_2:
			return True
		else:
			return False

	def distance(self, obj, gridPoint):
		""" This function finds the Euclidean distance between two grids."""
		dist_x =  obj.x - gridPoint.x
		dist_y = obj.y - gridPoint.y
		dist = np.sqrt(dist_x**2+dist_y**2)
		return dist

	def distance_wall(self, point, wall):
		''' This function calculates the minimum distance from a grid and a wall segment'''
		LineMag = math.sqrt(math.pow((wall[0][0] - wall[0][2]), 2)+ math.pow((wall[0][1] - wall[0][3]), 2))

		if LineMag < 0.00000001:
			DistancePointLine = float("inf")
			return DistancePointLine

		u1 = (((point.x - wall[0][0]) * (wall[0][2] - wall[0][0])) + ((point.y - wall[0][1]) * (wall[0][3] - wall[0][1])))
		u = u1 / (LineMag * LineMag)

		if (u < 0.00001) or (u > 1):
			#// closest point does not fall within the line segment
			DistancePointLine = float("inf")
			return DistancePointLine
		else:
			# Intersecting point is on the line, use the formula
			ix = wall[0][0] + u * (wall[0][2] - wall[0][0])
			iy = wall[0][1] + u * (wall[0][3] - wall[0][1])
			DistancePointLine = math.sqrt(math.pow((point.x - ix), 2)+ math.pow((point.y - iy), 2))
		return DistancePointLine

	def grid2meter(self, grid):
	    ''' grid to meter'''
	    x = grid[0] * self.env.unit_size_m
	    y = grid[1] * self.env.unit_size_m
	    return (x,y)          
          
	def real_score(self, score):
		''' this function goes through the matrix of scores and get rid of any 
        value less than 0.3. In this code, it is assume that all the grids inside
        the room have risk fall scores higher than 0.3. If it is <=0.3, then the 
        grid is out of the room and its value does not have any meaning for us
        Added by Sarvenaz'''
        
		realScore = []
		row = 0; 
		while row < score.shape[0]: # number of rows in the score matrix
			col = 0;
			while col < score.shape[1]: # number of columns in the score matrix
				if score[row][col] > 0.3:
					realScore.append(score[row][col])
				col += 1
			row +=1
		return realScore
    
	def update(self, assistive_device):
		""" This function updates the fall risk score based on all the factors except the trajectory. It needs to be updated. """
		for j in range(self.env.numOfCols):
			for i in range(self.env.numOfRows):
				if self.occupied[i,j] == 1:
					self.scores[i, j] = 0.2
				else:
					if assistive_device == False:
						assistive_device_risk = 1
					else:
						assistive_device_risk = 1.05
					factors = self.findFactors([i,j])
					self.scores[i, j] = assistive_device_risk
					for k in range(len(factors)):
						self.scores[i, j] = self.scores[i, j]*factors[k]
		self.realScores = self.real_score(self.scores)
	            # print self.scores

	def trajectoryRiskEstimate(self, point):
		""" This function calculates the effect of trajectory. It needs to be updated. """
		risk = 1
		if point[1] == 'walking':
			risk *= 1.3
		elif point[1] == 'sit-to-stand':
			risk *= 2
		elif point[1] == 'stand-to-sit':
			risk *= 2
		if point[0][4] != 0:
			risk *= math.exp(-self.theta_r*abs(point[0][3]/point[0][4]))+ self.theta_v*point[0][3]
		return risk

	def getDistibutionForTrajectory(self, trajectory, plot, assistive_device):
		""" Finding the risk distribution over a given trajectory defined by waypoints. """
		self.update(assistive_device)
		TrajectoryScores = []
		for point in trajectory:
			# print point
			# point: [[x, y, phi, v, w],"activity"]
			grid = self.env.find_grid_from_measure(point[0])
			PointScore = self.scores[grid[0],grid[1]]
			PointScore += 0.3*self.trajectoryRiskEstimate(point)
			TrajectoryScores.append(PointScore)
		if plot:
			self.plotTrajDist(TrajectoryScores, trajectory)

		return TrajectoryScores

	def plotDistribution(self):
		fig, ax =plt.subplots()
		data = plt.imshow(self.scores, cmap='jet', interpolation='nearest', vmin=0, vmax=1.5)
		plt.xlim((0-0.5, self.env.numOfCols-0.5))
		plt.ylim((0-0.5, self.env.numOfRows-0.5))
		fig.colorbar(data, ax=ax)
		# Major ticks every 20, minor ticks every 5
		major_ticks_x = np.arange(0, self.env.numOfCols, 5)
		minor_ticks_x = np.arange(0, self.env.numOfCols, 1)
		major_ticks_y = np.arange(0, self.env.numOfRows, 5)
		minor_ticks_y = np.arange(0, self.env.numOfRows, 1)

		ax.set_xticks(major_ticks_x)
		ax.set_xticks(minor_ticks_x, minor=True)
		ax.set_yticks(major_ticks_y)
		ax.set_yticks(minor_ticks_y, minor=True)
		ax.grid(which='minor', alpha=0.4)
		ax.grid(which='major', alpha=0.7)
		plt.show()

	def plotTrajDist(self, trajFallRisk, trajectory):
		x = []
		y = []
		dydx = []
		for i in range(len(trajectory)):
			x.append(trajectory[i][0][1])
			y.append(trajectory[i][0][0])
			dydx.append(trajFallRisk[i])

		# Create a set of line segments so that we can color them individually
		# This creates the points as a N x 1 x 2 array so that we can stack points
		# together easily to get the segments. The segments array for line collection
		# needs to be (numlines) x (points per line) x 2 (for x and y)
		points = np.array([x, y]).T.reshape(-1, 1, 2)
		segments = np.concatenate([points[:-1], points[1:]], axis=1)

		fig, ax = plt.subplots()
		# Create a continuous norm to map from data points to colors
		datafile = cbook.get_sample_data("/home/roya/catkin_ws/src/pam_manipulation_planning/src/Risk_Aware_Planning/VA_room_QR_day_2.png", asfileobj=False)
		im = image.imread(datafile)
		ax.imshow(im, aspect='auto', extent=(0, 10, 0, 10), alpha=0.5, zorder=-1)

		norm = plt.Normalize(min(dydx), max(dydx))
		lc = LineCollection(segments, cmap='jet', norm=norm)
		# Set the values used for colormapping
		lc.set_array(np.array(dydx))
		lc.set_linewidth(4)
		line = ax.add_collection(lc)
		fig.colorbar(line, ax=ax)
		plt.xlim(0, 10)
		plt.ylim(0, 10)
		plt.show()

