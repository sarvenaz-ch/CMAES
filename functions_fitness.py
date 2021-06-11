# -*- coding: utf-8 -*-
"""
Created on Thu Apr 30 11:26:40 2020

@author: sarve
"""
import numpy as np
import statistics as stats

#def environment_mean()


def fitness_Distribution (fallRisk):

     fitnessMean = np.mean(fallRisk.scores)
     fitnessMedian = np.median(fallRisk.scores)
     fitnessStd = np.std(fallRisk.scores)
     fitnessMax = np.amax(fallRisk.scores)
     alpha = 0.95*fitnessMax # the threshold that we define for the area under the curve, under the right tail of the distribution
     
     fitness = fitnessMedian + fitnessMax + ((alpha - fitnessMean)/fitnessStd)
     
     return fitness