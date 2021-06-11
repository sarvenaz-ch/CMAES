''' 
This main file uses imported functions and classes to create different room and
evaluate them with the base fall model. The base fall model does not have the
trajectpries implemented yet
'''
#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
#from functions_collision_detection import plot_lights, plot_polygon # for plotting the rooms
from class_CMAESRoom import CMAES_Room

#
#############################################################################
#                                 MAIN
#############################################################################

if __name__ == "__main__": # Run when it is called, not when it is imported
    
    ''' ______________________ ROOM GENERATOR ___________________________'''
    
    ''' WHICH FURNITURE, DOOR AND LIGHTS ARE USED '''
    mainFurnitureCodeArray = [8, 14, 16]
#    mainFurnitureCodeArray = [23,24, 28, 30, 16]
    mainDoorsCodeArray = [10] #*** MAIN DOORS ONLY TAKES IN THE DOORS THAT ARE NOT THE DOOR BETWEEN MAIN ROOM AND BATHROOM ****
#    mainDoorsCodeArray = []
    mainLightCodeArray = [2, 30]
#    mainLightCodeArray = []
    
    bathFurnitureCodeArray = [5,9]
#    bathFurnitureCodeArray = []
    bathDoorsCodeArray = [11] # *** BATHROOMDOORS ARE ONLY THE DOOR THAT IS BETWEEN THE MAIN ROOM AND THE BATHROOM
#    bathDoorsCodeArray = []
    bathLightCodeArray = [2]
    bathLightCodeArray = []

    mainRoom = [mainFurnitureCodeArray, mainDoorsCodeArray, mainLightCodeArray, 'main_room']
    bathRoom = [bathFurnitureCodeArray, bathDoorsCodeArray, bathLightCodeArray, 'bath_room']

    ''' ____________________________ FALL MODEL ___________________________'''

# intention_set = {'start': ['Bed', 'Main Door', 'Bed', 'Toilet', 'Sink-Bath', 'Chair-Patient', 'Toilet', 'Bed', 'Chair-Patient', 'Bed', 'Chair-Visitor'], 'end': ['Main Door', 'Bed', 'Toilet', 'Sink-Bath', 'Bed', 'Toilet', 'Chair-Patient', 'Chair-Patient', 'Bed', 'Chair-Visitor', 'Bed'], 'frequency': [3, 3, 6, 6, 6, 3, 3, 2, 2, 1, 1]} # for P22 & Room-4 & Room-2 designs
#    intention_set = {'start': ['Bed', 'Main Door', 'Bed', 'Toilet', 'Sink-Bath', 'Chair-Patient', 'Toilet', 'Bed', 'Chair-Patient', 'Bed', 'Sofa'], 'end': ['Main Door', 'Bed', 'Toilet', 'Sink-Bath', 'Bed', 'Toilet', 'Chair-Patient', 'Chair-Patient', 'Bed', 'Sofa', 'Bed'], 'frequency': [3, 3, 6, 6, 6, 3, 3, 2, 2, 1, 1]} # for A-K & S-B & J-M & J-C & J-G & B-L & B-JH & Room-1 & Room-3 designs
    intention_set = {'start': ['Bed', 'Bed'], 'end': ['Sofa', 'Toilet'], 'frequency': [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]} # for test
#    intention_set = {'start': ['Sofa', 'Sofa'], 'end': ['Cabinet', 'Chair-Patient'], 'frequency': [2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1]} # for test

    ''' _______________________   OPTIMIZATION __________________________'''
    ''' The size of elite population and consequently the size of the possible solution
    should be relatively large to avoid small covariance matrix. With small covariance matrices
    there is a chance the program gets stuck in an infinite loop of object placement without collision
    '''
    nBest =  5  # number of elite solutions. Size of the elite population
    numPossibleSolution = 10 # Number of possible solutions, size of the candid solutions
    numOfIteration = 70 # number of iteration for optimization
    optimization = CMAES_Room(nBest, numPossibleSolution, numOfIteration, mainRoom, bathRoom, intention_set)
    
#    plt.plot(optimization.m0x); plt.ylabel('object 0, x-axis'); plt.xlabel('# of iterations'); plt.show()
#    plt.plot(optimization.m1x); plt.ylabel('object 1, x-axis'); plt.xlabel('# of iterations'); plt.show()
#    
#    plt.plot(optimization.m0y); plt.ylabel('object 0, y-axis'); plt.xlabel('# of iterations'); plt.show()
#    plt.plot(optimization.m1y); plt.ylabel('object 1, y-axis'); plt.xlabel('# of iterations'); plt.show()
#    
#    plt.plot(optimization.m0theta); plt.ylabel('object 0, orientation'); plt.xlabel('# of iterations'); plt.show()
#    plt.plot(optimization.m1theta); plt.ylabel('object 1, orientation'); plt.xlabel('# of iterations'); plt.show()
#            
    summed_list = [optimization.bestSolutionMean[i] + optimization.bestSolutionStd[i] for i in range(len(optimization.bestSolutionMean))]
    subtracted_list = [optimization.bestSolutionMean[i] - optimization.bestSolutionStd[i] for i in range(len(optimization.bestSolutionMean))]
    plt.plot(np.linspace(0,numOfIteration-1, numOfIteration,numOfIteration), optimization.bestSolutionMean, color = 'crimson'); plt.fill_between(np.linspace(0,numOfIteration-1, numOfIteration,numOfIteration), subtracted_list, summed_list, facecolor = 'coral', alpha = 0.5) 
#    plt.ylabel('Best Solution Value'); plt.xlabel('# of iterations'); plt.show()
              
    plt.plot(optimization.bestSol)
    plt.ylabel('Best Solution Value'); plt.xlabel('# of iterations'); plt.show()
