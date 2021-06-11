# -*- coding: utf-8 -*-
"""
Created on Thu Feb  6 16:16:09 2020

@author: local_ergo
"""
import os
import numpy as np
from pandas import DataFrame
import matplotlib.pyplot as plt
import statistics as stats
import random
import seaborn as sn
from numpy import linalg as la

#from class_FallRiskAssesment import FallRiskAssesment
from Fall_risk_assesment_1 import FallRiskAssesment
#from class_EnvironmentGenerated import Environment_Generated
from class_EnvironmentGenerated_CMAES import Environment_Generated
from functions_collision_detection import plot_polygon, plot_lights
from Trajectory_generation import generate_trajectory, define_obstacles
from functions_fitness import fitness_Distribution

#------------------------------------------------------------------------------                
#                           CLASSES
#------------------------------------------------------------------------------    

class CMAES_Room():
  '''
  nBest: number of chosen best solutions
  numPossibleSolution: number of generated solution
  nIteration: number of iterations for CMA-ES
  '''
  def __init__(self, nBest, numPossibleSolution, nIteration, mainRoom, bathRoom, intention_set):
      # -------------------------- ROOM GENERATION PARAMETERS -----------------------------  
        
        self.numOfRows = 22 # Number of grids in the room in Y direction.
        self.numOfCols = 29 # Number of grids in the room in X direction.
        # -------------------------- FALL MODEL PARAMETERS -----------------------------
        pdf_filenames = []
        png_filenames = []
        traj_png_filenames = []
        traj_pdf_filenames = []
        path = os.path.join(os.getcwd()) # The object library file address.
        design_name = "Room-3-Inboard-Headwall"
        day_night = "day"
        plots = True
        
        '''Setup parameters used for motion prediction and evaluation'''
    #    v = [1.2, 0.3] # maximum linear velocity = [mu, sigma]
        v = [0.5, 0.3] # maximum linear velocity = [mu, sigma]
        w = [1, 0.3] # maximum angular velocity = [mu, sigma]
        num_points = 500
    
        # Setup scenarios within the room
        num_trials = 36
        
        library_file = "{0}/Object_Library.csv".format(path) # Put the object library file address here.
        background_filename = "{0}/Room_Designs/{1}_objects_rotated.png".format(path, design_name)
#       background_filename = '{0}/generated-room.png'.format(path)
        
        for counter in range(num_trials):
            traj_pdf_filenames.append("{0}/results/{1}/{2}/Trajectories/{1}_{2}_traj_{3}.pdf".format(path, design_name, day_night, counter+1))
            traj_png_filenames.append("{0}/results/{1}/{2}/Trajectories/{1}_{2}_traj_{3}.png".format(path, design_name, day_night, counter+1))
        for factor in ["light", "floor", "door", "support", "baseline"]:
            pdf_filenames.append("{0}/results/{1}/{2}/{1}_{2}_{3}.pdf".format(path, design_name, day_night, factor))
            png_filenames.append("{0}/results/{1}/{2}/{1}_{2}_{3}.png".format(path, design_name, day_night, factor))
            png_filenames.append("{0}/results/{1}/{2}/{1}_{2}_{3}.png".format(path, design_name, day_night, factor))
    
        # ------------------------   CMA-ES MODEL PARAMETER--------------------------------------    
    
        lightMu = []; doorMu = []
        lightSigma = np.identity(2); doorSigma = np.identity(3)
        
        self.bestSol = [] # Best found solution over all
        self.bestSolIter = []  # to keep track of the best found solution in one iteration
        self.bestSolutionMean = []        
        self.bestSolutionStd = []        
        
        # ---------------------------    ROOM GENERATION  ---------------------------------        
        furnMainStat = []; furnBathStat= []
        lightMainStat = []; lightBathStat = []
        doorMainStat = []; doorBathStat = []
        counter = 0
        while counter < nIteration:
            fitness = []
            fallRisk = []
            temp = []
            self.elite = []
            self.env = []
            i = 0
            while i < numPossibleSolution:
                ''' Generating candid solutions'''
                i = int(i)
                print('___________________________ Room', i,'of', numPossibleSolution-1,'in iteration', counter, 'of', nIteration,' ____________________________________')
                print('Generating room ',i, '...')        
                self.env.append(Environment_Generated(mainRoom, bathRoom, furnMainStat, furnBathStat, lightMainStat, lightBathStat, doorMainStat, doorBathStat, self.numOfRows, self.numOfCols))
                
                # -------------------------------------------------------------------------------------   
                '''PLOTTING THE ROOM'''
                i = int(i)
                for door in self.env[i].doorList:
                        plot_polygon(door.polygon, fignum = i, title = 'Final Placement, room %i'%i)
            
                for furniture in self.env[i].furnitureList:
                    plot_polygon(furniture.polygon, fignum = i, title = 'Final Placement, room %i'%i)
                    
                for light in self.env[i].lightList:
                    plot_lights(light.point, fignum = i, title = 'Final Placement, room %i'%i)
                
                for room in self.env[i].roomList:
                    plot_polygon(room.polygon, fignum = i, title = 'Final Placement, room %i'%i)
                plt.show()
#                fallRisk[i].plotDistribution()  # Plot fall risk distribution

                #-------------------------------------------------------------------------------
                #                                FALL MODELL 
                #-------------------------------------------------------------------------------
                
                print('Baseline evaluating room', i,'in iteration', counter,'with the fall model...')
                fallRisk.append(FallRiskAssesment(self.env[-1])) # Initial FallRiskAssesment class
                fallRisk[-1].update(png_filenames, pdf_filenames, assistive_device = False, plot = plots) # # no assistive devices, Find scores for each baseline factor and baseline evaluation

                print('Motion Evaluation room', i, ' with the fall model...')
                # Initialization
                TrajectoryPoints = []
                cc = 0
            
                for intention in range(len(intention_set['start'])):
                  statCC = 0 # this varibale keeps track of trials for finding trajectories for each intention/scenarios
                  for trial in range(intention_set['frequency'][intention]):
                        # Generating a trajectory for each scenario each trial
                        print("Trajectory prediction for scenario {0}, trial {1}: ".format(intention+1, trial+1))
                        obstacles = define_obstacles(self.env[-1]) # Defines obstacles including furniture and walls
                        traj, trajStatus = generate_trajectory(intention_set['start'][intention], intention_set['end'][intention], self.env[-1], obstacles, random.gauss(v[0], v[1]), random.gauss(w[0], w[1]), num_points)
                        print('Trajectory status:', trajStatus)
                        if trajStatus != 2:
                          statCC += 1 
                          print('Trajectory was not found in trial',trial, 'and statCC is:', statCC)
                          if statCC == intention_set['frequency'][intention]:

                            print('No trajectories were found for', intention_set['start'][intention],'to',intention_set['end'][intention]  , 'resetting room number ', i, '...')
                            del self.env[-1]
                            del fallRisk[-1]
                            break
                        
                        if statCC == intention_set['frequency'][intention]:
                          break
                        cc += 1
                        # Evaluating the generated trajectory
                        TrajectoryPoints.append(fallRisk[-1].getDistibutionForTrajectory(traj, cc, background_filename, traj_png_filenames, traj_pdf_filenames, plot=plots, assistive_device = False, baselinefig = fallRisk[i].baselinefig, baselineAx = fallRisk[i].baselineAx))

                  if statCC == intention_set['frequency'][intention]:
                      break
                if statCC == intention_set['frequency'][intention]:
                  continue
                
                print("Final Scores Calculation of room ",i,' ...')
                num = np.ones([self.env[-1].numOfRows,self.env[-1].numOfCols]) # Initializing number of points in each grid cell as one

                for traj in TrajectoryPoints:
                    for point in traj:
                        [m,n] = fallRisk[i].meter2grid(point[0][0]) # Finding the grid cell for each point in trajectories
                        fallRisk[-1].scores[m,n] += point[1] # Add the score of that point to the associated grid cell
                        num[m,n] += 1 # Add 1 to number of points inside that grid cell
                for r in range(self.env[-1].numOfRows):
                    for c in range(self.env[i].numOfCols):
                        fallRisk[-1].scores[r,c] = fallRisk[-1].scores[r,c]/num[r,c] # Take the avarage score for each grid cell
                        
#                print("Final evaluation plot...")
#                fallRisk.plotDistribution(fallRisk.scores, env, 'final', 'hamming')
#                fallRisk[i].plotDistribution(fallRisk[i].scores, self.env[i], 'final', 'nearest')
                
                fitness.append(fitness_Distribution(fallRisk[i])) 
#                print(' scores are: \n', fallRisk[i].scores, '\n and the mean value is: ', fallRisk[i].scores.mean())
                print('The score for room  ',i,' is: ', fitness[-1])
                i += 1  
            
            #-------------------------------------------------------------------------------
            #                                OPTIMIZATOION
            #-------------------------------------------------------------------------------           
            print('Optimization...')
            argBestSol = fitness.index(min(fitness))
            self.bestSolIter.append(fitness[argBestSol])              
            print(' The best solution value is: ', self.bestSolIter[counter], ' for iteration: ', counter)
            # Keepting track of the best found solution to this counter
            if counter == 0:
              self.bestSol.append(self.bestSolIter[counter])
            else:
              if self.bestSol[-1] > self.bestSolIter[counter]:
                self.bestSol.append(self.bestSolIter[counter])
              else:
                self.bestSol.append(self.bestSol[-1])
#            print('counter is:', counter, 'self.bestSol[-1] :',self.bestSol[-1], 'self.bestSol :', self.bestSol)
    
            arg_topNBest = np.argsort(fitness)[:nBest] # Choosing the indecis of the best candid solutions
            
            for arg in arg_topNBest:
                temp.append(fitness[arg]) # elite fitness values
                self.elite.append(self.env[arg]) # elite environments    
                
            self.bestSolutionMean.append(sum(temp) /len(temp)) # keeping track of the best solution found at each round
            self.bestSolutionStd.append(np.std(temp))
#            print('Round ', counter, ' of the rooms is done and the best average fall risk is: ', self.bestSolutionMean[counter])

            self.furniture = Elite_Furniture(self.elite, self.env, arg_topNBest)
            self.light = Elite_Light(self.elite, self.env, arg_topNBest)
            
#            mainfurnMu = self.furniture.mainGaussTranslate
#            mainfurnSigma = self.furniture.mainGaussTransform
            furnMainStat = [self.furniture.mainGaussTranslate, self.furniture.mainGaussTransform]
            
#            bathfurnMu = self.furniture.bathGaussTranslate
#            bathfurnSigma = self.furniture.bathGaussTransform
            furnBathStat = [self.furniture.bathGaussTranslate, self.furniture.bathGaussTransform]
            
#            lightMu = self.light.mainGaussTranslate
#            lightSigma = self.light.mainGaussTransform
            lightMainStat = [self.light.mainGaussTranslate, self.light.mainGaussTransform]
#            lightMu = self.light.bathGaussTranslate
#            lightSigma = self.light.bathGaussTransform
            lightBathStat = [self.light.bathGaussTranslate, self.light.bathGaussTransform]
    
            counter += 1 # keeps track of how many times the mu and sigma are updated
            

class Elite_Furniture():
    ''' This class finds the configuration and mean value for the each piece
    of furniture in the list of environments that is being sent to it as 
    the "elite_env" '''
    def __init__(self, elite, env, arg_topk):        
        self.eliteLight = [] # Elite Furniture
        self.envLight = []  # All Furniture
        self.eigenVal = []
        self.eigenVec = []
        self.eliteCovar = []    # covariance of the elite set
        self.eliteFurniture = []
        ###################################################################
#        self.GaussTransform = []
#        self.GaussTranslate = []
        self.mainGaussTransform = []
        self.mainGaussTranslate = []
        self.bathGaussTransform = []
        self.bathGaussTranslate = []
        ###################################################################333
        self.envFurniture = []
        
        i = 0 # i is the index for the furniture in hand
        while i < len(elite[0].furnitureList):
          self.eliteFurniture.append(Furniture_Define(elite, i)) # fruniture class objects
          self.envFurniture.append(Furniture_Define(env, i))
          
          ''' Creating the covariance matrix for the elite set'''

          self.eliteCovar.append(np.cov(np.transpose(self.eliteFurniture[i].conf)))
#            sn.heatmap(self.eliteCovar[-1], annot=True, fmt='g')
#            plt.show()
          w, E = la.eigh(self.eliteCovar[i]) # eigenvalue of the centered matrix
          self.eigenVal.append(w)
          self.eigenVec.append(E)
          ''' square root of omega, taking care of negative values'''
#            print('omega  is: ', w)
          for idx, val in enumerate(w):
              if val<0:
                  w[idx]=-(abs(val)**0.5)
              else:
                  w[idx] = val**0.5        
#            print('\n and omega sqr is: ', w)
          if elite[0].furnitureList[i].room == 'main_room':
            self.mainGaussTransform.append(E@np.diag(w)) # Transformation            
            self.mainGaussTranslate.append(self.eliteFurniture[i].meanVal) #Translation
          else:
            print('bath covariance:', self.eliteCovar)
            self.bathGaussTransform.append(E@np.diag(w)) # Transformation            
            self.bathGaussTranslate.append(self.eliteFurniture[i].meanVal) #Translation
          i += 1
        
class Furniture_Define():
    ''' This class is defined for each TYPE OF FURNITURE. All environments with 
    the furnitures specific index is being passed to this function for further 
    analysis '''
    def __init__(self, env, ind):
        i = 0
        self.conf = []
        while i< len(env):
            self.conf.append(env[i].furnitureList[ind].conf)
            i +=1
        self.conf = np.array([np.array(xi) for xi in self.conf]) # configuration
        self.meanVal = self.conf.mean(0)    # mean value
        

class Elite_Light():
    ''' This class is defined to extract useable feature for lights in the elite poopulation'''
    def __init__(self, elite, env, arg_topNBest):
        self.eliteLight = [] # Elite Furniture
        self.envLight = []  # All Furniture
        self.eigenVal = []
        self.eigenVec = []
        self.eliteCovar = []
        self.mainGaussTransform = []
        self.mainGaussTranslate = []
        self.bathGaussTransform = []
        self.bathGaussTranslate = []
        
        i = 0
        while i<len(elite[0].lightList):
          self.eliteLight.append(Light_Define(elite, i))
          self.envLight.append(Light_Define(env, i))
          ''' Creating the covariance matrix for the elite set'''
          nBest = len(self.eliteLight[0].pos)
          centered = np.transpose(self.eliteLight[i].pos - self.envLight[i].meanVal)
          self.eliteCovar.append((centered@np.transpose(centered))/(nBest))
          w, E = la.eigh(self.eliteCovar[i]) # eigenvalue of the centered matrix
          self.eigenVal.append(w)
          self.eigenVec.append(E)
          ''' omega square, taking care of negatve values'''
          for idx, val in enumerate(w):
              if val<0:
                  w[idx]=-(abs(val)**0.5)
              else:
                  w[idx] = val**0.5        
#            print('\n and omega sqr is: ', w)
                  
#          self.GaussTransform.append(E@np.diag(w)) # Transformation            
#          self.GaussTranslate.append(self.eliteLight[i].meanVal) #Translation
          if elite[0].lightList[i].room == 'main_room':
            self.mainGaussTransform.append(E@np.diag(w)) # Transformation            
            self.mainGaussTranslate.append(self.eliteLight[i].meanVal) #Translation
          else:
            self.bathGaussTransform.append(E@np.diag(w)) # Transformation            
            self.bathGaussTranslate.append(self.eliteLight[i].meanVal) #Translation
          i += 1

        
class Light_Define():
    ''' This class is defined for each TYPE OF Light. All environments with 
    the light specific index is being passed to this function for further 
    analysis '''
    def __init__(self, env, ind):
        i = 0
        self.pos = []
        while i< len(env):
            self.pos.append(env[i].lightList[ind].pos)
            i +=1
        self.pos = np.array([np.array(xi) for xi in self.pos]) # configuration
        self.meanVal = self.pos.mean(0)    # mean value
