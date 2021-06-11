from gurobipy import *
import math
import numpy as np
import matplotlib.pyplot as plt


def OptPath_patient2(state_s, state_f, v_max, obstacles, n, assistive_device):
    # print(obstacles)
    # print(state_s)
    # print(state_f)
    dt = 0.5
    big_M = 100000000
    if assistive_device:
        w = [2, 3, 0]
    else:
        w = [0, 3, 0]
    # Create a new model
    Patient_PathPlanner = Model("Patient_path")
    Patient_PathPlanner.reset(0)
    # Create variables
    n =20
    Patient_PathPlanner.setParam('OutputFlag', False) # stop Gurobi from printing
    Patient_PathPlanner.setParam('TimeLimit', 30.0); # Time limit for generating a trajectory
    Patient_PathPlanner.setParam('OutputFlag', 0)
    Patient_PathPlanner.params.threads = 1
    Patient_PathPlanner.modelSense = GRB.MAXIMIZE
    Patient_PathPlanner.update()
#    Patient_PathPlanner.setParam('IterationLimit', 10)
    states = Patient_PathPlanner.addVars(n, 2, lb=0, ub=10, vtype=GRB.CONTINUOUS, name="states")
    dstates = Patient_PathPlanner.addVars(n-1, 2, lb=-5, ub=5, vtype=GRB.CONTINUOUS, name="dstates")
    u = Patient_PathPlanner.addVars(len(obstacles), n+1, 4, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.BINARY, name="u")
    # Set objective
    Patient_PathPlanner.setObjective( w[0]*sum([(states[k,l]- obstacles[o][2][l])*(states[k,l]- obstacles[o][2][l]) for k in range(n) for l in range(2) for o in range(len(obstacles))]) + w[1]*sum([(dstates[i,k]*dstates[i,k]) for k in range(2) for i in range(n-1)]) , GRB.MINIMIZE)

    # Add position, velocity and acceleration constraints
    for i in range(2):
        Patient_PathPlanner.addConstr(states[0, i] == state_s[i])
        Patient_PathPlanner.addConstr(states[n-1, i] == state_f[i])
        for j in range(n-1):
            Patient_PathPlanner.addConstr(dstates[j,i] == (states[j+1, i]-states[j, i]))
    for j in range(n-1):
        Patient_PathPlanner.addConstr(v_max[0] * dt * v_max[0] * dt>= (dstates[j,1] * dstates[j,1]) + (dstates[j,0] * dstates[j,0]))
    # Add Obstacle constraints
    for n_obs in range(len(obstacles)):
        for i in range(0, n):
            for k in range(4):
                Patient_PathPlanner.addConstr((states[i, 1] - obstacles[n_obs][0][k]*states[i, 0] - obstacles[n_obs][1][k] )*(obstacles[n_obs][2][1] - obstacles[n_obs][0][k]*obstacles[n_obs][2][0] - obstacles[n_obs][1][k]) <= - (u[n_obs, i, k]-1)*big_M)
        for i in range(0,n):
            Patient_PathPlanner.addConstr(sum(u[n_obs, i, ii] for ii in range(0, 4)) >= 1)

    Patient_PathPlanner.optimize() 
#    print_trajectory_status(Patient_PathPlanner.Status)
        
        
    path = []
    cost = float('inf')
    if Patient_PathPlanner.Status == 2:
        cost = Patient_PathPlanner.objVal
        phi = [state_s[2]]
        for i in range(n-1):
            x1 = states[i,0].x
            y1 = states[i,1].x
            x2 = states[i+1,0].x
            y2 = states[i+1,1].x
            new_phi = math.atan2(y2-y1,x2-x1)
            v = math.sqrt((x2-x1)**2+(y2-y1)**2)/dt
            w = (new_phi-phi[-1])/dt
            phi.append(new_phi)
            path.append([states[i,0].x, states[i,1].x, phi[-1], v, w])
	# rbf_plot(t1, n, n_rbf, dt, epsilon, we, phi_sum)

    return cost, path, Patient_PathPlanner.Status


def OptPath_patient(state_s, state_f, v_max, obstacles, n, assistive_device):
    T = 15
    dt = float(T)/n
    epsilon = 1
    s = 1
    n_rbf = int(T/(epsilon*s))
    t1 = np.linspace(0, T, n_rbf)
    d = np.ones(n_rbf)
    theta_rbf = np.linspace(0, T, n)
    phi_rbf, phi_sum = Rbf(t1, d, theta_rbf, n_rbf, epsilon)
    big_M = 100000000
    if assistive_device:
        w = [2, 3, 0]
    else:
        w = [0, 3, 0]
    # Create a new model
    Patient_PathPlanner = Model("Patient_path")
    Patient_PathPlanner.reset(0)
    # Create variables
    Patient_PathPlanner.setParam('OutputFlag', 0)
    states = Patient_PathPlanner.addVars(n, 2, lb=0, ub=10, vtype=GRB.CONTINUOUS, name="states")
    weights = Patient_PathPlanner.addVars(n_rbf, 2, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="weights")
    dstates = Patient_PathPlanner.addVars(n-1, 2, lb=-5, ub=5, vtype=GRB.CONTINUOUS, name="dstates")
    u = Patient_PathPlanner.addVars(len(obstacles), n+1, 4, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.BINARY, name="u")
    # Set objective
    Patient_PathPlanner.setObjective( w[0]*sum([(states[k,l]- obstacles[o][2][l])*(states[k,l]- obstacles[o][2][l]) for k in range(n) for l in range(2) for o in range(len(obstacles))]) + w[1]*sum([(states[i,k]-state_f[k])*(states[i,k]-state_f[k]) for k in range(2) for i in range(n)]) + w[2] * sum([weights[j,i] * weights[j,i] for i in range(2) for j in range(n_rbf)]) , GRB.MINIMIZE)

    # Add position, velocity and acceleration constraints
    for i in range(2):
        Patient_PathPlanner.addConstr(states[0, i] == state_s[i])
        for j in range(n):
            Patient_PathPlanner.addConstr(states[j, i] == sum(weights[k,i] * phi_rbf[j][k] for k in range(n_rbf))/phi_sum[j])
        for j in range(n-1):
            Patient_PathPlanner.addConstr(dstates[j,i] == (states[j+1, i]-states[j, i]))
    for j in range(n-1):
        Patient_PathPlanner.addConstr(v_max[0] * dt * v_max[0] * dt>= (dstates[j,1] * dstates[j,1]) + (dstates[j,0] * dstates[j,0]))
    # Add Obstacle constraints
    for n_obs in range(len(obstacles)):
        for i in range(0, n):
            for k in range(4):
                Patient_PathPlanner.addConstr((states[i, 1] - obstacles[n_obs][0][k]*states[i, 0] - obstacles[n_obs][1][k] )*(obstacles[n_obs][2][1] - obstacles[n_obs][0][k]*obstacles[n_obs][2][0] - obstacles[n_obs][1][k]) <= - (u[n_obs, i, k]-1)*big_M)
        for i in range(0,n):
            Patient_PathPlanner.addConstr(sum(u[n_obs, i, ii] for ii in range(0, 4)) >= 1)

    Patient_PathPlanner.optimize()
    #rospy.loginfo(Patient_PathPlanner.Status)
    path = []
    cost = float('inf')
    if Patient_PathPlanner.Status == 2:
        cost = Patient_PathPlanner.objVal
        we = []
        for j in range(n_rbf):
    	    we.append([weights[j,0].x, weights[j,1].x])
        phi = [state_s[2]]
        for i in range(n-1):
            x1 = states[i,0].x
            y1 = states[i,1].x
            x2 = states[i+1,0].x
            y2 = states[i+1,1].x
            new_phi = math.atan2(y2-y1,x2-x1)
            v = math.sqrt((x2-x1)**2+(y2-y1)**2)/dt
            w = (new_phi-phi[-1])/dt
            phi.append(new_phi)
            path.append([states[i,0].x, states[i,1].x, phi[-1], v, w])
	# rbf_plot(t1, n, n_rbf, dt, epsilon, we, phi_sum)

    return cost, path

def rbf_plot(t1, n, n_rbf, dt, epsilon, weights, phi_sum):
    m =100
    theta = np.linspace(0,n*dt, m)
    time = np.linspace(0,n*dt, n)
    phi = []
    x = np.zeros(n)
    y = np.zeros(n)
    for j in range(n_rbf):
        p = []
    for i in range(n):
	    r = euclidean_norm(time[i], t1[j])
	    x[i] += np.exp(-(r/epsilon)**2) * weights[j][0]/phi_sum[j]
	    y[i] += np.exp(-(r/epsilon)**2) * weights[j][1]/phi_sum[j]
    for i in range(m):
	    r = euclidean_norm(theta[i], t1[j])
	    p.append(np.exp(-(r/epsilon)**2))
    phi.append(p)
    plt.subplot(3, 1, 1)
    for j in range(n_rbf):
        plt.plot(theta, phi[j], 'r')
    plt.title('Radial Basis Functions')

    plt.subplot(3, 1, 2)
    for j in range(n_rbf):
        plt.plot(theta, [phi[j][i]*weights[j][0] for i in range(m)], 'r--')
        plt.plot(time, x, 'b',linewidth = 3)
    plt.title('Weighted Sum of Radial Basis Functions for X trajectory')

    plt.subplot(3, 1, 3)
    for j in range(n_rbf):
        plt.plot(theta, [phi[j][i]*weights[j][1] for i in range(m)], 'r--')
        plt.plot(time, y, 'b',linewidth = 3)
    plt.title('Weighted Sum of Radial Basis Functions for Y trajectory')


def Rbf(t1, d, theta_rbf, n_rbf, epsilon):
    phi_rbf = []
    phi_sum = []
    for i in range(len(theta_rbf)):
        phi_j = []
    for j in range(len(t1)):
	    r = euclidean_norm(theta_rbf[i], t1[j])
	    phi = np.exp(-(r/epsilon)**2)
	    phi_j.append(phi)
	    phi_sum.append(sum(phi_j[k] for k in range(n_rbf)))
	    phi_rbf.append(phi_j)
    return phi_rbf, phi_sum


def euclidean_norm(x1, x2):
    return np.sqrt( ((x1 - x2)**2).sum(axis=0) )

def print_trajectory_status(Status):
    
    if Status == 1:
        print(' The trajectory optimization status code is : ', Status, ' -> Model is loaded, but no solution information is available. ')
    elif Status == 2:
        print(' The trajectory optimization status code is : ', Status, ' -> Model was solved to optimality (subject to tolerances), and an optimal solution is available. ')
    elif Status == 3:
        print(' The trajectory optimization status code is : ', Status, ' -> Model was proven to be infeasible. ')
    elif Status == 4:
        print(' The trajectory optimization status code is : ', Status, ' -> Model was proven to be either infeasible or unbounded.  ')
    elif Status == 5:
        print(' The trajectory optimization status code is : ', Status, ' -> Model was proven to be unbounded ')
    elif Status == 6:
        print(' The trajectory optimization status code is : ', Status, ' -> Optimal objective for model was proven to be worse than the value specified in the Cutoff parameter. No solution information is available. ')
    elif Status == 7:
        print(' The trajectory optimization status code is : ', Status, ' -> Optimization terminated because the total number of simplex iterations performed exceeded the value specified in the IterationLimit parameter, or because the total number of barrier iterations exceeded the value specified in the BarIterLimit parameter. ')
    elif Status == 8:
        print(' The trajectory optimization status code is : ', Status, ' -> Optimization terminated because the total number of branch-and-cut nodes explored exceeded the value specified in the NodeLimit parameter. ')
    elif Status == 9:
        print(' The trajectory optimization status code is : ', Status, ' -> Optimization terminated because the time expended exceeded the value specified in the TimeLimit parameter. ')
    elif Status == 10:
        print(' The trajectory optimization status code is : ', Status, ' -> Optimization terminated because the number of solutions found reached the value specified in the SolutionLimit parameter.')
    elif Status == 11:
        print(' The trajectory optimization status code is : ', Status, ' -> Optimization was terminated by the user.')
    elif Status == 12:
        print(' The trajectory optimization status code is : ', Status, ' -> Optimization was terminated due to unrecoverable numerical difficulties.')
    elif Status == 13:
        print(' The trajectory optimization status code is : ', Status, ' -> Unable to satisfy optimality tolerances; a sub-optimal solution is available. ')
    elif Status == 14:
        print(' The trajectory optimization status code is : ', Status, ' -> An asynchronous optimization call was made, but the associated optimization run is not yet complete.')
    elif Status == 15:
        print(' The trajectory optimization status code is : ', Status, ' -> User specified an objective limit and that limit has been reached.')
    else:
        print('Something wiered is going on! The status that is returned is not defined in Gurobi!!!')
