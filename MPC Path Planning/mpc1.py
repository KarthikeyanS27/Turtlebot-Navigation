import numpy as np
import itertools
from functions import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Inputs
H_p = 4 # number of steps in prediction horizon
H_c = 1 # number of steps in control horizon
dt = .2 # seconds in one time step

v_max = 1 # max velocity
phi_max = 45 # max steering angle in degrees

	# Generate control sequences	
num_v = 2 # number of potential velocity commands
num_phi = 5 # number of potential steering angle commands, must be odd to allow for 0deg steer

# positional coordinates
x = [0] 
y = [0]
v = [0]
phi = [0]
goal = [1.5, 2.5]
goalThresh = .25
goalReached=False

current_heading = 0

k = 0 # initialize time steps

obstacles = [[0, 1], [0.5, 2], [0.5,1], [1.2,.6], [1.2,.3], [1.5,1], [1,2], [.5, 2], [1.5, 2], [1,1], [1.25, 0], [2, 1], [2,3], [2,2], [2,2.5]]
obstacle_radius = 0.2
min_buffer = 0.1
avoidance_radius = obstacle_radius + min_buffer # min distance to be from obstacles

min_return = 0.1 # min distance for next step to move from previous two steps, resists staying still or going backwards too willingly

robot_length = 0.2
robot_width = 0.1
robot_dim = [robot_length, robot_width]

# Plotting switch
# 0 --> dynamic with future path
# 1 --> step-wise with future path
# 2 --> step-wise, no future path
active_plot = 0

run_once = False

###########################################

# Check for valid inputs

if not H_c < H_p:
    print("ERROR - Check input: Control horizon must be less than prediction horizion.")
    goalReached = True

v_sequence = gen_sequence_v(num_v, v_max) # sequence of potential velocity commands
phi_sequence = gen_sequence_phi(num_phi, phi_max) # sequence of potential steering angle commands
combos = [[a,b] for a in v_sequence for b in phi_sequence]

print()
print("=============== Control inputs ===============")
print("Velocity:    ", v_sequence)
print("Steer angles:", phi_sequence)
print("Combinations:", combos)

while(goalReached==False):

    # The method that prints all  
    # possible strings of length k. 
    # It is mainly a wrapper over  
    # recursive function printAllKLengthRec()
    # https://www.geeksforgeeks.org/print-all-combinations-of-given-length/ 
    def printAllKLength(set, l_seq):

        initialScore = 1e100
        optimal = [""]*k
        optimal.append(initialScore) 
      
        n = len(set)  
        prefix = []
        return printAllKLengthRec(set, prefix, n, l_seq, optimal)
      
    # The main recursive method 
    # to print all possible  
    # strings of length k 
    def printAllKLengthRec(set, prefix, n, l_seq, optimal): 
        score = optimal[-1]
        best = [1e100]

        # Base case: k is 0, 
        # print prefix 
        if (l_seq == 0):
            J_v, J_phi, J_dist, J_obs, J_return = calc_score(prefix, dt, H_p, x, y, v, current_heading, k, goal, obstacles, avoidance_radius, min_return)
            newScore = J_v + J_phi + J_dist + J_obs + J_return
            if newScore < score:
                better = prefix.copy()
                better.append(newScore)
                # print(J_v, J_phi, J_dist, J_obs, J_return)
                # print(better)
                return better
        
        # One by one add all characters  
        # from set and recursively  
        # call for k equals to k-1 
        for i in range(n): 

            # Next character of input added 
            newPrefix = prefix.copy()
            newPrefix.append(set[i])
              
            # k is decreased, because  
            # we have added a new character 
            new = printAllKLengthRec(set, newPrefix, n, l_seq - 1, optimal)
            if new[-1] < best[-1]:
                best = new

        return best

    optimal_sequence = printAllKLength(combos, H_p)
    print("-------- Control selection --------")
    print("time =", k*dt, "s")
    print(optimal_sequence)


    # plot future path
    optimal_sequence = optimal_sequence[:-1]
    x_optimal = []
    y_optimal = []
    x_curr = x[-1]
    y_curr = y[-1]
    new_heading = current_heading
    for i in range(len(optimal_sequence)):
        new_v = optimal_sequence[i][0] # new initial future velocity command
        new_heading = calc_heading(new_heading, optimal_sequence[i][1]) # new intial future heading, current heading + next steering command
        x2, y2 = calc_position(x_curr, y_curr, new_v, dt, new_heading)
        x_optimal.append(x2)
        y_optimal.append(y2)
        x_curr = x_optimal[-1]
        y_curr = y_optimal[-1]

    optimal_plot = [x_optimal, y_optimal]


    # Apply first steps of sequence
    for i in range(H_c):

        v.append(optimal_sequence[i][0])
        phi.append(optimal_sequence[i][1])
        new_v = v[-1]
        new_phi = phi[-1]
        current_heading = calc_heading(current_heading, new_phi) # new heading angle is current heading + latest steering input
        x2, y2 = calc_position(x[k], y[k], new_v, dt, current_heading)
        x.append(x2)
        y.append(y2)

        k+=1

    plotPath(robot_dim, x, y, dt, current_heading, goal, goalThresh, obstacles, obstacle_radius, avoidance_radius, optimal_plot, active_plot)


    goalReached = goalCheck(x, y, goal, goalThresh)

    max_seconds = 15
    max_k = max_seconds/dt

    if(goalReached==True or k>max_k or run_once==True):
        print()
        print("Goal Reached!")
        active_plot = 2
        plotPath(robot_dim, x, y, dt, current_heading, goal, goalThresh, obstacles, obstacle_radius, avoidance_radius, optimal_plot, active_plot)
        break


 