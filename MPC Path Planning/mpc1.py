import numpy as np
import itertools
from functions import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Inputs
H_p = 3 # number of steps in prediction horizon
H_c = 1 # number of steps in control horizon
dt = .5 # seconds in one time step

v_max = 1 # max velocity
phi_max = 60 # max steering angle in degrees

	# Generate control sequences	
num_v = 6 # number of potential velocity commands
num_phi = 4 # number of potential steering angle commands

# positional coordinates
x = [0] 
y = [0]
v = [0]
phi = [0]
goal = [5, 5]
goalThresh = .1
goalReached=False

k = 0 # initialize time steps

obstacles = [[0, 1], [2, 1], [2,3], [2,2], [2,2.5]]
min_buffer = 0.3

v_sequence = gen_sequence(num_v, v_max) # sequence of potential velocity commands
phi_sequence = gen_sequence(num_phi, phi_max) # sequence of potential steering angle commands
combos = [[a,b] for a in v_sequence for b in phi_sequence]
print(combos)

while(goalReached==False):

    # The method that prints all  
    # possible strings of length k. 
    # It is mainly a wrapper over  
    # recursive function printAllKLengthRec()
    # https://www.geeksforgeeks.org/print-all-combinations-of-given-length/ 
    def printAllKLength(set, k):

        initialScore = 1e100
        optimal = [""]*k
        optimal.append(initialScore) 
      
        n = len(set)  
        prefix = []
        return printAllKLengthRec(set, prefix, n, k, optimal)
      
    # The main recursive method 
    # to print all possible  
    # strings of length k 
    def printAllKLengthRec(set, prefix, n, l_seq, optimal): 
        score = optimal[-1]
        best = [1e100]

        # Base case: k is 0, 
        # print prefix 
        if (l_seq == 0):
            newScore = calc_score(prefix, dt, H_c, x, y, k, goal, obstacles, min_buffer)
            if newScore < score:
                better = prefix.copy()
                better.append(newScore)
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

    # Apply first steps of sequence
    for i in range(H_c):
        v.append(optimal_sequence[i][0])
        phi.append(optimal_sequence[i][1])
        x2, y2 = calc_position(x[k], y[k], v[-1], dt, phi[-1])
        x.append(x2)
        y.append(y2)  

        k+=1

    plt.scatter(x, y)
    plt.scatter(goal[0], goal[1], c="green")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("MPC Based Path Planning")

    for center in obstacles:
        circle = plt.Circle((center[0], center[1]), min_buffer, color='r')
        ax = plt.gca()
        ax.add_artist(circle)

    ax.axis("equal")

    plt.show()

    goalReached = goalCheck(x, y, goal, goalThresh)

    if(goalReached==True):
        print("Goal Reached!")
        break


 