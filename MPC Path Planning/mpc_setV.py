import numpy as np
import itertools
from functions import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from timeit import default_timer as timer



#################### INPUTS ####################

# prediction/control horizons, time scale
H_p = 2 # number of steps in prediction horizon
H_c = 1 # number of steps in control horizon
dt = .2 # seconds in one time step (length of step is v*dt so smaller dt means smaller steps, "greater resolution", with same velocity)

goal = [1, 1.5] # goal location
goalThresh = .25 # minimum distance to goal to be considered at goal

# intitial obstacles ([x_1, y_1], [x_2, y_2], ... , [x_n, y_n]) - all circular
obstacles = [[-.08, 1], [0.2,1.1], [1,0], [0, -0.5], [2,2], [0.5, 1.8], [0.5, -0.5], [0.5, 0], [0.5, 1.4], [-0.5,1.3], [0.5,1], [1.2,.6], [1.2,.3], [1.5,1], [1,2], [1.5, 2], [1,1], [1.25, 0], [2, 1], [2,3], [2,2.5]]
obstacle_radius = 0.2 # radius of actual obstacle
min_buffer = 0.1 # distance to avoid obstacle by

# robot dimensions
robot_length = 0.2
robot_width = 0.1
robot_dim = [robot_length, robot_width]

# generate control sequences    
v_max = 1 # max velocity
phi_max = 30 # max steering angle in degrees

num_v = 4 # number of potential velocity commands
num_phi = 5 # number of potential steering angle commands, must be odd to allow for 0deg steer

# other control
circles_radius = v_max*dt*0.7


#################### SETUP/TESTING ####################

# initial control commands (v=velocity, phi=steering angle input) and state (x, y, heading)
v = [0]
phi = [0]
x = [0] 
y = [0]
current_heading = 0 

# stop conditions
max_seconds = 30 # max simulation time in seconds
max_k = (max_seconds-dt)/dt # max number of simulation steps
run_once = False # toggle True/False to execute code once or as many times as set (see if statement for stop conditions below)

# miscellaenous
k = 0 # initialize time steps
goalReached=False # initialize if at goal
avoidance_radius = obstacle_radius + min_buffer # min distance to be from center of obstacle, used for calculation/plotting
run_time = 0 # initialize track of how long program has run for as sum of execution loops

# plotting switch
# 0 --> dynamic with future path
# 1 --> step-wise with future path
# 2 --> step-wise, no future path
active_plot = 1




# Check for valid inputs
if not H_c < H_p:
    print("\nERROR - Check input: Control horizon (H_c) must be less than prediction \nhorizion (H_p).\n")
    goalReached = True
if num_phi %2 == 0:
    print("\nERROR - Check input: Number of steering angle commands (num_phi) must be \nan odd number to allow for driving straight.\n")
    goalReached = True

#################### CALCULATIONS ####################

if(goalReached==False):
    v_sequence = gen_sequence_v(num_v, v_max) # sequence of potential velocity commands
    phi_sequence = gen_sequence_phi(num_phi, phi_max) # sequence of potential steering angle commands
    combos = [[a,b] for a in v_sequence for b in phi_sequence]

    print()
    print("=============== Control inputs ===============")
    print("Velocity:    ", v_sequence)
    print("Steer angles:", phi_sequence)
    # print("Combinations:", combos)

while(goalReached==False): # run until goal is reached or while loop broken for other conditions

    start = timer()

    # 1. find distance to goal from current position
    # 2. adjust prediction horizon to be smaller if closer to goal

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
            J_v, J_phi, J_dist, J_obs, J_return, J_fast, J_circles = calc_score(prefix, dt, H_p, x, y, v, v_max, current_heading, k, goal, obstacles, avoidance_radius, circles_radius)
            newScore = J_v + J_phi + J_dist + J_obs + J_return + J_fast + J_circles
            if newScore < score:
                better = prefix.copy()
                better.append(newScore)
                # print(J_v, J_phi, J_dist, J_obs, J_return, J_fast, J_circles)
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

    optimal_sequence = printAllKLength(combos, H_p) # find control sequence with lowest score

    # print()
    # print("-------- Control selection --------")

    # # display step time vs. execution time
    # sim_time = k*dt + dt
    # print("time step =", sim_time, "s")
    # execution_time = timer()-start
    # run_time = run_time + execution_time
    # print("run time =", run_time ,"s", "(execution time =", execution_time,"s )")
    # if execution_time > dt: # execution must be faster than desired time step or else can't apply control in time
    #     print("! EXECUTION TOO SLOW !")

    # print(optimal_sequence)


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

    # plot path so far and read in any new obstacles to avoid
    new_obstacles = plotPath(robot_dim, x, y, dt, current_heading, goal, goalThresh, obstacles, obstacle_radius, avoidance_radius, optimal_plot, active_plot)
    if len(new_obstacles)!= 0:
        obstacles.extend(new_obstacles)

    goalReached = goalCheck(x, y, goal, goalThresh) # check if position within goal threshold

    # stop conditions: check if searching complete due to reaching goal, timeout, or testing
    if(goalReached==True):
        print()
        print("Goal Reached!")
        active_plot = 2
        plotPath(robot_dim, x, y, dt, current_heading, goal, goalThresh, obstacles, obstacle_radius, avoidance_radius, optimal_plot, active_plot)
        break

    if k>max_k or run_once==True:
        print()
        print("Done")
        break


 