import numpy as np
import itertools
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def gen_sequence_phi(numToGenerate, maxInput):
    step = 2*maxInput/(numToGenerate-1)
    sequence = np.zeros(numToGenerate)
    new = -maxInput
    for i in range(numToGenerate):
        sequence[i] = new
        new = new + step
    return sequence 

def gen_sequence_a(numToGenerate, maxThrottle, maxBrake):
    a_sequence = [maxBrake]
    return np.append(a_sequence, np.linspace(0, maxThrottle, numToGenerate-1))

def gen_v_thresh(numToGenerate, minThresh, maxThresh):
    return np.linspace(minThresh, maxThresh, numToGenerate)



def calc_score(control, dt, H_p, x, y, a, v_max, v_min, current_v, current_heading, k, goal, obstacles, avoidance_radius, previousLoc_radius, robot_memory):
    W_a = 1 # generally leave at one if don't mind changes in velocity that much
    W_phi = 10 # increase to tune amount of turning, higher good to prevent going in circles (continuing to turn) but need low enough to have flexible route
    W_dist = 50 # can be somewhat low because sum_dist already relatively large
    W_obs = 1 # leave at 1, cost_obs applies weight
    cost_obs = 1e10 # only gets applied if within obstacle zone
    W_vmin = 1 # leave at 1, cost_vmin applies weight
    cost_vmin = 1e3 # only gets applied if velocity below certain threshold
    W_reverse = 1
    cost_reverse = 1e8
    W_fast = 30
    cost_previousLoc = 1e3
    W_previousLoc = 1
    W_vmax = 1
    cost_vmax = 1e6

    # velocity change
    sum_fast = 0
    for i in range(H_p):
        new_v = calc_v(current_v, control[i][0], dt)
        sum_fast = sum_fast + (new_v - v_max)**2
    J_fast = W_fast*sum_fast

    # acceleration
    sum_a = 0 # initialize with difference in first applied move from current state
    for i in range(H_p):
        sum_a = sum_a + (control[i][0])**2 # sum all acceleration inputs
    J_a = W_a*sum_a
    
    # steer angle change, want to minimize entirely to limit steering change
    sum_phi = 0 # initialize with difference in first applied move from current state
    for i in range(H_p):
        sum_phi = sum_phi + (control[i][1]/360)**2 # divide by 360 to put on sum on similar scale as other weights
    J_phi = W_phi*sum_phi

    # distance to goal, distance to obstacles, backtrack prevention
    sum_dist = 0
    sum_obs = 0
    sum_vmin = 0  # costly to return to same position, want to keep exploring if speed below vmin (aka stuck)
    sum_reverse = 0
    sum_previousLoc = 0
    sum_vmax = 0
    x_control = x.copy()
    y_control = y.copy()


    new_heading = current_heading 
    new_v = current_v

    for i in range(len(control)):

        new_heading = calc_heading(new_heading, control[i][1]) # new intial future heading, current heading + next steering command
        x2, y2 = calc_position(x_control[k+i], y_control[k+i], new_v, control[i][0], dt, new_heading)
        new_v = calc_v(new_v, control[i][0], dt) # new initial future velocity 
       
        sum_dist = sum_dist + math.sqrt( (x2-goal[0])**2 + (y2-goal[1])**2 )

        for j in range(len(obstacles)):
            clearance = math.sqrt( (x2-obstacles[j][0])**2 + (y2-obstacles[j][1])**2 )

            if clearance <= avoidance_radius:
                sum_obs = sum_obs + cost_obs

        # keep robot moving within desired velocity range
        if abs(new_v) < v_min:
            sum_vmin = sum_vmin + cost_vmin
        if new_v > v_max:
            sum_vmax = sum_vmax + cost_vmax
        if new_v < 0:
            sum_reverse = sum_reverse + cost_reverse

        memory_length = robot_memory
        path_length = len(x_control)    
        if path_length < robot_memory:
            memory_length = path_length

        for j in range(path_length-memory_length, path_length-1):
            dist_previousLoc = math.sqrt( (x2-x_control[j])**2 +(y2-y_control[j])**2 )
            if dist_previousLoc <= previousLoc_radius:
                sum_previousLoc = sum_previousLoc + cost_previousLoc


        # update arrays for prediction horizon steps
        x_control.append(x2)
        y_control.append(y2)


    J_dist = W_dist*sum_dist
    J_obs = W_obs*sum_obs
    J_vmin = W_vmin*sum_vmin
    J_previousLoc = W_previousLoc*sum_previousLoc
    J_vmax = W_vmax*sum_vmax
    J_reverse = W_reverse*sum_reverse

    objective = J_a + J_phi + J_dist + J_obs + J_vmin + J_vmax + J_reverse + J_fast + J_previousLoc 

    return J_a, J_phi, J_dist, J_obs, J_vmin, J_vmax, J_reverse, J_fast, J_previousLoc


def calc_position(x1, y1, v, a, dt, phi):

    x2 = x1 + v*dt*math.sin(math.radians(phi)) + 0.5*a*(dt**2)*math.sin(math.radians(phi))
    y2 = y1 + v*dt*math.cos(math.radians(phi)) + 0.5*a*(dt**2)*math.cos(math.radians(phi))

    return x2, y2

def calc_vector_location(x1, y1, v, dt, phi):

    x2 = x1 + v*dt*math.sin(math.radians(phi))
    y2 = y1 + v*dt*math.cos(math.radians(phi)) 

    return x2, y2

def calc_heading(current_heading, new_phi):
    return current_heading + new_phi

def calc_v(current_v, new_a, dt):
    return current_v + new_a*dt


def goalCheck(x, y, goal, goalThresh):
    distance = math.sqrt( (x[-1]-goal[0])**2 + (y[-1]-goal[1])**2 )
    if distance < goalThresh:
        return True
    else:
        return False

def plotPath(robot_dim, robot_memory, x, y, dt, current_heading, goal, goalThresh, obstacles, obstacle_radius, avoidance_radius, optimal_plot, active_plot):

    fig, ax = plt.subplots()

    new_obstacle = []

    def onclick(event):
        if event.xdata != None and event.ydata != None:
            new_obstacle.append([event.xdata, event.ydata])

    cid = fig.canvas.mpl_connect('button_press_event', onclick)

    # plot path and goal
    path_length = len(x)    
    if path_length < robot_memory:
        robot_memory = path_length

    plt.plot(x[path_length-robot_memory:], y[path_length-robot_memory:], marker=".")
    plt.scatter(goal[0], goal[1], color="green")
    goal_circle = plt.Circle((goal[0], goal[1]), goalThresh, alpha=0.2, color='green')
    ax.add_artist(goal_circle)

    # plot robot body
    robot_length = robot_dim[0]
    robot_width = robot_dim[1]
    rect_phi = 90 - current_heading

    x_rear, y_rear = calc_vector_location(x[-1], y[-1], -robot_length/2, 1, current_heading)
    x_front, y_front = calc_vector_location(x[-1], y[-1], robot_length/2, 1, current_heading)
    rect_x = x_rear+(robot_width/2)*math.sin(math.radians(rect_phi))
    rect_y = y_rear-(robot_width/2)*math.cos(math.radians(rect_phi))

    plt.scatter(x_front, y_front, color="blue", marker="*")
    rect = patches.Rectangle((rect_x, rect_y), robot_length, robot_width, rect_phi, linewidth=1, edgecolor='blue',facecolor='none')
    ax.add_patch(rect)


    # plot obstacles
    for center in obstacles:
        obstacle_circle = plt.Circle((center[0], center[1]), obstacle_radius, color='red')
        avoidance_circle = plt.Circle((center[0], center[1]), avoidance_radius, alpha=0.2, color='red')
        ax = plt.gca()
        ax.add_artist(obstacle_circle)
        ax.add_artist(avoidance_circle)

    # format plot
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("MPC Based Path Planning")
    ax.axis("equal")
    lower = -1
    upper = 3.5

    if active_plot == 0: # go as fast as time steps allow, plot future moves
        plt.scatter(optimal_plot[0][:], optimal_plot[1][:], color="orange", marker="x")
        ax.set_xlim([lower, upper])
        ax.set_ylim([lower, upper])
        ax.autoscale(enable=False, axis='both')
        plt.show(block=False)
        plt.pause(dt)
        plt.close()
    elif active_plot == 1: # stepwise, plot future moves
        plt.scatter(optimal_plot[0][:], optimal_plot[1][:], color="orange", marker="x")
        ax.set_xlim([lower, upper])
        ax.set_ylim([lower, upper])
        ax.autoscale(enable=True, axis='both')
        plt.show()
    else: # stepwise, only plot current moves
        plt.title("MPC Based Path Planning \n------->  Complete!  <-------")
        ax.set_xlim([lower, upper])
        ax.set_ylim([lower, upper])
        ax.autoscale(enable=True, axis='both')
        plt.show()

    return new_obstacle





