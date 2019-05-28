import numpy as np
import itertools
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.interpolate import interp1d

def gen_sequence_phi(numToGenerate, maxInput):
    step = 2*maxInput/(numToGenerate-1)
    sequence = np.zeros(numToGenerate)
    new = -maxInput
    for i in range(numToGenerate):
        sequence[i] = new
        new = new + step
    return sequence 

def gen_sequence_v(numToGenerate, maxInput):
    start = 0
    step = (maxInput - start)/(numToGenerate-2)
    v_sequence = [-step]
    return np.append(v_sequence, np.linspace(start, maxInput, numToGenerate-1))


def calc_score(control, dt, H_p, x, y, v, v_max, current_heading, k, goal, obstacles, avoidance_radius, circles_radius):
    W_v = 1 # generally leave at one if don't mind changes in velocity that much
    W_phi = 4 # increase to tune amount of turning, higher good to prevent going in circles (continuing to turn) but need low enough to have flexible route
    W_dist = 1 # leave as one because sum_dist already relatively large
    W_obs = 1 # leave at 1, cost_obs applies weight
    cost_obs = 1e6 # only gets applied if within obstacle zone
    W_stuck = 1 # leave at 1, cost_return applies weight
    cost_stuck = 1e5 # only gets applied if velocity below certain threshold
    W_fast = 1
    cost_circles = 2
    W_circles = 2

    # velocity change
    sum_v = (control[0][0] - v[-1])**2 # initialize with difference in first applied move from current state
    sum_fast = (control[0][0] - v_max)**2
    for i in range(H_p-1):
        sum_v = sum_v + (control[i+1][0]-control[i][0])**2
        sum_fast = sum_fast + (control[i+1][0] - v_max)**2
    J_v = W_v*sum_v
    J_fast = W_fast*sum_fast

    
    # steer angle change, want to minimize entirely to limit steering change
    sum_phi = 0 # initialize with difference in first applied move from current state
    for i in range(H_p):
        sum_phi = sum_phi + (control[i][1]/360)**2 # divide by 360 to put on similar scale to sum_v
    J_phi = W_phi*sum_phi

    # distance to goal, distance to obstacles, backtrack prevention
    sum_dist = 0
    sum_obs = 0
    sum_stuck = 0  # costly to return to same position, want to keep exploring if stuck
    sum_circles = 0
    x_control = x.copy()
    y_control = y.copy()


    new_heading = current_heading 

    for i in range(len(control)):

        new_v = control[i][0] # new initial future velocity command
        new_heading = calc_heading(new_heading, control[i][1]) # new intial future heading, current heading + next steering command
        x2, y2 = calc_position(x_control[k+i], y_control[k+i], new_v, dt, new_heading)
       
        # print()
        # print(current_heading)
        # print(new_heading)
        # print(new_v)
        # print(x2, y2)
        # print(x_control[k+i], y_control[k+i])
       
        sum_dist = sum_dist + math.sqrt( (x2-goal[0])**2 + (y2-goal[1])**2 )
        #print(obstacles)

        for j in range(len(obstacles)):
            # clearance = math.sqrt( (x_control[k+i]-obstacles[j][0])**2 + (y_control[k+i]-obstacles[j][1])**2 )
            clearance = math.sqrt( (x2-obstacles[j][0])**2 + (y2-obstacles[j][1])**2 )

            if clearance <= avoidance_radius:
                sum_obs = sum_obs + cost_obs

        # keep robot moving with cost for low velocity
        if new_v < 0.1:
            sum_stuck = sum_stuck + cost_stuck

        dist_circles_to_check = 40
        if len(x_control) < dist_circles_to_check:
            dist_circles_to_check = len(x_control)

        for j in range(len(x)-dist_circles_to_check, len(x)):
            dist_circles = math.sqrt( (x2-x_control[j])**2 +(y2-y_control[j])**2 )
            if dist_circles <= circles_radius:
                sum_circles = sum_circles + cost_circles


        # update arrays for prediction horizon steps
        x_control.append(x2)
        y_control.append(y2)


    J_dist = W_dist*sum_dist
    J_obs = W_obs*sum_obs
    J_stuck = W_stuck*sum_stuck
    J_circles = W_circles*sum_circles

    objective = J_v + J_phi + J_dist + J_obs + J_stuck + J_fast + J_circles

    return J_v, J_phi, J_dist, J_obs, J_stuck, J_fast, J_circles


def calc_position(x1, y1, v, dt, phi):

    x2 = x1 + v*dt*math.sin(math.radians(phi))
    y2 = y1 + v*dt*math.cos(math.radians(phi))

    return x2, y2

def calc_heading(current_heading, new_phi):
    return current_heading + new_phi


def goalCheck(x, y, goal, goalThresh):
    distance = math.sqrt( (x[-1]-goal[0])**2 + (y[-1]-goal[1])**2 )
    if distance < goalThresh:
        return True
    else:
        return False

def plotPath(robot_dim, x, y, dt, current_heading, goal, goalThresh, obstacles, obstacle_radius, avoidance_radius, optimal_plot, active_plot):
    
    # ax = plt.gca()
    fig, ax = plt.subplots()

    new_obstacle = []

    def onclick(event):
        if event.xdata != None and event.ydata != None:
            new_obstacle.append([event.xdata, event.ydata])

    cid = fig.canvas.mpl_connect('button_press_event', onclick)


    # plot path and goal
    plt.plot(x, y, marker=".")
    plt.scatter(goal[0], goal[1], color="green")
    goal_circle = plt.Circle((goal[0], goal[1]), goalThresh, alpha=0.2, color='green')
    ax.add_artist(goal_circle)

    # plot robot body
    robot_length = robot_dim[0]
    robot_width = robot_dim[1]
    rect_phi = 90 - current_heading

    x_rear, y_rear = calc_position(x[-1], y[-1], -robot_length/2, 1, current_heading)
    x_front, y_front = calc_position(x[-1], y[-1], robot_length/2, 1, current_heading)
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
    lower = -2
    upper = 4.5

    if active_plot == 0: # go as fast as time steps allow, plot future moves
        plt.scatter(optimal_plot[0][:], optimal_plot[1][:], color="orange", marker="x")
        ax.set_xlim([lower, upper])
        ax.set_ylim([lower, upper])
        ax.autoscale(enable=False, axis='both')
        plt.show(block=False)
        plt.pause(0.01)
        plt.close()
    elif active_plot == 1: # stepwise, plot future moves
        plt.scatter(optimal_plot[0][:], optimal_plot[1][:], color="orange", marker="x")
        ax.set_xlim([lower, upper])
        ax.set_ylim([lower, upper])
        ax.autoscale(enable=False, axis='both')
        plt.show()
    else: # stepwise, only plot current moves
        plt.title("MPC Based Path Planning \n------->  Complete!  <-------")
        ax.set_xlim([lower, upper])
        ax.set_ylim([lower, upper])
        ax.autoscale(enable=False, axis='both')
        plt.show()

    return new_obstacle





