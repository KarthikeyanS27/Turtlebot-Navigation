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

def gen_sequence_v(numToGenerate, maxInput):
    start = 0
    return np.linspace(start, maxInput, numToGenerate) 


def calc_score(control, dt, H_p, x, y, v, current_heading, k, goal, obstacles, avoidance_radius, min_return):
    W_v = 1
    W_phi = 3
    W_dist = 1 # leave as one because sum_dist already relatively large
    W_obs = 1
    cost_obs = 1e6 # only gets applied if within obstacle zone
    W_return = 1
    cost_return = 1e5 # only gets applied if velocity below certain threshold

    # velocity change
    sum_v = (control[0][0] - v[-1])**2 # initialize with difference in first applied move from current state
    for i in range(H_p-1):
        sum_v = sum_v + (control[i+1][0]-control[i][0])**2
    J_v = W_v*sum_v
    
    # steer angle change, want to minimize entirely to limit steering change
    sum_phi = 0 # initialize with difference in first applied move from current state
    for i in range(H_p):
        sum_phi = sum_phi + (control[i][1]/360)**2 # divide by 360 to put on similar scale to sum_v
    J_phi = W_phi*sum_phi

    # distance to goal, distance to obstacles, backtrack prevention
    sum_dist = 0
    sum_obs = 0
    sum_return = 0  # costly to return to same position, want to keep exploring if stuck
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

        for j in range(len(obstacles)):
            # clearance = math.sqrt( (x_control[k+i]-obstacles[j][0])**2 + (y_control[k+i]-obstacles[j][1])**2 )
            clearance = math.sqrt( (x2-obstacles[j][0])**2 + (y2-obstacles[j][1])**2 )

            if clearance <= avoidance_radius:
                sum_obs = sum_obs + cost_obs

        # keep robot moving with cost for low velocity
        if new_v < 0.1:
            sum_return = sum_return + cost_return

        # update arrays for prediction horizon steps
        x_control.append(x2)
        y_control.append(y2)


    J_dist = W_dist*sum_dist
    J_obs = W_obs*sum_obs
    J_return = W_return*sum_return

    objective = J_v + J_phi + J_dist + J_obs + J_return

    J_v, J_phi, J_dist, J_obs, J_return

    return J_v, J_phi, J_dist, J_obs, J_return


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
    
    ax = plt.gca()

    # plot path and goal
    plt.scatter(x, y)
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

    if active_plot == 0:
        # plot future moves
        plt.scatter(optimal_plot[0][:], optimal_plot[1][:], color="orange", marker="x")

        plt.show(block=False)
        plt.pause(dt)
        plt.close()
    elif active_plot == 1:
        # plot future moves
        plt.scatter(optimal_plot[0][:], optimal_plot[1][:], color="orange", marker="x")
        plt.show()
    else:
        plt.show()




