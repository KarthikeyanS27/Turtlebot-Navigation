import numpy as np
import itertools
import math

def gen_sequence(numToGenerate, maxInput):
    step = 2*maxInput/(numToGenerate-1)
    sequence = np.zeros(numToGenerate)
    new = -maxInput
    for i in range(numToGenerate):
        sequence[i] = new
        new = new + step
    return sequence 


def calc_score(control, dt, H_c, x, y, k, goal, obstacles, min_buffer):
    W_v = 1
    W_phi = 1
    W_dist = 1000
    W_obs = 1

    # velocity change
    sum_v = 0
    for i in range(H_c-1):
        sum_v = sum_v + (control[i+1][0]-control[i][0])**2
    J_v = W_v*sum_v
    
    # steer angle change
    sum_phi = 0
    for i in range(H_c):
        sum_phi = sum_phi + (control[i+1][1]-control[i][1])**2
    J_phi = W_phi*sum_phi

    # distance to goal, distance to obstacles
    sum_dist = 0
    sum_obs = 0
    x_control = x.copy()
    y_control = y.copy()

    for i in range(len(control)):

        x2, y2 = calc_position(x_control[k+i], y_control[k+i], control[i][0], dt, control[i][1])
        sum_dist = sum_dist + math.sqrt( (x_control[k+i]-goal[0])**2 + (y_control[k+i]-goal[1])**2 )
        
        for j in range(len(obstacles)):
            clearance = math.sqrt( (x_control[k+i]-obstacles[j][0])**2 + (y_control[k+i]-obstacles[j][1])**2 )
            if clearance <= min_buffer:
                sum_obs = sum_obs + 1e6
            else:
                sum_obs = sum_obs + 0 

        x_control.append(x2)
        y_control.append(y2)

    J_dist = W_dist*sum_dist
    J_obs = W_obs*sum_obs

    #print(J_v, J_phi, J_dist, J_obs)

    return J_v + J_phi + J_dist + J_obs


def calc_position(x1, y1, v, dt, phi):

    x2 = x1 + v*dt*math.sin(math.radians(phi))
    y2 = y1 + v*dt*math.cos(math.radians(phi))

    return x2, y2


def goalCheck(x, y, goal, goalThresh):
    distance = math.sqrt( (x[-1]-goal[0])**2 + (y[-1]-goal[1])**2 )
    if distance < goalThresh:
        return True
    else:
        return False

def csvtoarray(csvfile = 'floorplan.csv'):
    datafile = open(csvfile, 'r')
    datareader = csv.reader(datafile, delimiter=',')
    data = []
    for row in datareader:
        data.append(row)
    # print(data)
    # print(data[0][1])
    # data[y][x], origin at top left corner
    return(data)



