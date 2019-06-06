# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Michael Abir (abir2@illinois.edu) on 08/28/2018
# Modified by Rahul Kunji (rahulsk2@illinois.edu) on 01/16/2019

"""
This is the main entry point for MP1. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""

# from heapq import heappush, heappop
import sys

# Search should return the path and the number of states explored.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# Number of states explored should be a number.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,greedy,astar)

import queue

def search(maze, searchMethod):
    return {
        "bfs": bfs,
        "dfs": dfs,
        "greedy": greedy,
        "astar": astar,
    }.get(searchMethod)(maze)


def bfs(maze):
    # TODO: Write your code here
    # return path, num_states_explored
    frontier = []
    visited = {}
    parent = {}
    path = []
    # init frontier, curr, etc.
    frontier.append(maze.getStart())
    curr = maze.getStart()
    num_states_explored = 0
    objective_found = False
    objective = None
    parent[curr] = None
    visited[curr] = True
    while frontier != []:
        curr = frontier.pop(0)
        neighbors = maze.getNeighbors(curr[0], curr[1])
        for i in neighbors:
            if i not in visited:
                visited[i] = True
                parent[i] = curr
                frontier.append(i)
                num_states_explored = num_states_explored + 1
                if maze.isObjective( i[0], i[1]):
                    objective = i
                    objective_found = True
                    break
    i = objective
    while parent[i] != None:
        path.insert(0,i)
        i = parent[i]
        print("Parent: {}".format(parent[i]), end='\r')
    path.insert(0,maze.getStart())
    return path, num_states_explored


def dfs(maze):
    frontier = []
    visited = {}
    parent = {}
    path = []
    # init frontier, curr, etc.
    frontier.append(maze.getStart())
    curr = maze.getStart()
    num_states_explored = 0
    objective_found = False
    objective = None
    parent[curr] = None
    visited[curr] = True
    while frontier != []:
        curr = frontier.pop(-1)
        neighbors = maze.getNeighbors(curr[0], curr[1])
        for i in neighbors:
            if i not in visited:
                visited[i] = True
                parent[i] = curr
                frontier.append(i)
                num_states_explored = num_states_explored + 1
                if maze.isObjective( i[0], i[1]):
                    objective = i
                    objective_found = True
                    break
    i = objective
    while parent[i] != None:
        path.insert(0,i)
        i = parent[i]
        print("Parent: {}".format(parent[i]), end='\r')
    path.insert(0,maze.getStart())
    return path, num_states_explored


def gheuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def greedy_select(frontier, objective):
    min_h = sys.maxsize
    min_elem = None
    for i, _ in enumerate(frontier):
        h = gheuristic(frontier[i], objective)
        if h < min_h:
            min_h = h
            min_elem = i

    return frontier.pop(min_elem), frontier

def greedy(maze):
    # return path, num_states_explored
    frontier = []
    visited = {}
    parent = {}
    path = []
    objective = maze.getObjectives()[0]
    # init frontier, curr, etc.
    frontier.append(maze.getStart())
    curr = maze.getStart()
    num_states_explored = 0
    objective_found = False
    parent[curr] = None
    visited[curr] = True
    while frontier != []:
        # TODO write solution
        curr, frontier = greedy_select(frontier, objective)
        num_states_explored += 1
        neighbors = maze.getNeighbors(curr[0], curr[1])
        for i in neighbors:
            if i not in visited:
                visited[i] = True
                parent[i] = curr
                frontier.append(i)
                if maze.isObjective( i[0], i[1]):
                    objective = i
                    objective_found = True
                    break
    i = objective
    while parent[i] != None:
        path.insert(0,i)
        i = parent[i]
        # print("Parent: {}".format(parent[i]), end='\r')
    path.insert(0,maze.getStart())
    return path, num_states_explored



def heuristic(x, y):
    return 0

def heuristic_mul(x, objectives):
    min_h = sys.maxsize
    for j in objectives:
        h = heuristic(x, j)
        if h < min_h:
            min_h = h
    return min_h



def heuristic_augmented(x, objectives):
    # find full path with heuristics.
    objectives_ = objectives.copy()
    final_heuristic = 0
    start = x
    while objectives_ != []:
        # get the closest objective to current point.
        closest = closest_objective(x, objectives_)
        # Assume we went there, append distance to estimate.
        final_heuristic += heuristic_mul(start, objectives)
        # remove from remaining objectives
        objectives_.remove(closest)
        # Find the rest from this new point.
        start = closest
        pass
    return final_heuristic

def closest_objective(x, objectives):
    min_h = sys.maxsize
    min_obj = None
    for j in objectives:
        h = heuristic(x, j)
        if h < min_h:
            min_obj = j
            min_h = h
    return min_obj

def astar_select(frontier, objectives, cost):
    min_score = sys.maxsize
    min_index = None
    for i, elem in enumerate(frontier):
        # score = cost[elem] + heuristic_mul(elem, objectives)
        score = cost[elem] + heuristic_augmented(elem, objectives)
        if score < min_score:
            min_score = score
            min_index = i
            pass
    assert min_index != None
    return frontier.pop(min_index), frontier

def generate_path(curr, parent):
    path = []
    while parent[curr] != None:
        path.insert(0, curr)
        curr = parent[curr]
    return path

def traverse_astar(maze, startPoint, objectives):
    # Initialize Everything.
    frontier = []
    cost = {}
    # startPoint = maze.getStart()
    parent = {}
    path = []
    # objectives = maze.getObjectives()
    num_states = 0
    parent[startPoint] = None
    cost[startPoint] = 0
    frontier.append(startPoint)
    while frontier != []:
        # Smart select from frontier.
        curr, frontier = astar_select(frontier, objectives, cost)
        num_states += 1
        # handle reaching objective.
        if curr in objectives:
            # remove from objectives, generate path, reset everything.
            objectives.remove(curr)
            path += generate_path(curr, parent)
            parent = {}
            frontier = []
            cost = {}
            parent[curr] = None
            cost[curr] = 0
        # Handle complete case.
        if objectives == []:
            path.insert(0,maze.getStart())
            return path, num_states
        # Expand node from frontier.
        new_frontier = maze.getNeighbors(curr[0], curr[1])
        for i in new_frontier:
            if i not in cost:
                cost[i] = cost[curr] + 1
                frontier.append(i)
                parent[i] = curr
            else:
                new_cost = cost[curr] + 1
                if new_cost < cost[i]:
                    cost[i] = new_cost
                    parent[i] = curr
                    pass
                pass
            pass
    path.insert(0,maze.getStart())
    return path, num_states
def astar(maze):
    path, n_states = traverse_astar(maze, maze.getStart(), maze.getObjectives())
    return path, n_states