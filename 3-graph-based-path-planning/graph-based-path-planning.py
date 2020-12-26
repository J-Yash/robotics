#!/usr/local/bin/python3

"""
Author: Yashvardhan Jain
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation
import shapely
from shapely.geometry import LineString, Point
from collections import deque
import random

fig = plt.figure()
plt.axis('equal')
ax = fig.add_subplot(111)
ax.set_xlim(-10, 800)
ax.set_ylim(-10, 600)

# Coordinates of start point and goal point
start_xy = [150, 150]
goal_xy = [650, 400]

# Defining start point and goal point
start_point = patches.Circle(start_xy, 10, fill=True, fc='blue')
goal_point = patches.Circle(goal_xy, 10, fill=True, fc='teal')

# Defining the bounding box
bbox1 = ax.hlines(100, 100, 700)
bbox2 = ax.vlines(100, 100, 500)
bbox3 = ax.hlines(500, 100, 700)
bbox4 = ax.vlines(700, 100, 500)

# Coordinates of the obstacles
obs1 = [[190,195], [250, 210], [250, 250], [210, 240]]
obs2 = [[300,190], [340, 220], [350, 280], [310, 270]]
obs3 = [[385,280], [425, 270], [435, 340], [380, 340]]
obs4 = [[470,300], [520, 300], [520, 350], [470, 350]]
obs5 = [[560,343], [623, 333], [590, 408], [546, 428]]

# Defining the obstacles
obstacle1 = patches.Polygon(obs1, closed=True, color='green', fill=True)
obstacle2 = patches.Polygon(obs2, closed=True, color='green', fill=True)
obstacle3 = patches.Polygon(obs3, closed=True, color='green', fill=True)
obstacle4 = patches.Polygon(obs4, closed=True, color='green', fill=True)
obstacle5 = patches.Polygon(obs5, closed=True, color='green', fill=True)

# Define a list of all nodes(vertices of obstacles) (including the start point and goal point)
nodes = [
        start_xy,
        goal_xy,
        obs1[0],
        obs1[1],
        obs1[2],
        obs1[3],
        obs2[0],
        obs2[1],
        obs2[2],
        obs2[3],
        obs3[0],
        obs3[1],
        obs3[2],
        obs3[3],
        obs4[0],
        obs4[1],
        obs4[2],
        obs4[3],
        obs5[0],
        obs5[1],
        obs5[2],
        obs5[3],

]

# Defining obstacle edges

obs_edges = [
    [obs1[0], obs1[1]],
    [obs1[1], obs1[2]],
    [obs1[2], obs1[3]],
    [obs1[3], obs1[0]],

    [obs2[0], obs2[1]],
    [obs2[1], obs2[2]],
    [obs2[2], obs2[3]],
    [obs2[3], obs2[0]],

    [obs3[0], obs3[1]],
    [obs3[1], obs3[2]],
    [obs3[2], obs3[3]],
    [obs3[3], obs3[0]],

    [obs4[0], obs4[1]],
    [obs4[1], obs4[2]],
    [obs4[2], obs4[3]],
    [obs4[3], obs4[0]],

    [obs5[0], obs5[1]],
    [obs5[1], obs5[2]],
    [obs5[2], obs5[3]],
    [obs5[3], obs5[0]],
]

# Defining all segments and then removing intersecting segments to create a visibility graph
segments = []

for node1 in nodes:
    for node2 in nodes:
        if node1 != node2:
            segment = [node1, node2]
            segment_t = [node2, node1]
            if segment not in segments and segment_t not in segments:
                segments.append(segment)

def check_intersect(segment1, segment2):
    line1 = LineString(segment1)
    line2 = LineString(segment2)

    int_pt = line1.intersection(line2)
    if int_pt.is_empty:
        return None
    else:
        if type(int_pt) == Point:
            return [int_pt.x, int_pt.y]
        else:
            return None

vis_segments = []
for segment1 in segments:
    flag = False
    for segment2 in obs_edges:
        if segment1 != segment2:
            val = check_intersect(segment1, segment2)
            if val is not None:
                if val not in segment1 and val not in segment2:
                    flag = True
    if flag is False:
        vis_segments.append(segment1)

obs_all = [obs1,obs2,obs3,obs4,obs5]

# Remove diagonals of polygons
for segment in vis_segments:
    for obs in obs_all:
        if segment[0] in obs and segment[1] in obs:
            if segment not in obs_edges:
                vis_segments.remove(segment)

def plot_segments(all_segments, lw, color):
    for segment in all_segments:
        seg_x, seg_y = zip(*segment)
        _, = ax.plot(seg_x, seg_y, lw=lw, color=color)

plot_segments(vis_segments, 2, None)

# Create the adjacency list for all the nodes in the visibility graph

nodes_adj = {}
for node in nodes:
    for segment in vis_segments:
        if node == segment[0]:
            node_tup = tuple(node)
            if node_tup not in nodes_adj:
                nodes_adj[node_tup] = [segment[1]]
                
            else:
                nodes_adj[node_tup].append(segment[1])
                
        elif node == segment[1]:
            node_tup = tuple(node)
            if node_tup not in nodes_adj:
                nodes_adj[node_tup] = [segment[0]]
                
            else:
                nodes_adj[node_tup].append(segment[0])

# Helper function for BREADTH FIRST SEARCH to get possible moves while searching
def possible_moves(nodes_adj, curr_move, visited_loc):
    moves = nodes_adj[tuple(curr_move)]
    valid_moves = [move for move in moves if move not in visited_loc]
    return valid_moves

# Function to find a path from start point to goal point using BREADTH FIRST SEARCH
def find_path(nodes_adj):
    fringe = deque()
    fringe.append((start_xy, [start_xy]))
    visited_loc = []

    while fringe:
        (curr_move, path) = fringe.popleft()
        visited_loc.append(curr_move)
        all_moves = possible_moves(nodes_adj, curr_move, visited_loc)
        for i in range(len(all_moves)):
            if all_moves[i] == goal_xy:
                return path + [all_moves[i]]
            else:
                fringe.append((all_moves[i], path + [all_moves[i]]))


robot_path = find_path(nodes_adj)

robot_path_x, robot_path_y = zip(*robot_path)
linepath, = ax.plot(robot_path_x, robot_path_y, lw=3, color='black')  

robot = patches.Circle(start_xy, 7, fill=True, fc='red')

# Get intermediate points on the path for animation.

def intermediates(p1, p2, nb_points):
    x_spacing = (p2[0] - p1[0]) / (nb_points + 1)
    y_spacing = (p2[1] - p1[1]) / (nb_points + 1)

    return [[p1[0] + i * x_spacing, p1[1] +  i * y_spacing] 
            for i in range(1, nb_points+1)]

rob_x = []
rob_y = []

nb = 360//(len(robot_path)-1)

for i in range(len(robot_path)-1):
    p1 = [robot_path[i][0], robot_path[i][1]]
    p2 = [robot_path[i+1][0], robot_path[i+1][1]]
    p = intermediates(p1, p2, nb)
    x,y = zip(*p)
    rob_x = rob_x + list(x)
    rob_y = rob_y + list(y)

def init():
    ax.add_patch(start_point)
    ax.add_patch(goal_point)
    ax.add_patch(obstacle1)
    ax.add_patch(obstacle2)
    ax.add_patch(obstacle3)
    ax.add_patch(obstacle4)
    ax.add_patch(obstacle5)
    ax.add_patch(robot)
    return robot,

def animate(i):
    robot.center = rob_x[i], rob_y[i]
    return robot,
    
anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=360,
                               interval=100,
                               blit=True,
                               repeat=False)
plt.show()