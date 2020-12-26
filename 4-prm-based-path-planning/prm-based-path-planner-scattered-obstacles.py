#!/usr/local/bin/python3

"""
Author: Yashvardhan Jain

PRM based robot planner for case 1 (scattered obstacles). 

Randomly samples configs(nodes) and creates a path.
Uses Breadth First Search for path finding.
In some cases, due to random sampling, the program may result in an error since our search method does not return any path.
This happens in cases when the random samples are generated so that no path can be formed through the tunnel.
In such cases, simply run the file again so that new samples can be generated which would lead to a path.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation
import shapely
from shapely.geometry import LineString, Point
from shapely.geometry.polygon import Polygon
from collections import deque
import random
import random
import math

fig = plt.figure()
plt.axis('equal')
ax = fig.add_subplot(111)
ax.set_xlim(0, 1100)
ax.set_ylim(0, 1100)

print("Please wait. This might take a few minutes...\nIn case you get a TypeError in findpath(), no path could be found. \nSimply rerun the file for new sampling of configurations.")

# Coordinates of start point and goal point
start_xy = [150, 150]
goal_xy = [950, 950]

# Defining start point and goal point
start_point = patches.Circle(start_xy, 10, fill=True, fc='blue')
goal_point = patches.Circle(goal_xy, 10, fill=True, fc='teal')

# Defining the bounding box
bbox1 = ax.hlines(100, 100, 1000)
bbox2 = ax.vlines(100, 100, 1000)
bbox3 = ax.hlines(1000, 100, 1000)
bbox4 = ax.vlines(1000, 100, 1000)

# Coordinates of the obstacles
obs1 = [[150, 240], [260, 240], [260, 350], [150, 350]]
obs2 = [[400, 250], [500, 250], [500, 360], [400, 360]]
obs3 = [[480, 420], [570, 420], [570, 610], [480, 610]]
obs4 = [[160, 600], [300, 600], [300, 710], [160, 710]]
obs5 = [[600, 270], [770, 270], [770, 380], [600, 380]]
obs6 = [[650, 620], [770, 620], [770, 700], [650, 700]]
obs7 = [[770, 150], [860, 150], [860, 800], [770, 800]]

# Obstacles for workspace (Adding padding equal to diagonal of square(15x15) robot)
diag = 22
obs1_w = [[150-diag, 240-diag], [260+diag, 240-diag], [260+diag, 350+diag], [150-diag, 350+diag]]
obs2_w = [[400-diag, 250-diag], [500+diag, 250-diag], [500+diag, 360+diag], [400-diag, 360+diag]]
obs3_w = [[480-diag, 420-diag], [570+diag, 420-diag], [570+diag, 610+diag], [480-diag, 610+diag]]
obs4_w = [[160-diag, 600-diag], [300+diag, 600-diag], [300+diag, 710+diag], [160-diag, 710+diag]]
obs5_w = [[600-diag, 270-diag], [770+diag, 270-diag], [770+diag, 380+diag], [600-diag, 380+diag]]
obs6_w = [[650-diag, 620-diag], [770+diag, 620-diag], [770+diag, 700+diag], [650-diag, 700+diag]]
obs7_w = [[770-diag, 150-diag], [860+diag, 150-diag], [860+diag, 800+diag], [770-diag, 800+diag]]

# Defining the obstacles
obstacle1 = patches.Polygon(obs1, closed=True, color='green', fill=True)
obstacle2 = patches.Polygon(obs2, closed=True, color='green', fill=True)
obstacle3 = patches.Polygon(obs3, closed=True, color='green', fill=True)
obstacle4 = patches.Polygon(obs4, closed=True, color='green', fill=True)
obstacle5 = patches.Polygon(obs5, closed=True, color='green', fill=True)
obstacle6 = patches.Polygon(obs6, closed=True, color='green', fill=True)
obstacle7 = patches.Polygon(obs7, closed=True, color='green', fill=True)

obs_all = [obs1,obs2,obs3,obs4,obs5, obs6, obs7]

##### Roadmap construction

# Generate random sample configs [x, y, theta] for the robot in the workspace.
sample_configs = []
sampling_threshold = 70 # Defines how many random configs would be sampled.

for i in range(sampling_threshold):
    config = [random.randint(101, 999), random.randint(101, 999), random.randint(0, 360)] 
    sample_configs.append(config)

# Remove invalid configs
valid_configs = []
for config in sample_configs:
    point = Point(config[0], config[1])
    flag = False
    for obs in obs_all:
        diag = 22
        obs = [[obs[0][0]-diag, obs[0][1]-diag],[obs[1][0]+diag, obs[1][1]-diag],[obs[2][0]+diag, obs[2][1]+diag],[obs[3][0]-diag, obs[3][1]+diag]]
        poly = Polygon(obs)
        if poly.contains(point):
            flag = True
    if flag is False:
        valid_configs.append(config)


configs_x, configs_y,_ = zip(*valid_configs)
configs_x = list(configs_x)
configs_y = list(configs_y)

# Define a list of all nodes ([x,y] for all valid configs) (including the start point and goal point)

nodes = [[x,y] for x,y,z in valid_configs]
nodes.append(start_xy)
nodes.append(goal_xy)

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

    [obs6[0], obs6[1]],
    [obs6[1], obs6[2]],
    [obs6[2], obs6[3]],
    [obs6[3], obs6[0]],

    [obs7[0], obs7[1]],
    [obs7[1], obs7[2]],
    [obs7[2], obs7[3]],
    [obs7[3], obs7[0]],
]

# Workspace Obs_edges (Padded obstacles)
obs_edges_w = [
    [obs1_w[0], obs1_w[1]],
    [obs1_w[1], obs1_w[2]],
    [obs1_w[2], obs1_w[3]],
    [obs1_w[3], obs1_w[0]],

    [obs2_w[0], obs2_w[1]],
    [obs2_w[1], obs2_w[2]],
    [obs2_w[2], obs2_w[3]],
    [obs2_w[3], obs2_w[0]],

    [obs3_w[0], obs3_w[1]],
    [obs3_w[1], obs3_w[2]],
    [obs3_w[2], obs3_w[3]],
    [obs3_w[3], obs3_w[0]],

    [obs4_w[0], obs4_w[1]],
    [obs4_w[1], obs4_w[2]],
    [obs4_w[2], obs4_w[3]],
    [obs4_w[3], obs4_w[0]],

    [obs5_w[0], obs5_w[1]],
    [obs5_w[1], obs5_w[2]],
    [obs5_w[2], obs5_w[3]],
    [obs5_w[3], obs5_w[0]],

    [obs6_w[0], obs6_w[1]],
    [obs6_w[1], obs6_w[2]],
    [obs6_w[2], obs6_w[3]],
    [obs6_w[3], obs6_w[0]],

    [obs7_w[0], obs7_w[1]],
    [obs7_w[1], obs7_w[2]],
    [obs7_w[2], obs7_w[3]],
    [obs7_w[3], obs7_w[0]],
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
    for segment2 in obs_edges_w:
        if segment1 != segment2:
            val = check_intersect(segment1, segment2)
            if val is not None:
                if val not in segment1 and val not in segment2:
                    flag = True
    if flag is False:
        vis_segments.append(segment1)

# Remove diagonals of polygons
#for segment in vis_segments:
#    for obs in obs_all:
#        if segment[0] in obs and segment[1] in obs:
#            if segment not in obs_edges:
#                vis_segments.remove(segment)

def plot_segments(all_segments, lw, color):
    for segment in all_segments:
        seg_x, seg_y = zip(*segment)
        _, = ax.plot(seg_x, seg_y, lw=lw, color=color)

plot_segments(vis_segments, 1, None)

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
linepath, = ax.plot(robot_path_x, robot_path_y, lw=1, color='black')  

robot = patches.Rectangle(start_xy, 15, 15, angle=0, fc='red')

# Showing all valid configurations as X on the map. And start and end points as a circle.
plt.plot(configs_x, configs_y, 'x', color='black')
plt.plot(start_xy[0], start_xy[1], 'o', color='blue')
plt.plot(goal_xy[0], goal_xy[1], 'o', color='teal')

# Get intermediate points (x, y, forward heading angle theta) on the path for animation.

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

rob_theta = []
rob_theta.append(0)
for i in range(1, 360):
    theta = math.degrees(math.atan((rob_y[i]-rob_y[i-1])/(rob_x[i]-rob_x[i-1])))
    rob_theta.append(theta)

def init():
    ax.add_patch(start_point)
    ax.add_patch(goal_point)
    ax.add_patch(obstacle1)
    ax.add_patch(obstacle2)
    ax.add_patch(obstacle3)
    ax.add_patch(obstacle4)
    ax.add_patch(obstacle5)
    ax.add_patch(obstacle6)
    ax.add_patch(obstacle7)
    ax.add_patch(robot)
    return robot,

def animate(i):
    robot.set_xy([rob_x[i], rob_y[i]])
    robot.angle = rob_theta[i]
    return robot,
    
anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=360,
                               interval=100,
                               blit=True,
                               repeat=True)
plt.show()