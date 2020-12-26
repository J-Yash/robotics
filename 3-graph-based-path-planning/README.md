# Graph Based Path Planning for a robot

This code implements a graph based path planner for a simulated robot in a simulated environment that contains several obstacles.

- Create a simulation workspace environment that contains several obstacles (squares and triangles). We define a start point and an end point for the robot. The robot itself is defines as a point robot in the workspace.

- Generate a visibility graph of the environment for the robot to traverse. The graph is created using the corners of the obstacles and the start/end point as the nodes. The edges that collide with the obstacles are removed.

- Once the visibility graph is generated, a path from the start point to the end point is generated using Breadth First Search. This search algorithm guarantees a path (if possible) but may not always find the shortest path.

- Once the final path is found, the robot is traversed from the start point to the end point along the found path. This is implemented as an animation of the point robot along the path.
