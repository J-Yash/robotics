# Probabilistic Roadmap (PRM) Based Path Planning for a robot

This code implements a Probabilistic Roadmap (PRM) based path planner for a simulated robot in a simulated environment that contains several obstacles.

- Create a simulation workspace environment that contains several obstacles. We define a start point and an end point for the robot. The robot itself is defines as a square robot in the workspace. Two different workspace environments are created: 1. Obstacles scattered throughout the workspace 2. Workspace containing a narrow passage (tunnel).

- Sample random configurations of the various states of the robot in the workspace environment. These configurations are then used as nodes in the environment for which the visibility graph would be created. The start node and the end node are connected to every other node. 

- Generate a visibility graph of the environment for the robot to traverse. The graph is created using a straight-line local planner. Edges that collide with the obstacles are removed.

- Once the visibility graph is generated, a path from the start point to the end point is generated using Breadth First Search. This search algorithm guarantees a path (if possible) but may not always find the shortest path.

- Once the final path is found, the robot is traversed from the start point to the end point along the found path. This is implemented as an animation of the point robot along the path.

- PRM based configuration sampling reqiures much higher sampling number for the narrow passage case than the scattered obstacle case and hence takes a much longer time to process. Furthermore, the narrow passage case requires a modified sampling strategy in which extra random samples have to be taken specifically from the tunnel region in addition to the random samples taken from the overall workspace. This increases the probability of the path planner to actually find a proper path between the start point and the end point.
