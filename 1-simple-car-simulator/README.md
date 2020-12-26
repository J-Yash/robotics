# Simple Car Simulator

Implementing a simple car simulator that works on Dubins dynamics.

Car state - x, y, \theta

Control inputs - v (linear speed), w (angular speed)

Simulation visualization using matplotlib.

Setting values of v and w would move the car in the 2D plane accordingly.

Dubins Dynamics equations:

    x_1 = x_0 + v cos\theta_0 delta_t

    y_1 = y_0 + v sin\theta_0 delta_t

    \theta_1 = \theta_0 + w delta_t

These equations are used to calculate the state of the simulated car at each time step. In our case, each animation frame is a different time step. 
