#!/usr/local/bin/python3

"""
Author: Yashvardhan Jain
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation

fig = plt.figure()
plt.axis('equal')
ax = fig.add_subplot(111)
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)

# Car dimensions
car_width = 1
car_height = 1.5

# State variables
x0 = 0.0
y0 = 0.0
theta0 = 0.0 # degrees

# Control variables
v = [] # m/sec
w = [] # degrees/sec

patch = patches.Rectangle((x0, y0), car_width, car_height, angle=theta0, fc='r')

line, = ax.plot([], [], lw=2) 

text = ax.text(-9, 4, '')

xdata, ydata = [], [] 

def get_control_variables():
    global v,w
    v = [1]*100 + [1.2]*100 + [1.7]*100 + [2]*60 
    w = [30]*360

def set_new_state(v, w):
    global x0, y0, theta0
    x0 = x0 + (v * np.cos(np.radians(theta0))*0.1) # delta_t = 0.1
    y0 = y0 + (v * np.sin(np.radians(theta0))*0.1)
    theta0 = theta0 + (w*0.1)

def init():
    get_control_variables()
    ax.add_patch(patch)
    return patch,

def animate(i):
    global car_height, car_width, v, w
    # Animating Car
    set_new_state(v[i], w[i])
    patch.set_width(car_height)
    patch.set_height(car_width)
    patch.set_xy([x0, y0])
    patch.angle = theta0

    # Animating text
    # Note: (theta0 % 360) for better clarity of driving angle while plotting
    loc = "x: {:.2f} \ny: {:.2f} \ntheta: {:.2f} \nv: {:.2f} m/s \nw: {:.2f} deg/s".format(x0, y0, theta0%360, v[i], w[i]) 

    text.set_text(loc)

    # Animating car trail
    xdata.append(x0)
    ydata.append(y0)
    line.set_data(xdata, ydata)

    return patch, line, text, 

anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=360,
                               interval=100,
                               blit=True,
                               repeat=True)
plt.show()