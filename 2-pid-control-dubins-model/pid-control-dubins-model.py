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
ax.set_xlim(-60, 60)
ax.set_ylim(-60, 60)

# Car dimensions
car_width = 1
car_height = 1.5

# State variables # for line: x0=-55,y0=10; for circle:x0=0,y0=0
x0 = -55
y0 = 10
theta0 = 0.0 # degrees

# Control variables
v = 5 # m/sec
w = 0 # degrees/sec

patch = patches.Rectangle((x0, y0), car_width, car_height, angle=theta0, fc='r')

circle = patches.Circle((0,0), 5, fill=False)

#lane1 = ax.hlines(44.5, 0, 44.5)
#lane2 = ax.vlines(44.5, 0, 44.5)
#lane3 = ax.hlines(0, 0, 44.5)
#lane4 = ax.vlines(0, 0, 44.5)

lane = ax.hlines(5, -50, 50)

line, = ax.plot([], [], lw=2) 

text = ax.text(-15, 60, '')

xdata, ydata = [], [] 

# Constants for PID controller (PID for circle: 20,0.5,10; PID for line: 20,0.5,10)
kp = 20
ki = 0.5
kd = 10
err = 0
err_sum = 0
err_prev = 0
dt = 0.1

# Track Coordinates
x_track = []
y_track = []
#z_track = []

def define_track():
    global x_track, y_track, z_track
    for i in range(0, 90):
        x_track.append(i/2)
    for i in range(90):
        x_track.append(44.5)
    for i in range(89,-1,-1):
        x_track.append(i/2)
    for i in range(90):
        x_track.append(0)

    for i in range(0,90):
        y_track.append(44.5)
    for i in range(89, -1, -1):
        y_track.append(i/2)
    for i in range(90):
        y_track.append(0)
    for i in range(0,90):
        y_track.append(i/2)

    #for i in range(0,90):
    #    z_track.append(0)
    #for i in range(90, 180):
    #    z_track.append(270)
    #for i in range(0, 90):
    #    z_track.append(180)
    #for i in range(0,90):
    #    z_track.append(90)
    

def calculate_error(t):
    global err, err_sum, x0, y0, theta0, err_prev, x_track, y_track, z_track
    err_prev = err
    # For a circular path centered at 0,0 and radius=5
    #r=5
    #err = np.sqrt((x0**2) + (y0**2))-r # Error for circular path
    
    # Error for line at y=5
    err = np.sqrt((y0 - 5)**2)
    if y0 > 5:
        err = -err

    # Error for Square lane

    #if t >= 0 and t <= 90:
    #    err = np.sqrt((y0-44.5)**2)
    #    if y0 > 44.5:
    #        err = -err
    #elif t > 90 and t <= 180:
    #    err = np.sqrt((x0-44.5)**2)
    #    if x0 > 44.5:
    #        err = -err
    #elif t > 180 and t <= 270:
    #    err = np.sqrt((y0-0)**2)
    #    if y0 < 0:
    #        err = -err
    #elif t > 270 and t <= 360:
    #    err = np.sqrt((x0-0)**2)
    #    if x0 < 0:
    #        err = -err 

    #err = np.sqrt(((x0-x_track[t])**2)+((y0-y_track[t])**2))
    #if y0 > 5:
    #    err = -err
    
    #d1 = np.sqrt(((22.25-x_track[t])**2)+((22.25-y_track[t])**2))
    #d2 = np.sqrt(((x0-22.25)**2)+((y0-22.25)**2))

    #err = d1 - d2

    #if d2 - d1 > 0:
    #    err = -err
    
    err_sum += err

def set_w_from_pid_controller(t):
    global kp, ki, kd, err, w, dt, err_sum, err_prev
    calculate_error(t)
    #print(w)
    w = (kp*err) + (ki*err_sum) + (kd*((err-err_prev)/dt))
    

def set_new_state(v, w):
    global x0, y0, theta0
    x0 = x0 + (v * np.cos(np.radians(theta0))*0.1) # delta_t = 0.1
    y0 = y0 + (v * np.sin(np.radians(theta0))*0.1)
    theta0 = theta0 + (w*0.1)

def init():
    #get_control_variables()
    define_track()
    ax.add_patch(patch)
    ax.add_patch(circle)
    return patch,

def animate(i):
    global car_height, car_width, v, w, err, theta0
    # Animating Car
    set_w_from_pid_controller(i)
    set_new_state(v, w)
    patch.set_width(car_height)
    patch.set_height(car_width)
    patch.set_xy([x0, y0])
    patch.angle = theta0

    # Animating text
    # Note: (theta0 % 360) for better clarity of driving angle while plotting
    loc = "x: {:.2f} \ny: {:.2f} \ntheta: {:.2f} \nv: {:.2f} m/s \nw: {:.2f} deg/s \nErr: {:.2f}".format(x0, y0, theta0%360, v, w, err) 

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