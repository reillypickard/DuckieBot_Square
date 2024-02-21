# Mech4640 Project - Open Loop Square for a DuckieBot
# March 27th, 2023
# Reilly Pickard and Evan Burns - DuckieBot 95
# This python is an open loop controller to get a DuckieBot to execute
# a 0.5x0.5m square. # It commands the DuckieBot using
#the moveVl function right and left velocities of 0.25 m/s for
# Straight portions that are held on straight loop iterations
# for 2.0s. To turn 90 degrees to the right on turning iterations, the commanded
# left velocity is 0.125 m/s and the commanded right velocity
# is -0.125 m/s. The corresponds to an angular velocity of 2.63
# rad/s, and this command is therefore held for 0.6 seconds to turn pi/2 rads.
# There is a pause after every segment, meaning that straight segments
# are every 4th iteration. 


#Imports
import sys
import os
sys.path.insert(0, os.getcwd())
sys.path.append('libraries')
from dagu_wheels_driver import DaguWheelsDriver
import time
import math as m
import numpy as np
import pigpio
import picamera
import matplotlib
import matplotlib.pyplot as plt


# Initialize the velocityController and camera objects
from motorDriver import velocityController
vc = velocityController()
camera = picamera.PiCamera()
camera.resolution = (2592, 1944)


## Store Velocities for plotting
RVc= []
LVc= []
LV = []
RV = []
tim = []
movement=np.matrix([[0,0,0,0,0,0]])
lr=np.matrix([[0.25,0.25]])

a = 0.095/2 # Wheel Base

# Velocity targets (Pause after Each)       
velocities = [0.25, 0, 0.125, 0, 0.25, 0, 0.125, 0, 0.25, 0,0.125,0,0.25,0,0.125]

# Define the time for each movement in seconds
times = [2, 2, 0.6, 2, 2, 2, 0.6, 2,2,2,0.6,2,2,2,0.6]
#Initialize Theta and Desired Theta (For Plotting)
theta = 0
thetades = 0
thetdes = []
thet = []
thet.append(theta)
thetdes.append(theta)

#Initialize xy coords (For Plotting)
x = 0
xd = 0
x_plot = []
xd_plot = []
x_plot.append(x)
xd_plot.append(0.5)
y=0
y_plot=[]
y_plot.append(y)
yd_plot = []
yd_plot.append(xd)
x_err = []
x_err.append(0.5)
y_err = []
y_err.append(0)
w_plot = []
w_plot.append(0)

#Reference Trajectory for Plotting
x_ref = [0.5, 0.5,0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0,0,0,0,0,0,0]
y_ref = [0, 0,0,0,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,0,0,0]   
theta_err = []
theta_err.append(0)
dt =0

# Loop through the movements
for i in range(len(velocities)):
    #Take Pic Between Iterations
    camera.capture("OpenLoop{0:02d}.jpg".format(i))
    # Start loop 
    while time.time() - t0 < times[i]:
        if(i%4 ==0): #Straight Portion
            #Set wheel Vels and Generate Measurements
            l_sig, r_sig, l_time, r_time, l_vel, r_vel, l_enc, r_enc = vc.moveVel(velocities[i], velocities[i])
            lr = np.append(lr, [[velocities[i], velocities[i]]], axis=0)
            wdes = 0#For plotting                
        else: #else we are either turning or pausing
            #Set wheel Vels and Generate Measurements
            l_sig, r_sig, l_time, r_time, l_vel, r_vel, l_enc, r_enc = vc.moveVel(velocities[i], -velocities[i])
            lr = np.append(lr, [[velocities[i], -velocities[i]]], axis=0)

            if(velocities[i] == 0):
                wdes = (-2*velocities[i])/(2*a) #For plotting pauses
            else:
                wdes = (-2*0.125)/(2*a) #For plotting angular vel on turns

                
        movement = np.append(movement, [[l_time, r_time, l_vel, r_vel, l_enc, r_enc]], axis=0)
        #lr = np.append(lr, [[velocities[i], -velocities[i]]], axis=0)

        #Compute kinematics for plotting the measurements
        thetades = thetades + wdes*(l_time +r_time)/2
        vQ = (r_vel + l_vel)/2
        w = (r_vel - l_vel)/(2*a)
        theta =  theta + w*(l_time+r_time)/2
        x += vQ*np.cos(theta)*(l_time+r_time)/2
        x_plot.append(x)
        y += vQ*np.sin(theta)*(l_time+r_time)/2
        y_plot.append(y)
        thet.append(theta)
        thetdes.append(thetades)
        theta_err.append(thetades -theta)
        xd_plot.append(x_ref[i])
        yd_plot.append(y_ref[i])
        x_err.append(x_ref[i] - x)
        w_plot.append((r_vel - l_vel)/(2*a))
        y_err.append(y_ref[i] - y)
        

vc.moveVel(0, 0) #Stop motors after all waypoints met

# Construct points to plot
avgVel = np.zeros(len(movement))
W = np.zeros(len(movement))
avgVelD = np.zeros(len(movement))
WD = np.zeros(len(movement))
for i in range (1, len(movement)):
    movement[i,0] = movement[i,0] + movement[i-1,0]
    movement[i,1] = movement[i,1] + movement[i-1,1]
    avgVel[i] = (movement[i,2]+movement[i,3])/2
    avgVelD[i] = (lr[i,0]+lr[i,1])/2
    W[i] = (movement[i,3]-movement[i,2])/2*a
    WD[i] = (lr[i,1]-lr[i,0])/(2*a)
    #print(movement[i,3]-movement[i,2])
    #print(W[i])

# Plot points for comparison
plt.figure()

plt.subplot(311)
plt.title("Open Loop Results")
plt.plot(movement[:,0], avgVel, label = "Average Velocity", color = "blue")
plt.plot(movement[:,0], avgVelD, label = "Target Average Velocity", color = "red")
plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.grid(True)


plt.subplot(312)
plt.plot(movement[:,0], w_plot, label = "Angular Velocity", color = "blue")
plt.plot(movement[:,0], WD, label = "Target Angular Velocity", color = "red")
plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.grid(True)

plt.subplot(313)
plt.plot(movement[:,0], thet, label = "Heading", color = "blue")
plt.plot(movement[:,0], thetdes, label = "Desired Heading", color = "red")
plt.legend()
plt.grid(True)
plt.xlabel("Time (s)")
plt.ylabel("Heading (rads)")
plt.yticks(np.arange(0, -7.85, -1.57), np.arange(0, -7.85, -1.57))


plt.figure()
plt.title("Open Loop Trajectory")
plt.plot(xd_plot,yd_plot, color = "red", label = "desired trajectory")
plt.plot(x_plot,y_plot, color = "blue", label = "measured trajectory")
plt.xlabel("x (m)")
plt.ylabel("y(m)")
plt.legend()
plt.grid(True)



plt.figure()

plt.subplot(311)
plt.title("Open Loop Position Results")
plt.plot(movement[:,0],xd_plot,  label = "Desired x Position", color = "red")
plt.plot(movement[:,0],x_plot, label = "x Position", color = "blue")
plt.ylabel("Position (m)")
plt.xlabel("Time(s)")
plt.grid(True)
plt.legend()
plt.subplot(312)
plt.plot(movement[:,0],yd_plot,  label = "Desired y Position", color = "red")
plt.plot(movement[:,0],y_plot, label = "y Position", color = "blue")
plt.ylabel("Position (m)")
plt.xlabel("Time(s)")
plt.grid(True)
plt.legend()

plt.subplot(313)
plt.plot(movement[:,0],x_err,  label = "x error", color = "orange")
plt.plot(movement[:,0],y_err, label = "y error", color = "green")
plt.legend()
plt.ylabel("error (m)")
plt.xlabel("Time(s)")
plt.grid(True)
plt.show()

