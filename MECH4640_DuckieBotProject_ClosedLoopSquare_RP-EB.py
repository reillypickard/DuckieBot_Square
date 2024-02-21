# Mech4640 Project - Closed Loop Square for a DuckieBot
# March 27th, 2023
# Reilly Pickard and Evan Burns - DuckieBot 95
# This python program controls a DuckieBot as it executes
# a 0.5x0.5m square. It first defines the reference trajectory
# using 8 waypoints (4 straightaways, 4 turns), before starting a loop
# to minimize the error for each waypoint. Once the error is minimized,
# the loop moves to the next waypoint.
# The code calls on the velocityController() written in
# the motorDriver.py script. This allows for the moveVel(vL, vR) function
# to be called to assign the left and right wheel velocities.
# moveVel also returns the encoder counts for each wheel, their velocities,
# and their times. This allows for the current pose of the robot
# to be calculated with differentially steered robot kinematics.

#Imports
import sys
import picamera
import os
sys.path.insert(0, os.getcwd())
sys.path.append('libraries')
import time
import math as m
import numpy as np
import pigpio
from motorDriver import velocityController
import matplotlib
import matplotlib.pyplot as plt

#Assign vc to class velocityController()
vc = velocityController()
#Assign camera to picamera.PiCamera()
camera = picamera.PiCamera()
camera.resolution = (2592, 1944) #Camera Res
camera.annotate_background = picamera.Color('black')

#Initialize Variables
a = 0.095/2 # Wheel base = 2a
vQ = 0.0 #Initial average velocity
#Define 8 waypoints. This is for turning RIGHT
ref = [(0.5,0, 0), (0.5,0, -np.pi/2), (0.5,-0.5, -np.pi/2),
       (0.5,-0.5, -np.pi),(0.0,-0.5, -np.pi),
       (0.0,-0.5, -3*np.pi/2), (0.0,0.0, -3*np.pi/2), (0.0,0.0,-2*np.pi)]

#x, y, theta ref for plotting
x_ref = [0.5, 0.5,0.5,0.5, 0.0, 0.0, 0.0, 0.0]
y_ref = [0,0,-0.5, -0.5,-0.5, -0.5,0, 0]
theta_ref = [0, -np.pi/2, -np.pi/2, -np.pi, -np.pi, np.pi/2, np.pi/2, -2*np.pi]

i = 0 #Initialize i
target = ref[i] #initialize target
dt = 0 # Initialize time difference
t = 0
#Assign gains - these are tuned and are case by case
K_rho = 0.5 #Keep low
K_alpha =17 # Make higher then K_rho
K_beta = -0.01 #Keep low and <1

stop = False #Initalize stop flag as false
pause = False #Initialize pause flag as false

#InitalizeSet all pose variables and vels to 0
x = 0
y = 0
theta = 0
thetaw = 0
thetaxy = 0
vL = 0
vR = 0

#Arrays to be filled with values for plotting
x_plot = []
y_plot = []
x_turn = []
y_turn = []
V = []
W = []
tim = []
turn_time = []
Vc = []
Wc = []
theta_plot =[]
theta_des = []
theta_err = []
xd_plot = []
yd_plot = []
x_err = []
y_err = []

#Capture first picture
camera.capture("Start.jpg")

while not stop: #Run loop until stop flag is TRUE
    if(i%2 ==0): #Even iterations are straight segments
        target = ref[i] # Update Target
        # Call moveVel(), return measurements
        l_sig, r_sig, l_time, r_time, l_vel, r_vel, l_enc, r_enc = vc.moveVel(vL, vR)

        vQ = (l_vel + r_vel) / 2 #Compute avg. velocity
        w = (r_vel - l_vel) / (a * 2) # Compute angular velocity
        thetaw = thetaw + w*dt # This is the wrapped theta - to be used in alpha calculation
        thetaxy = thetaxy + w*dt # Used to compute current x-y pose, unwrapped
        theta = theta + w*dt # Theta for turns 
        
        x = x + vQ * dt * np.cos(thetaxy) # Update x
        y = y + vQ * dt * np.sin(thetaxy) # Update y
        current = (x, y) #Current pose

        dx = target[0] - current[0] #Compute X error
        dy = target[1] - current[1]# Compute Y error
        rho = np.sqrt(dx**2 + dy**2) +0.2  #Compute rho - add 0.2 for torque and speed when rho tends to zero
       
        thetaw = (thetaw+np.pi)%(2*np.pi)-np.pi # Wrap theta to [-pi,pi]
        alpha = np.arctan2(dy, dx) -thetaw # Compute alpha
        alpha = (alpha +np.pi)%(2*np.pi)-np.pi #Wrap alpha
        beta = -thetaw - alpha #Compute Beta
        beta = (-beta -alpha+ np.pi)%(2*np.pi)-np.pi #wrap beta
        
        theta_d = (target[2]+np.pi)%(2*np.pi) - np.pi #wrap desired angle
        theta_des.append(target[2]) # Append desired angle
        theta_plot.append(thetaxy) # Append theta
        theta_err.append(target[2] - thetaxy) # Append theta error
            
        if np.abs(alpha) <= np.pi/2: #Check if robot is not past target
            V_out = K_rho * rho # Compute outputted velocity
            W_out = K_alpha * alpha + K_beta *beta #Compute outputted angular velocity
            vL = V_out - W_out * a #Calculate new desired left velocity
            vR = V_out + W_out * a #Calculate new desired right velocity
        else: #We are past waypoint, negate velocity and turn around
            V_out = 0
            W_out = K_alpha * alpha + K_beta *beta 
            vL = -W_out * a
            vR = W_out *a

        #Limit maximum velocity of wheel speed commands
        if(vR > 0.2):
            vR = 0.2
        elif(vR<-0.2):
            vR = -0.2
        else:
            vR = vR  
        if(vL> 0.2):
            vL = 0.2
        elif(vL<-0.2):
            vL = -0.2
        else:
            vL = vL

        #Stopping condition, we are within 0.01m of target
        if rho < 0.21:
            #theta=0
            i += 1 #Update i to go to next waypoint
            pause = True # Pause the system 
        
    else: # Odd iterations - Turning to desired pose
        target = ref[i]# Update Target
        # Send desired velocitiesnad return measurments
        l_sig, r_sig, l_time, r_time, l_vel, r_vel, l_enc, r_enc = vc.moveVel(vL, vR)
        vQ = (l_vel + r_vel) / 2 # Compute avg vel
        w = (r_vel - l_vel) / (a * 2) # Compute angular velocity
        # Compute angles
        theta = theta + w*dt 
        thetaxy = thetaxy + w*dt
        thetaw = thetaw+w*dt
        thetaw = (thetaw+np.pi)%(2*np.pi)-np.pi

        # Update Pose
        x = x + vQ * dt * np.cos(thetaxy)
        y = y + vQ * dt * np.sin(thetaxy)
        #theta = (theta+np.pi)%(2*np.pi)-np.pi
        
        theta_des.append(target[2])
        theta_plot.append(thetaxy)
        theta_err.append(target[2] - theta)
        #Compute theta error
        theta_error = target[2] - theta
        V_out = 0 # Dont go forward when turning
        W_out = 1.5*theta_error # Assign angular velocity proportional to error
       
        vL = V_out - W_out * a # New desired left velocity
        vR = V_out + W_out * a # NEw desired right velocity
        
        #Stopping condition on turns = 0.05 rads
        if abs(theta_error) < 0.05:
            theta=target[2] #Update theta
            #Pause motors
            l_sig, r_sig, l_time, r_time, l_vel, r_vel, l_enc, r_enc = vc.moveVel(0, 0)

            i += 1 # Increment i
            
            pause = True # Pause flag
            
    if(pause == True): # Pause routine
        t0 = time.time() # initialize time
        while time.time()-t0 < 2: # Pause for 2 seconds
            camera.capture("ClosedLoop{0:02d}.jpg".format(i)) # Take a picture on pause
            #Stop motors
            l_sig, r_sig, l_time, r_time, l_vel, r_vel, l_enc, r_enc = vc.moveVel(0, 0)
            # Update variables for plot
            vQ = (l_vel + r_vel) / 2
            w = (r_vel - l_vel) / (a * 2)
            theta = theta + 0
            x = x + 0
            y = y + 0
            current = (x, y)
            
        pause = False # Resume 
        
    if i >= len(ref): # If all waypoints met, stop program
        stop = True
    
    #Update time    
    t += dt
    tim.append(t)

    #Append plotting values
    V.append(r_vel + l_vel/2)
    W.append(r_vel - l_vel/(2*a))
    Vc.append(vR + vL/2)
    Wc.append(vR - vL/(2*a))
    x_plot.append(x)
    y_plot.append(y)
    xd_plot.append(target[0])
    yd_plot.append(target[1])
    x_err.append(target[0]  -x)
    y_err.append(target[1] - y)
    dt = (l_time + r_time)/2
    

#Shut off motors
vc.moveVel(0, 0)

#Plot results
plt.figure()
plt.subplot(311)
plt.title("Closed Loop Results - Cardboard")
plt.plot(tim, V, label = "Average Velocity", color = "blue")
plt.plot(tim, Vc, label = "Target Average Velocity", color = "red")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.grid(True)

plt.legend()


plt.subplot(312)
plt.plot(tim, W, label = "Angular Velocity", color = "blue")
plt.plot(tim, Wc, label = "Target Angular Velocity", color = "red")
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.grid(True)
plt.legend(loc = 'best')

plt.subplot(313)
plt.plot(tim, theta_plot, label = "Heading", color = "blue")
plt.plot(tim, theta_des, label = "Desired Heading", color = "red")
plt.grid(True)

plt.legend(loc = 'best')
plt.xlabel("Time (s)")
plt.ylabel("Heading (rads)")
plt.yticks(np.arange(0, -7.85, -1.57), np.arange(0, -7.85, -1.57))
plt.legend()


#plt.figure()

##plt.subplot(311)
##plt.title("CL Position Results - Map")
##plt.plot(tim,x_plot, label = "x Position", color = "blue")
##plt.plot(tim, xd_plot, label = "Desired x Position", color = "black")
##plt.grid(True)
##plt.ylabel("x position (m)")
##plt.xlabel("Time (s)")
##plt.legend()
##
##plt.subplot(312)
##plt.plot(tim,y_plot, label = "y Position", color = "blue")
##plt.plot(tim, yd_plot, label = "Desired y Position", color = "red")
##plt.grid(True)
##plt.ylabel("y position (m)")
##plt.xlabel("Time (s)")
##plt.legend()
##
##plt.subplot(313)
##plt.plot(tim,y_err, label = "y error", color = "orange")
##plt.plot(tim, x_err, label = "x error", color = "green")
##plt.grid(True)
##plt.ylabel("Position Error (m)")
##plt.xlabel("Time")
##plt.legend()

plt.figure()
plt.title("Closed Loop Trajectory - Cardboard")
plt.plot(x_ref, y_ref, color = "red", label = "desired")
plt.plot(x_plot,y_plot, color = "blue", label = "measured")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.legend(loc = 'best')
plt.grid(True)

plt.show()


