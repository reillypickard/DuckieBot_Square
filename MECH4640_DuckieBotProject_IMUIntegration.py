#Mech4640 Term Project - IMU Data integration Scrip
#Reilly Pickard & Evan Burns - Duckie #95
#March 27th, 2023
#This code integrates the IMU data captured during each
# experiment on the iPhone physics sensor suite.
# The csv files were cleaned so that the time was
# converted to seconds for integration, and the only
# measurements taken from the IMU were the forward
# acceleration (ay) and the yaw rate from the gyroscop (wz).
# Integration is performed using the cumulative trapezoid rule
# "cumtrapz" from the scipy.integrate library.
# There was 3 experiments that generated IMU data, so there
# is 3 datasets for closed loop, and 3 for open loop.

# imports
import pandas as pd
import scipy.integrate as it
import numpy as np 
import matplotlib.pyplot as plt

#Experiment 1 closed loop
df = pd.read_csv('ClosedLoopIMU2.csv', usecols = ['time2', 'ay', 'wz'],
                 low_memory = False) #read data

dt = np.diff(df['time2']) #time difference

vx = it.cumtrapz(df['ay'], dx=dt, initial=0) #velocity
x = it.cumtrapz(vx, dx=dt, initial=0) #displacement


theta = it.cumtrapz(df['wz'], dx=dt, initial=0) #heading
df['time2'] = df['time2'] - df['time2'].iloc[0] #Time back to 0

#Plot Results

plt.figure()
plt.subplot(131)
plt.title("Closed Loop Square")
plt.plot(df.time2, x, color = "black" )
plt.ylabel("Cumulative Displacement (m)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.subplot(132)
plt.plot(df.time2, vx, color = "black")
plt.ylabel("Linear Velocity (m/s)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.subplot(133)
plt.plot(df.time2, theta, color = "black")
plt.ylabel("Cumulative Heading (rads)")
plt.xlabel("Time (s)")
plt.grid(True)


##########
#Experiment 1 open loop
################
df = pd.read_csv('OL_imu.csv', usecols = ['time2', 'ay',  'wz'],
                 low_memory = False) #read data

dt = np.diff(df['time2']) #time difference

vx = it.cumtrapz(df['ay'], dx=dt, initial=0) #velocity
x = it.cumtrapz(vx, dx=dt, initial=0) #displacement


theta = it.cumtrapz(df['wz'], dx=dt, initial=0) #heading
df['time2'] = df['time2'] - df['time2'].iloc[0] #Time back to 0

#Plot Results

plt.figure()
plt.subplot(131)
plt.title("Open Loop Square")
plt.plot(df.time2, x, color = "black" )
plt.ylabel("Cumulative Displacement (m)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.subplot(132)
plt.plot(df.time2, vx, color = "black")
plt.ylabel("Linear Velocity (m/s)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.subplot(133)
plt.plot(df.time2, theta, color = "black")
plt.ylabel("Cumulative Heading (rads)")
plt.xlabel("Time (s)")
plt.grid(True)



##########
#Experiment 2 closed loop
################
df = pd.read_csv('CL_nudge.csv', usecols = ['time2', 'ay',  'wz'],
                 low_memory = False) #read data

dt = np.diff(df['time2']) #time difference

vx = it.cumtrapz(df['ay'], dx=dt, initial=0) #velocity
x = it.cumtrapz(vx, dx=dt, initial=0) #displacement


theta = it.cumtrapz(df['wz'], dx=dt, initial=0) #heading
df['time2'] = df['time2'] - df['time2'].iloc[0] #Time back to 0

#Plot Results

plt.figure()
plt.subplot(131)
plt.title("Closed Loop Square - Disturbance")
plt.plot(df.time2, x, color = "black" )
plt.ylabel("Cumulative Displacement (m)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.subplot(132)
plt.plot(df.time2, vx, color = "black")
plt.ylabel("Linear Velocity (m/s)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.subplot(133)
plt.plot(df.time2, theta, color = "black")
plt.ylabel("Cumulative Heading (rads)")
plt.xlabel("Time (s)")
plt.grid(True)


##########
#Experiment 2 open loop
################
df = pd.read_csv('OL_nudge.csv', usecols = ['time2', 'ay',  'wz'],
                 low_memory = False) #read data

dt = np.diff(df['time2']) #time difference

vx = it.cumtrapz(df['ay'], dx=dt, initial=0) #velocity
x = it.cumtrapz(vx, dx=dt, initial=0) #displacement


theta = it.cumtrapz(df['wz'], dx=dt, initial=0) #heading
df['time2'] = df['time2'] - df['time2'].iloc[0] #Time back to 0

#Plot Results

plt.figure()
plt.subplot(131)
plt.title("Open Loop Square - Disturbance")
plt.plot(df.time2, x, color = "black" )
plt.ylabel("Cumulative Displacement (m)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.subplot(132)
plt.plot(df.time2, vx, color = "black")
plt.ylabel("Linear Velocity (m/s)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.subplot(133)
plt.plot(df.time2, theta, color = "black")
plt.ylabel("Cumulative Heading (rads)")
plt.xlabel("Time (s)")
plt.grid(True)



##########
#Experiment 3 closed loop
################
df = pd.read_csv('CL_cardboard.csv', usecols = ['time2', 'ay',  'wz'],
                 low_memory = False) #read data

dt = np.diff(df['time2']) #time difference

vx = it.cumtrapz(df['ay'], dx=dt, initial=0) #velocity
x = it.cumtrapz(vx, dx=dt, initial=0) #displacement


theta = it.cumtrapz(df['wz'], dx=dt, initial=0) #heading
df['time2'] = df['time2'] - df['time2'].iloc[0] #Time back to 0

#Plot Results

plt.figure()
plt.subplot(131)
plt.title("Closed Loop Square - Cardboard")
plt.plot(df.time2, x, color = "black" )
plt.ylabel("Cumulative Displacement (m)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.subplot(132)
plt.plot(df.time2, vx, color = "black")
plt.ylabel("Linear Velocity (m/s)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.subplot(133)
plt.plot(df.time2, theta, color = "black")
plt.ylabel("Cumulative Heading (rads)")
plt.xlabel("Time (s)")
plt.grid(True)



##########
#Experiment 3 open loop
################
df = pd.read_csv('OL_cardboard.csv', usecols = ['time2', 'ay',  'wz'],
                 low_memory = False) #read data

dt = np.diff(df['time2']) #time difference

vx = it.cumtrapz(df['ay'], dx=dt, initial=0) #velocity
x = it.cumtrapz(vx, dx=dt, initial=0) #displacement


theta = it.cumtrapz(df['wz'], dx=dt, initial=0) #heading
df['time2'] = df['time2'] - df['time2'].iloc[0] #Time back to 0

#Plot Results

plt.figure()
plt.subplot(131)
plt.title("Open Loop Square - Cardboard")
plt.plot(df.time2, x, color = "black" )
plt.ylabel("Cumulative Displacement (m)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.subplot(132)
plt.plot(df.time2, vx, color = "black")
plt.ylabel("Linear Velocity (m/s)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.subplot(133)
plt.plot(df.time2, theta, color = "black")
plt.ylabel("Cumulative Heading (rads)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.show()
