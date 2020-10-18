import airsim
import time
from pid_plot import Plot
from garage import pid_car
import math
import numpy as np

# Connect to the AirSim simulator
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

# Define the sample time to perform all processing
sample_time = 0.01  # [s]

# Define the speed step parameters
speed_step_duration = 20  # [s]
speed_step_amplitude = 10.0  # [m/s]
speed_step_samples = math.floor(speed_step_duration / sample_time)

# Define the track angle step parameters
track_angle_step_duration = 3  # [s]
track_angle_step_amplitude = np.deg2rad(3.0)  # [rad]
track_angle_step_samples = math.floor(track_angle_step_duration / sample_time)

# Create the PID car
pid_car1 = pid_car.Car(client, sample_time, 'SetCarName1', '2', filename='run-fast4.pickle') 

# Create the plotter
p = Plot(blit=True)

# Get initial track angle
initial_track_angle = pid_car1.getCurrentTrackAngle()
# print("initial_track_angle: " + str(np.rad2deg(initial_track_angle)) + " [deg]")

# Apply a speed step keeping the current track angle
for x in range(speed_step_samples):
    pid_car1.drive(speed_step_amplitude, initial_track_angle)
    p.update(pid_car1)

# Apply a track angle step to the right keeping the speed target
for x in range(track_angle_step_samples):
    pid_car1.drive(speed_step_amplitude, initial_track_angle + track_angle_step_amplitude)
    p.update(pid_car1)

# Apply a track angle step to the left keeping the speed target
for x in range(track_angle_step_samples):
    pid_car1.drive(speed_step_amplitude, initial_track_angle - track_angle_step_amplitude)
    p.update(pid_car1)

# Stop the car
for x in range(speed_step_samples):
    pid_car1.drive(0, initial_track_angle)
    p.update(pid_car1)

pid_car1.resetControls()

# Restore to original car state
client.reset()
client.enableApiControl(False)
