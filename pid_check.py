import airsim
import time
from plot import Plot
from garage import pid_car
import math
import numpy as np

# Connect to the AirSim simulator
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

# Define the sample time to perform all processing
sample_time = 0.1  # [s]

# Define the speed step parameters
speed_step_duration = 5  # [s]
speed_step_amplitude = 10.0  # [m/s]
speed_step_samples = math.floor(speed_step_duration / sample_time)

# Define the track angle step parameters
track_angle_step_duration = 3  # [s]
track_angle_step_amplitude = np.deg2rad(2.5)  # [rad]
track_angle_step_samples = math.floor(track_angle_step_duration / sample_time)

# Create the PID car
pid_car1 = pid_car.Car(client, sample_time, 'SetCarName1', '2', filename='run-fast4.pickle') 

# Create the plotter
p = Plot(blit=True, sample_interval=sample_time)

# Get initial track angle
initial_track_angle = np.deg2rad(-90.0) #pid_car1.getCurrentTrackAngle()
print("initial_track_angle: " + str(np.rad2deg(initial_track_angle)) + " [deg]")

# Apply a speed step keeping the current track angle
for x in range(speed_step_samples):
    pid_car1.drive(speed_step_amplitude, initial_track_angle)
    client.simPause(False)
    time.sleep(sample_time)
    client.simPause(True)
    p.update(pid_car1)

    #print("current_track_angle: " + str(np.rad2deg(pid_car1.getCurrentTrackAngle())) + " [deg]")

print(" Apply a track angle step to the right keeping the speed target: " + str(np.rad2deg(track_angle_step_amplitude)))
# Apply a track angle step to the right keeping the speed target
for x in range(track_angle_step_samples):
    pid_car1.drive(speed_step_amplitude, initial_track_angle + track_angle_step_amplitude)
    client.simPause(False)
    time.sleep(sample_time)
    client.simPause(True)
    p.update(pid_car1)

# Apply a track angle step to the left keeping the speed target
print(" Apply a track angle step to the left keeping the speed target: " + str(np.rad2deg(-track_angle_step_amplitude)))
for x in range(track_angle_step_samples):
    pid_car1.drive(speed_step_amplitude, initial_track_angle - track_angle_step_amplitude)
    client.simPause(False)
    time.sleep(sample_time)
    client.simPause(True)
    p.update(pid_car1)

# # Stop the car
print(" Stop the car ")
for x in range(speed_step_samples):
    pid_car1.drive(0, initial_track_angle)
    client.simPause(False)
    time.sleep(sample_time)
    client.simPause(True)
    p.update(pid_car1)

pid_car1.resetControls()

# Restore to original car state
time.sleep(60)
client.reset()
client.enableApiControl(False)