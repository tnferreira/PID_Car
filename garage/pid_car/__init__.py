##########################################
### Skoods.org -> Self-Racing Car Team ###
##########################################

import airsim
from garage.pid_car import localization, planning, control
import numpy as np


class Car:
    def __init__(self, client, sample_time, car_name, mode_input, waypoints_correction=[0, 0], filename = 'something.pickle'):
        self.client = client
        self.sample_time = sample_time
        self.name = car_name
        self.mode_input = mode_input
        self.waypoints_correction = waypoints_correction
        self.waypoints = planning.Waypoints(self.name) # Initialize waypoints object
        self.filename = filename

        if self.mode_input == '2' or self.mode_input == '3': # If Qualify or Race
            self.client.enableApiControl(True, self.name)
            self.keep_racing = True
            self.loadWaypointsFromFile() # load waypoints previously recorded
            self.updateTrajectory() # Get x, y and v lists from waypoints. could be inside the while loop if dynamic trajectory, but would decrease performance
            self.behavior = planning.Behavior(self) # Set the starting behavior of the car: 'start' or 'cruze'. You can create more behaviors.
            self.resetControls() # Initialize controls

            ### INITIALIZE PID CONTROLLER PARAMETERS
            #throttle_pid_params = [0.2, 0.03, 0.08] #best values ~20m/s
            #steering_pid_params = [0.1, 0.00, 0.18] #best values ~20m/s
            throttle_pid_params = [0.2, 0.03, 0.08]
            steering_pid_params = [0.1, 0.00, 0.18]
            speed_pid_params = [0.2, 0.03, 0.08]
            track_angle_pid_params = [0.1, 0.00, 0.18]

            # Lower and upper control limits
            throttle_limits = [-1.0, 1.0]
            steering_limits = [-0.5, 0.5]

            # Initialize PID objects
            self.throttle_controller = control.PIDThrottleControl(self, throttle_pid_params, self.sample_time,
                                                                  throttle_limits)
            self.steering_controller = control.PIDSteeringControl(self, steering_pid_params, self.sample_time,
                                                                  steering_limits)
            self.steering_controller.setTargetValue()  # It's always zero (different from Throttle PID)

            self.speed_controller = control.PIDSpeedControl(self, speed_pid_params, self.sample_time,
                                                            throttle_limits)
            self.track_angle_controller = control.PIDTrackAngleControl(self, track_angle_pid_params, self.sample_time,
                                                                       steering_limits)
        
        else:  # Record Waypoints
            self.recordWaypointsToFile()


    def resetControls(self):
        self.controls = airsim.CarControls()
        self.client.setCarControls(self.controls, self.name)

    def updateState(self):
        self.state = localization.updateState(self)
        self.state.kinematics_estimated.position.x_val += self.waypoints_correction[0]
        self.state.kinematics_estimated.position.y_val += self.waypoints_correction[1]

    def updateCarBehavior(self):
        self.behavior.setCarBehavior()

    def loadWaypointsFromFile(self):
        self.waypoints.loadWaypointsFromFile(filename=self.filename)

    def recordWaypointsToFile(self, sample_time = 0.005):
        self.waypoints.recordWaypointsToFile(self, sample_time, filename = self.filename)

    def updateTrajectory(self):
        self.waypoints_x, self.waypoints_y, self.waypoints_v = self.waypoints.waypointsToLists(self.waypoints_correction)

    def updateControls(self):
        # Run Throttle PID
        keep_racing_throttle = self.throttle_controller.setTargetValue(self, self.waypoints_x, self.waypoints_y, self.waypoints_v) # Set goal each interaction, as speed target will change
        self = self.throttle_controller.getControlsFromPID(self) # Return controls after running PID (this method is different for each PID type)

        # Run Steering PID
        self, keep_racing_steering = self.steering_controller.getControlsFromPID(self, self.waypoints_x, self.waypoints_y) # Return controls after running PID (this method is different for each PID type)
        keep_racing = (keep_racing_throttle and keep_racing_steering)
        keep_racing = True # Force race all the time
        if keep_racing:
            self.setControls() # Send controls to simulation
        else:
            self.resetControls()
            print(self.name + " || WARNING: WAYPOINTS - Waypoints out of range. You didn't complete the lap :(")
        return keep_racing


    def setControls(self):
        # After updating the controls, we have to set the controls to update the simulation
        # We are also handling what to do with different behavior modes here
        if self.behavior.mode == 'CRUZE':
            self.client.setCarControls(self.controls, self.name)
        elif self.behavior.mode == 'START':
            self.controls.throttle = 1.0
            self.controls.brake = 0.0
            # self.controls.steering = 0.0
            self.client.setCarControls(self.controls, self.name)


    def race(self):
        self.updateState() # update position and other data
        self.updateCarBehavior() # define the behavior of the car based on conditions
        keep_racing = self.updateControls() # return false if waypoints reach final index
        return keep_racing


    def drive(self, speed, steering):
        self.updateState()  # update position and other data
        self.updateCarBehavior()  # define the behavior of the car based on conditions

        # Run Speed PID
        self = self.speed_controller.getControlsFromPID(self, speed)

        # Run Track Angle PID
        self = self.track_angle_controller.getControlsFromPID(self, steering)

        # Send controls to simulation
        self.setControls()


    def getCurrentSpeed(self):
        return self.state.speed


    def getCurrentTrackAngle(self):
        return np.arctan2(self.state.kinematics_estimated.linear_velocity.y_val,
                          self.state.kinematics_estimated.linear_velocity.x_val)

