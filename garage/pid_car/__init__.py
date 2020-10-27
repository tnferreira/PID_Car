##########################################
### Skoods.org -> Self-Racing Car Team ###
##########################################

import airsim
from garage.pid_car import localization, planning, control, guidance
import numpy as np
from airsim import utils
from plot import Plot

class Car:
    def __init__(self, client, sample_time, car_name, mode_input, waypoints_correction=[0, 0],
                 filename='something.pickle', compute_sample_time=False, show_profile=False, show_pid=False):
        """
        Default constructor.
        :param client: AirSim client
        :param sample_time: default sample time in seconds
        :param car_name: name of the car
        :param mode_input: race mode(1 RECORD, 2 QUALIFY, 3 RACE)
        :param waypoints_correction: X-Y waypoints correction
        :param filename: name of the file containing the recorded waypoints
        :param compute_sample_time: a flag to indicate whether the sample time shall be estimated or not
        :param show_profile: flag to enable the reference profile and vehicle trajectory visualization
        :param show_pid: flag to enable the PID controller input / output visualization
        """
        self.client = client
        self.sample_time = sample_time
        self.name = car_name
        self.mode_input = mode_input
        self.waypoints_correction = waypoints_correction
        self.waypoints = planning.Waypoints(self.name) # Initialize waypoints object
        self.filename = filename
        self.min_speed = 2.0  # meters per second
        self.compute_sample_time = compute_sample_time
        self.last_timestamp = []
        self.estimated_sample_time = []

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

            speed_pid_params = [1.5, 0.047, 0.1]
            #speed_pid_params = [1, 0.047, 0.1]
                        
            track_angle_pid_params = [2.3, 0.02, 0.1]
            #track_angle_pid_params = [0.99, 0.00, 0.1]


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

            self.path_planner = planning.PathPlanner(epsilon=0.25,
                                                     sample_time=self.sample_time,
                                                     number_samples=500,
                                                     min_distance=10)
            self.path_planner.update_reference_profile_from_recorded_waypoints(self.waypoints_x,
                                                                               self.waypoints_y,
                                                                               self.waypoints_v)

            # 9.5
            self.pure_pursuit = guidance.Guidance(max_straight_track_speed=30.5,
                                                  max_curving_speed=12.0,
                                                  max_turning_rate=5.0,
                                                  braking_distance=10.0)

            self.show_profile = show_profile
            self.show_pid = show_pid
            if show_pid:
                self.pid_plot = Plot(blit=True)
        
        else:  # Record Waypoints
            self.recordWaypointsToFile()

    def resetControls(self):
        self.controls = airsim.CarControls()
        self.client.setCarControls(self.controls, self.name)

    def updateState(self):
        self.state = localization.updateState(self)
        self.state.kinematics_estimated.position.x_val += self.waypoints_correction[0]
        self.state.kinematics_estimated.position.y_val += self.waypoints_correction[1]

    def estimateSampleTime(self):
        current_timestamp = self.getCurrentTimestamp()
        if self.compute_sample_time and not self.last_timestamp:
            self.estimated_sample_time = current_timestamp - self.last_timestamp  # compute it using the last timestamp
        else:
            self.estimated_sample_time = self.sample_time  # use the default sample time
        self.last_timestamp = current_timestamp

    def updateCarBehavior(self):
        self.behavior.setCarBehavior()

    def loadWaypointsFromFile(self):
        self.waypoints.loadWaypointsFromFile(filename=self.filename)

    def recordWaypointsToFile(self, sample_time = 0.005):
        self.waypoints.recordWaypointsToFile(self, sample_time, filename = self.filename)

    def saveRaceToFile(self, filename = "lastrace.pickle"):
        self.waypoints.saveRecordedRaceToFile(filename)

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

    def drive(self, speed, track_angle):
        self.updateState()  # update position and other data
        self.updateCarBehavior()  # define the behavior of the car based on conditions
        self.estimateSampleTime()  # update the sample time estimate
        self.behavior.mode = 'CRUZE'

        # Run Speed PID
        self = self.speed_controller.getControlsFromPID(self, speed, self.estimated_sample_time)

        # Run Track Angle PID
        self = self.track_angle_controller.getControlsFromPID(self, track_angle, self.estimated_sample_time)

        self.waypoints.recordRaceWaypoint(self, self.sample_time)

        # Send controls to simulation
        self.setControls()

    def speedy_race(self):
        """
        Run the car faster using alternative PID controllers.
        :return: A flag to still racing.
        """
        self.updateState()  # update position and other data
        self.updateCarBehavior()  # define the behavior of the car based on conditions
        self.estimateSampleTime()  # update the sample time estimate
        #self.behavior.mode = 'CRUZE'

        # Get vehicle state
        current_vehicle_position_x, current_vehicle_position_y = self.getCurrentPostion()
        current_vehicle_speed = self.getCurrentSpeed()
        current_vehicle_track_angle = self.getCurrentTrackAngle()

        # Get next waypoint
        next_waypoint_x, next_waypoint_y, next_waypoint_v, curr_segment_d, curr_segment_a, completed_lap = \
            self.path_planner.get_next_reference_profile_waypoint(current_vehicle_position_x,
                                                                  current_vehicle_position_y,
                                                                  current_vehicle_speed,
                                                                  current_vehicle_track_angle)
        #  print("x: " + str(next_waypoint_x) + " y: "+  str(next_waypoint_y) + " v: " + str(next_waypoint_v))

        # Show profile
        if self.show_profile:
            self.path_planner.show_reference_profile()

        # Compute speed and track angle set points
        speed_set_point, track_angle_set_point = self.pure_pursuit.update_control_targets(current_vehicle_position_x,
                                                                                          current_vehicle_position_y,
                                                                                          current_vehicle_speed,
                                                                                          current_vehicle_track_angle,
                                                                                          next_waypoint_x,
                                                                                          next_waypoint_y,
                                                                                          next_waypoint_v,
                                                                                          curr_segment_d,
                                                                                          curr_segment_a)
        #  print("speed sp: " + str(speed_set_point) + " angle: "+  str(np.rad2deg(track_angle_set_point)) + " [deg]")

        # Run Speed PID
        self = self.speed_controller.getControlsFromPID(self, speed_set_point, self.estimated_sample_time)

        # Run Track Angle PID
        self = self.track_angle_controller.getControlsFromPID(self, track_angle_set_point, self.estimated_sample_time)

        # Send controls to simulation
        self.setControls()

        self.waypoints.recordRaceWaypoint(self, self.sample_time)

        # Show PIDs
        if self.show_pid:
            self.pid_plot.update(self)

        return True

    def getCurrentPostion(self):
        """
        Get the current vehicle position with X-Y coordinates in meters.

        :return: The X and Y coordinates.
        """
        return self.state.kinematics_estimated.position.x_val, self.state.kinematics_estimated.position.y_val

    def getCurrentSpeed(self):
        """
        Get the current velocity magnitude in meters per second.

        :return: The vehicle speed.
        """
        return self.state.speed

    def getCurrentTrackAngle(self):
        """
        Get the current velocity orientation in radians w.r.t. X-axis counter-clockwise.

        :return: The track angle if the speed is greater than a threshold; the vehicle yaw, otherwise.
        """
#        if self.state.speed > self.min_speed:
#            return np.arctan2(self.state.kinematics_estimated.linear_velocity.y_val,
#                              self.state.kinematics_estimated.linear_velocity.x_val)
#        else:
#            (pitch, roll, yaw) = utils.to_eularian_angles(self.state.kinematics_estimated.orientation)
#            return yaw

        (pitch, roll, yaw) = utils.to_eularian_angles(self.state.kinematics_estimated.orientation)
        return yaw

    def getCurrentTimestamp(self):
        """
        Get the current state time stamp in seconds since UNIX epoch with nanoseconds precision.

        :return: The current state time stamp.
        """
        return np.double(self.state.timestamp) / 1e9

