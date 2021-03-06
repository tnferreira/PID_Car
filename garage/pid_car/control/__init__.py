##########################################
### Skoods.org -> Self-Racing Car Team ###
##########################################

import math
from garage.pid_car.control import pid
from skoods import utils
import numpy as np


# Create PIDControl generic class
class PIDControl:
    def __init__(self, car, pid_params, sample_time, limits):
        self.car = car
        self.pid_params = pid_params
        self.sample_time = sample_time
        self.limits = limits
        self.pid_controller = pid.PID(self.__class__.__name__, self.pid_params[0], self.pid_params[1], self.pid_params[2])
        self.pid_controller.setSampleTime(self.sample_time)
        self.initial_search_index = 0
        self.DELTA_INITIAL_SEARCH_INDEX = 300 # Avoid searching all waypoints. We can make it dinamic by getting time stamps.

    def updateNearestIndex(self, car, waypoints_x, waypoints_y):
        # Funtion to return nearest point from waypoints to car position
        length = len(waypoints_x)
        final_search_index = self.initial_search_index + self.DELTA_INITIAL_SEARCH_INDEX
        if final_search_index > length-1:
            final_search_index = -1 # define as last element

        dx = [car.state.kinematics_estimated.position.x_val - iwpx for iwpx in waypoints_x[self.initial_search_index:final_search_index]]  
        dy = [car.state.kinematics_estimated.position.y_val - iwpy for iwpy in waypoints_y[self.initial_search_index:final_search_index]]
        d = [ix ** 2 + iy ** 2 for (ix, iy) in zip(dx, dy)]
        nearest_waypoint_distance = min(d)
        nearest_waypoint_index = d.index(nearest_waypoint_distance)
        nearest_waypoint_index += self.initial_search_index

        # Check end of waypoints index
        if nearest_waypoint_index >= length-2:
            self.initial_search_index = 0
            keep_racing = False
        else:
            keep_racing = True
            self.initial_search_index = nearest_waypoint_index
        
        keep_racing = True # Force

        return nearest_waypoint_index, keep_racing


    def limitOutput(self, output):
        # Limit the output of the controller based on limitations of the car
        if output < self.limits[0]:
            output = self.limits[0]
            self.pid_controller.output = output
        elif output > self.limits[1]:
            output = self.limits[1]
            self.pid_controller.output = output
        return output

    def setSampleTime(self, new_sample_time):
        self.sample_time = new_sample_time
        self.pid_controller.setSampleTime(new_sample_time)


class PIDThrottleControl(PIDControl):
    def __init__(self, car_state, pid_params, sample_time, limits):
        PIDControl.__init__(self, car_state, pid_params, sample_time, limits)

    def setTargetValue(self, car_state, waypoints_x, waypoints_y, waypoints_v):
        # Set the value of the target by returning the speed of the nearest waypoint
        nearest_waypoint_index, keep_racing = self.updateNearestIndex(car_state, waypoints_x, waypoints_y)
        if keep_racing:
            # Avoid low speed set points, specialy during the start
            min_speed = 2.0
            if waypoints_v[nearest_waypoint_index] > min_speed:
                self.pid_controller.SetPoint = waypoints_v[nearest_waypoint_index]
            else:
                self.pid_controller.SetPoint = min_speed
        return keep_racing

    def setTargetValueFixed(self, setPoint):
        self.pid_controller.SetPoint = setPoint
        return True

    def getControlsFromPID(self, car):
        # Return the value of the controls after updating the PID
        self.pid_controller.update(car.state.speed)
        output = self.pid_controller.output
        output = self.limitOutput(output)
        #print("throttle: speed=" + str(car.state.speed) + " output: " + str(output))
        # Define throttle and brake
        if output < 0.0:
            car.controls.throttle = 0.0
            car.controls.brake = output
        if output >= 0.0:
            car.controls.brake = 0.0
            car.controls.throttle = output
        return car


class PIDSteeringControl(PIDControl):
    def __init__(self, car, pid_params, sample_time, limits):
        PIDControl.__init__(self, car, pid_params, sample_time, limits)

    def setTargetValue(self):
        # The target of steering is always 0, and the error is the distance from the car to the nearest point
        self.pid_controller.SetPoint = 0.0
    
    def setTargetValueFixed(self, setPoint):
        self.pid_controller.SetPoint = setPoint
        return True

    def getControlsFromPID(self, car, waypoints_x, waypoints_y):
        # Return the value of the controls after updating the PID
        nearest_waypoint_index, keep_racing = self.updateNearestIndex(car, waypoints_x, waypoints_y)
        if keep_racing:
            delta_error = utils.get_distance_of_point_to_line([car.state.kinematics_estimated.position.x_val, car.state.kinematics_estimated.position.y_val],
                                                        [waypoints_x[nearest_waypoint_index], waypoints_y[nearest_waypoint_index]],
                                                        [waypoints_x[nearest_waypoint_index+1], waypoints_y[nearest_waypoint_index+1]])
            # Get direction of steering (+ or -?)
            # Change coordinate system
            point_A = [waypoints_x[nearest_waypoint_index], waypoints_y[nearest_waypoint_index]]
            point_B = [waypoints_x[nearest_waypoint_index+1], waypoints_y[nearest_waypoint_index+1]]  # define the increment of the index depending on the sampling time.
            point_P = [car.state.kinematics_estimated.position.x_val, car.state.kinematics_estimated.position.y_val]
            # point_A_line = [point_A[0]-point_A[0], point_A[1]-point_A[1]] # This will be the origin (0.0, 0.0)
            point_B_line = [point_B[0]-point_A[0], point_B[1]-point_A[1]]
            point_P_line = [point_P[0]-point_A[0], point_P[1]-point_A[1]]
            # BP cross product
            cross_product_BP = point_B_line[0]*point_P_line[1]-point_P_line[0]*point_B_line[1]
            # Check direction
            if cross_product_BP < 0:
                delta_error *= -1

            # Update PID and set controls
            #print("steering: delta=" + str(delta_error))
            self.pid_controller.update(delta_error)
            output = self.pid_controller.output    #SetPoint defined before, output is calculated in parent class
            output = self.limitOutput(output)
            car.controls.steering = output

        return car, keep_racing


class PIDSpeedControl(PIDControl):
    """"
    This class allows one to control the magnitude of the velocity vector, i.e. car's speed. In particular, it employs
     a PID controller to generate a throttle control signal such that the car's speed will seek a reference value.
    """
    def __init__(self, car, pid_params, sample_time, limits):
        """
        Default constructor.
        :param car: the car object
        :param pid_params: PID parameters
        :param sample_time: default sample time in seconds
        :param limits: PID control signals bounds
        """
        PIDControl.__init__(self, car, pid_params, sample_time, limits)

    def getControlsFromPID(self, car, target_speed, sample_time=[]):
        """
        Compute PID control signals.
        :param car: the car object
        :param target_speed: the speed reference in meters per second
        :param sample_time: the estimated sample time in seconds (optional)
        :return: The update car object with new PID control signals.
        """

        # Set the sample time
        if not sample_time:
            self.setSampleTime(sample_time)

        # Get current speed
        current_speed = car.getCurrentSpeed()
        # print("target_speed: " + str(target_speed) + " [m/s]")
        # print("current_speed: " + str(current_speed) + " [m/s]")

        # Return the value of the controls after updating the PID
        self.pid_controller.SetPoint = target_speed
        self.pid_controller.update(current_speed)
        output = self.pid_controller.output
        output = self.limitOutput(output)

        # Define throttle and brake
        if output < 0.0:
            car.controls.throttle = 0.0
            car.controls.brake = output
        if output >= 0.0:
            car.controls.brake = 0.0
            car.controls.throttle = output
        return car


class PIDTrackAngleControl(PIDControl):
    """"
    This class allows one to control the orientation of the velocity vector, i.e. track angle. In particular, it employs
     a PID controller to generate a steering control signal such that the car's track angle will seek a reference value.
    """
    def __init__(self, car, pid_params, sample_time, limits):
        """
        Default constructor.
        :param car: the car object
        :param pid_params: PID parameters
        :param sample_time: default sample time in seconds
        :param limits: PID control signals bounds
        """
        PIDControl.__init__(self, car, pid_params, sample_time, limits)

    def getControlsFromPID(self, car, target_track_angle, sample_time=[]):
        """
        Compute PID control signals.
        :param car: the car object
        :param target_track_angle: the track angle reference in radians
        :param sample_time: the estimated sample time in seconds (optional)
        :return: The update car object with new PID control signals.
        """

        # Set the sample time
        if not sample_time:
            self.setSampleTime(sample_time)

        # Get current track angle
        current_track_angle = car.getCurrentTrackAngle()

        # Compute the track angle error properly
        track_angle_error = target_track_angle - current_track_angle
        while track_angle_error > math.pi:
            track_angle_error -= 2*math.pi
        while track_angle_error < -math.pi:
            track_angle_error += 2*math.pi
        # track_angle_error = np.unwrap([target_track_angle - current_track_angle])[0]
        # print("target_track_angle: " + str(np.rad2deg(target_track_angle)) + " [deg]")
        # print("current_track_angle: " + str(np.rad2deg(current_track_angle)) + " [deg]")
        # print("track_angle_error: " + str(np.rad2deg(track_angle)) + " [deg]")

        # Update PID and set controls
        self.pid_controller.SetPoint = 0.0  # Actually, the set point is not zero
        self.pid_controller.update(-track_angle_error)  # The difference between the current and the target track angle
        # is plugged here
        output = self.pid_controller.output
        output = self.limitOutput(output)
        car.controls.steering = output
        self.pid_controller.SetPoint = target_track_angle  # Store the target track angle
        return car
