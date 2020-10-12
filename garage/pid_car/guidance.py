from matplotlib import pyplot as plt
import numpy as np

class Guidance:
    """
    This class implements the guidance law. It computes the steering and speed references based on the current vehicle
    state and the reference trajectory.
    """

    def __init__(self, name="", max_straight_track_speed=40.0, max_curving_speed=15.0):
        """
        Class constructor.

        :param name: object name for logging purposes
        :param max_straight_track_speed: maximum allowed velocity magnitude along straight tracks
        :param max_curving_speed: maximum allowed velocity magnitude along curves
        """
        self.name = name
        self.max_straight_track_speed = max_straight_track_speed
        self.max_curving_speed = max_curving_speed
        self.speed_reference = 0.0
        self.steering_reference = 0.0
        self.initial_search_index = 0
        self.DELTA_INITIAL_SEARCH_INDEX = 300 # Avoid searching all waypoints. We can make it dinamic by getting time stamps.

    def updateNearestIndex(self, car, waypoints_x, waypoints_y):
        """
        Function to return nearest point from waypoints to car position.

        :param car: car object containing all parameters and states
        :param waypoints_x: array containing the X axis coordinates of all waypoints along the reference trajectory
        :param waypoints_y: array containing the Y axis coordinates of all waypoints along the reference trajectory
        :return: index of the nearest waypoint and a flag indicating whether the trajectory end was reached or not
        """
        length = len(waypoints_x)
        final_search_index = self.initial_search_index + self.DELTA_INITIAL_SEARCH_INDEX
        if final_search_index > length - 1:
            final_search_index = -1  # define as last element

        dx = [car.state.kinematics_estimated.position.x_val - iwpx for iwpx in
              waypoints_x[self.initial_search_index:final_search_index]]
        dy = [car.state.kinematics_estimated.position.y_val - iwpy for iwpy in
              waypoints_y[self.initial_search_index:final_search_index]]
        d = [ix ** 2 + iy ** 2 for (ix, iy) in zip(dx, dy)]
        nearest_waypoint_distance = min(d)
        nearest_waypoint_index = d.index(nearest_waypoint_distance)
        nearest_waypoint_index += self.initial_search_index

        # Check end of waypoints index
        if nearest_waypoint_index >= length - 2:
            self.initial_search_index = 0
            keep_racing = False
        else:
            keep_racing = True
            self.initial_search_index = nearest_waypoint_index

        keep_racing = True  # Force

        return nearest_waypoint_index, keep_racing

    def updateTargets(self, car, waypoints_x, waypoints_y, waypoints_v):
        """
        Update the speed and steering references for the PID controllers.

        :param car: car object containing all parameters and states
        :param waypoints_x: array containing the X axis coordinates of all waypoints along the reference trajectory
        :param waypoints_y: array containing the Y axis coordinates of all waypoints along the reference trajectory
        :param waypoints_v: array containing the velocity magnitude of all waypoints along the reference trajectory
        :return: the new speed and steering references
        """
        # Set the value of the target by returning the speed of the nearest waypoint
        nearest_waypoint_index, keep_racing = self.updateNearestIndex(car, waypoints_x, waypoints_y)
        if keep_racing:
            # Avoid low speed set points, specialy during the start
            if waypoints_v[nearest_waypoint_index] > 2.0:
                self.pid_controller.SetPoint = waypoints_v[nearest_waypoint_index]
            else:
                self.pid_controller.SetPoint = 2.0

        return keep_racing