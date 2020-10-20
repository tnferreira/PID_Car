from matplotlib import pyplot as plt
import numpy as np

class Guidance:
    """
    This class implements the guidance law. It computes the steering and speed references based on the current vehicle
    state and the reference trajectory.
    """

    def __init__(self, max_straight_track_speed=20.0, max_curving_speed=15.0, max_turning_rate=5.0):
        """
        Class constructor.

        :param max_straight_track_speed: maximum allowed velocity magnitude along straight tracks in meter per second
        :param max_curving_speed: maximum allowed velocity magnitude along curves in meters per second
        :param max_turning_rate: maximum allowed turning rate in degrees per second
       """
        self.max_straight_track_speed = max_straight_track_speed
        self.max_curving_speed = max_curving_speed
        self.max_turning_rate = max_turning_rate

    def update_control_targets(self, current_vehicle_position_x, current_vehicle_position_y, current_vehicle_speed,
                               current_vehicle_track_angle, next_waypoint_x, next_waypoint_y, next_waypoint_v):
        """
        Compute the new speed and track angle control set points according to a modified pure pursuit guidance law.

        :param current_vehicle_position_x: current vehicle position X coordinate
        :param current_vehicle_position_y: current vehicle position X coordinate
        :param current_vehicle_speed: current vehicle velocity magnitude
        :param current_vehicle_track_angle: current vehicle velocity orientation
        :param next_waypoint_x: next waypoint position X coordinate
        :param next_waypoint_y: next waypoint position Y coordinate
        :param next_waypoint_v: next waypoint velocity magnitude
        :return: Updated speed and track angle set points.
        """

        # Displacements along the X and Y axis
        dx = next_waypoint_x - current_vehicle_position_x
        dy = next_waypoint_y - current_vehicle_position_y

        # Distance and angle to the next waypoint
        distance_to_next_waypoint = np.hypot(dx, dy)
        angle_to_next_waypoint = np.arctan2(dy, dx)

        dv = next_waypoint_v - current_vehicle_speed
        da = np.unwrap([angle_to_next_waypoint - current_vehicle_track_angle])[0]

        speed_set_point = next_waypoint_v
        track_angle_set_point = angle_to_next_waypoint

        return speed_set_point, track_angle_set_point
