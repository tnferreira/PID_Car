from matplotlib import pyplot as plt
import numpy as np
import math

class Guidance:
    """
    This class implements the guidance law. It computes the steering and speed references based on the current vehicle
    state and the reference trajectory.
    """

    def __init__(self, max_straight_track_speed=20.0, max_curving_speed=7.5, max_turning_rate=5.0, braking_distance=10.0):
        """
        Class constructor.

        :param max_straight_track_speed: maximum allowed velocity magnitude along straight tracks in meter per second
        :param max_curving_speed: maximum allowed velocity magnitude along curves in meters per second
        :param max_turning_rate: maximum allowed turning rate in degrees per second
        :param braking_distance: braking distance to slow down at curves in meters
      """
        self.max_straight_track_speed = max_straight_track_speed
        self.max_curving_speed = max_curving_speed
        self.max_turning_rate = max_turning_rate
        self.braking_distance = braking_distance
        self.last_speed_set_point = 0.0
        self.last_speed_set_point_init = False
        self.speed_set_point_update_rate = 0.1
        self.last_track_angle_set_point = 0.0
        self.last_track_angle_set_point_init = False
        self.track_angle_set_point_update_rate = 0.95
        #self.track_angle_set_point_update_rate = 0.45

        self.min_segment_length = 10.0
        self.max_segment_length = 140.0


    def update_control_targets(self, current_vehicle_position_x, current_vehicle_position_y, current_vehicle_speed,
                               current_vehicle_track_angle, next_waypoint_x, next_waypoint_y, next_waypoint_v,
                               curr_segment_d, curr_segment_a):
        """
        Compute the new speed and track angle control set points according to a modified pure pursuit guidance law.

        :param current_vehicle_position_x: current vehicle position X coordinate
        :param current_vehicle_position_y: current vehicle position X coordinate
        :param current_vehicle_speed: current vehicle velocity magnitude
        :param current_vehicle_track_angle: current vehicle velocity orientation
        :param next_waypoint_x: next waypoint position X coordinate
        :param next_waypoint_y: next waypoint position Y coordinate
        :param next_waypoint_v: next waypoint velocity magnitude
        :param curr_segment_d: current line segment length in meters
        :param curr_segment_a: current line segment orientation in radians
        :return: Updated speed and track angle set points.
        """

        # Displacements along the X and Y axis
        dx = next_waypoint_x - current_vehicle_position_x
        dy = next_waypoint_y - current_vehicle_position_y

        # Distance and angle to the next waypoint
        distance_to_next_waypoint = np.hypot(dx, dy)
        angle_to_next_waypoint = np.arctan2(dy, dx)

        dv = next_waypoint_v - current_vehicle_speed
        da = angle_to_next_waypoint - current_vehicle_track_angle
        while (da > math.pi):
            da -= 2*math.pi
        while (da < -math.pi):
            da += 2*math.pi

        # speed_set_point = next_waypoint_v

        # Simple rule to slow down at curves and go faster at straight tracks
        #if distance_to_next_waypoint > self.braking_distance:
        #    speed_set_point = self.max_straight_track_speed
        #else:
        #    speed_set_point = self.max_curving_speed

        if curr_segment_d <= self.min_segment_length:
            speed_set_point = self.max_curving_speed
        elif curr_segment_d >= self.max_segment_length:
            speed_set_point = self.max_straight_track_speed
        else:
            speed_set_point = (curr_segment_d - self.min_segment_length) / \
                              (self.max_segment_length - self.min_segment_length) * \
                              (self.max_straight_track_speed - self.max_curving_speed) + self.max_curving_speed
        # print("speed sp: " + str(speed_set_point))

        # Speed set point smoother
        if self.last_speed_set_point_init:
            speed_set_point = self.speed_set_point_update_rate * speed_set_point +\
                              (1-self.speed_set_point_update_rate) * self.last_speed_set_point
        self.last_speed_set_point = speed_set_point
        self.last_speed_set_point_init = True

        # Simple rule to align the velocity orientation to the next waypoint
        track_angle_set_point = angle_to_next_waypoint

        # Track angle set point smoother
        if self.last_track_angle_set_point_init:
            track_angle_set_point = self.track_angle_set_point_update_rate * track_angle_set_point +\
                                    (1 - self.track_angle_set_point_update_rate) * self.last_track_angle_set_point
        self.last_track_angle_set_point = track_angle_set_point
        self.last_track_angle_set_point_init = True

        # print("speed sp: " + str(speed_set_point) + " angle: "+  str(np.rad2deg(da)) + " [deg]")
        # if(abs(np.rad2deg(da)) <= 1.0):
        #     speed_set_point = speed_set_point * 1.5
        
        # if(speed_set_point>self.max_straight_track_speed):
        #     speed_set_point = self.max_straight_track_speed

        return speed_set_point, track_angle_set_point
