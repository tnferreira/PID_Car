from garage.pid_car import localization, planning, control
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from rdp import rdp

# Load recorded racing waypoints from file
waypoints_correction = [0, 0]
waypoints = planning.Waypoints("")
waypoints.loadWaypointsFromFile("run-fast4-new.pickle")
waypoints_x, waypoints_y, waypoints_v = waypoints.waypointsToLists(waypoints_correction)


current_vehicle_position_x = waypoints_x[0] + 2
current_vehicle_position_y = waypoints_y[0]
current_vehicle_speed = waypoints_v[0]
current_vehicle_track_angle = 0

path_planner = planning.PathPlanner(epsilon=0.25, sample_time=0.01, number_samples=500, min_distance=6)
path_planner.update_reference_profile_from_recorded_waypoints(waypoints_x, waypoints_y, waypoints_v)
next_waypoint_x, next_waypoint_y, next_waypoint_v, curr_segment_d, curr_segment_a, completed_lap = path_planner.get_next_reference_profile_waypoint(
    current_vehicle_position_x, current_vehicle_position_y, current_vehicle_speed, current_vehicle_track_angle)

path_planner.show_reference_profile(show_full_circuit=True)

