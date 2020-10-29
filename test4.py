from garage.pid_car import localization, planning, control
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from rdp import rdp

# Load recorded racing waypoints from file
waypoints_correction = [0, 0]
waypoints = planning.Waypoints("")

waypoint_letf = planning.Waypoints("")
waypoint_right = planning.Waypoints("")

#waypoints.loadWaypointsFromFile("last-race.pickle")
waypoints.loadWaypointsFromFile("run-fast4-old.pickle")
waypoint_letf.loadWaypointsFromFile("run-fast4-borda-esquerda.pickle")
waypoint_right.loadWaypointsFromFile("run-fast4-borda-direita.pickle")

waypoints_x_right, waypoint_y_right,  waypoint_v_right=  waypoint_right.waypointsToLists(waypoints_correction)
waypoints_x_left, waypoint_y_left, waypoint_v_left =  waypoint_letf.waypointsToLists(waypoints_correction)

waypoints_race_x, waypoints_race_y, waypoints_race_v = waypoints.waypointsToLists(waypoints_correction)


"""
Plot the reference profile along the recorded points; indicates the next waypoint and the vehicle trajectory.
:return: None
"""
# Get recorded points as line segments
points_race = np.array([waypoints_race_x, waypoints_race_y ]).T.reshape(-1, 1, 2)
#points_left = np.array([waypoints_x_left, waypoint_y_left ]).T.reshape(-1, 1, 2)
#points_right = np.array([waypoints_x_right, waypoint_y_right ]).T.reshape(-1, 1, 2)

segments_race = np.concatenate([points_race[:-1], points_race[1:]], axis=1)

waypoints_race_v = np.array(waypoints_race_v)
waypoints_race_x = np.array(waypoints_race_x)
waypoints_race_y = np.array(waypoints_race_y)

waypoint_y_left = np.array(waypoint_y_left)
waypoints_x_left = np.array(waypoints_x_left)

waypoint_y_left = np.array(waypoint_y_left)
waypoint_y_right = np.array(waypoint_y_right)

# Create two axis

fig, axs = plt.subplots(1, 1)

# Create a continuous norm to map from data points to colors
norm = plt.Normalize(waypoints_race_v.min(), waypoints_race_v.max())
lc = LineCollection(segments_race, cmap='jet', norm=norm)

# Set the values used for color mapping
lc.set_array(waypoints_race_v)
lc.set_linewidth(6)
# Add recorded waypoints
line = axs.add_collection(lc)
fig.colorbar(line, ax=axs)

# Plot recorded position profile

axs.set_xlim(waypoints_race_x.min(), waypoints_race_x.max())
axs.set_ylim(waypoints_race_y.min(), waypoints_race_y.max())

axs.set_xlabel('x [m]')
axs.set_ylabel('y [m]')

axs.plot(waypoints_x_left, waypoint_y_left, 'k--')
axs.plot(waypoints_x_right, waypoint_y_right, 'k--')

plt.show()


