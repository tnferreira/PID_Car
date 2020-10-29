from garage.pid_car import localization, planning, control
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from rdp import rdp
import csv
import os.path


def save_csv(filename, array_x, array_y, array_z):
  with open(filename, mode='w') as datafile:
    writer = csv.writer(datafile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    writer.writerow(['id', 'x', 'y', 'v'])
    for i in range(len(array_x)):
      writer.writerow([i, array_x[i], array_y[i], array_z[i]])

# Load recorded racing waypoints from file
waypoints_correction = [0, 0]
waypoints = planning.Waypoints("")
waypoints_left = planning.Waypoints("")
waypoints_right = planning.Waypoints("")

#waypoints.loadWaypointsFromFile("last-race.pickle")
waypoints.loadWaypointsFromFile("last-race-1.pickle")
#waypoints.loadWaypointsFromFile("run-fast4-old.pickle")
waypoints_left.loadWaypointsFromFile("run-fast4-borda-esquerda.pickle")
waypoints_right.loadWaypointsFromFile("run-fast4-borda-direita.pickle")

waypoints_x_right, waypoints_y_right,  waypoints_v_right = waypoints_right.waypointsToLists(waypoints_correction)
waypoints_x_left, waypoints_y_left, waypoints_v_left = waypoints_left.waypointsToLists(waypoints_correction)
waypoints_race_x, waypoints_race_y, waypoints_race_v = waypoints.waypointsToLists(waypoints_correction)

#Try to load updated waypoints ("id, x, y, v, newx, newy")
fname = 'race-updated.csv'
if os.path.isfile(fname):
  race_x = []
  race_y = []
  race_v = []

  with open(fname) as csv_file:
      csv_reader = csv.reader(csv_file, delimiter=',')
      line_count = 0
      for row in csv_reader:
          if line_count == 0:
              print(f'Column names are {", ".join(row)}')
          else:
              print(f'\t id: {row[0]} x: {row[1]} y:{row[2]} v:{row[3]} newx:{row[5]} newy:{row[6]}')
              race_x.append(row[5])
              race_y.append(row[6])
              race_v.append(row[3])
          line_count += 1
      print(f'Processed {line_count} lines.')
  print(str(len(race_x)) + " points")

  reference_profile_waypoints_x = np.array(race_x, dtype=np.float32)
  reference_profile_waypoints_y = np.array(race_y, dtype=np.float32)
  reference_profile_waypoints_v = np.array(race_v, dtype=np.float32)
  #print(reference_profile_waypoints_x)
  #exit()

else:
  path_planner = planning.PathPlanner(epsilon=0.25, sample_time=0.1, number_samples=500, min_distance=6)
  path_planner.update_reference_profile_from_recorded_waypoints(waypoints_race_x, waypoints_race_y, waypoints_race_v)
  reference_profile_waypoints_x, reference_profile_waypoints_y, reference_profile_waypoints_v = path_planner.getReferenceProfile()
  #print(reference_profile_waypoints_x)
  #exit()
  #Save csv
  #save_csv('waypoints_x_right.csv', waypoints_x_right, waypoints_y_right, waypoints_v_right)
  #save_csv('waypoints_x_left.csv', waypoints_x_left, waypoints_y_left, waypoints_v_left)
  save_csv('race.csv', reference_profile_waypoints_x, reference_profile_waypoints_y, reference_profile_waypoints_v)


"""
Plot the reference profile along the recorded points; indicates the next waypoint and the vehicle trajectory.
:return: None
"""
# Get recorded points as line segments
points_race = np.array([waypoints_race_x, waypoints_race_y ]).T.reshape(-1, 1, 2)
#points_left = np.array([waypoints_x_left, waypoint_y_left ]).T.reshape(-1, 1, 2)
#points_right = np.array([waypoints_x_right, waypoint_y_right ]).T.reshape(-1, 1, 2)

segments_race = np.concatenate([points_race[:-1], points_race[1:]], axis=1)

waypoints_race_x = np.array(waypoints_race_x)
waypoints_race_y = np.array(waypoints_race_y)
waypoints_race_v = np.array(waypoints_race_v)

waypoints_x_left = np.array(waypoints_x_left)
waypoints_y_left = np.array(waypoints_y_left)

waypoints_x_right = np.array(waypoints_x_right)
waypoints_y_right = np.array(waypoints_y_right)

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
axs.set_xlim(waypoints_x_left.min(), waypoints_x_left.max())
axs.set_ylim(waypoints_y_left.min(), waypoints_y_left.max())

axs.set_xlabel('x [m]')
axs.set_ylabel('y [m]')

axs.set_aspect('equal', 'box')
axs.invert_yaxis()

axs.plot(reference_profile_waypoints_x, reference_profile_waypoints_y, 'k--', marker='o', markersize=4)
axs.plot(waypoints_x_left, waypoints_y_left, 'k-')
axs.plot(waypoints_x_right, waypoints_y_right, 'k-')

# Adds point index to the graphic
sp=0.05
idx = 0
for i,j in zip(reference_profile_waypoints_x, reference_profile_waypoints_y):
      axs.annotate(str(idx), xy=(i+sp,j+sp))
      idx += 1


plt.show()


