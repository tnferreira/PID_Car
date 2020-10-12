from garage.pid_car import localization, planning, control
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from rdp import rdp

waypoints_correction = [0, 0]
waypoints = planning.Waypoints("")
waypoints.loadWaypointsFromFile("run-fast4.pickle")
waypoints_x, waypoints_y, waypoints_v = waypoints.waypointsToLists(waypoints_correction)

x = np.array(waypoints_x)
y = np.array(waypoints_y)
v = np.array(waypoints_v)

xy = np.vstack((x, y)).T
reducedXY = rdp(xy, epsilon=1)
reducedX = reducedXY[:, 0]
reducedY = reducedXY[:, 1]

n = v.size
sample_time = 0.01
tmin = 0
tmax = (n-1)*sample_time
t = np.linspace(tmin, tmax, num=n)


#fig, ax = plt.subplots()
#ax.plot(x, y)
#plt.show()

# Create a set of line segments so that we can color them individually
# This creates the points as a N x 1 x 2 array so that we can stack points
# together easily to get the segments. The segments array for line collection
# needs to be (numlines) x (points per line) x 2 (for x and y)
points = np.array([x, y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

fig, axs = plt.subplots(2, 1)

# Create a continuous norm to map from data points to colors
norm = plt.Normalize(v.min(), v.max())
lc = LineCollection(segments, cmap='jet', norm=norm)
# Set the values used for colormapping
lc.set_array(v)
lc.set_linewidth(6)
line = axs[0].add_collection(lc)
fig.colorbar(line, ax=axs[0])
axs[0].set_xlim(x.min(), x.max())
axs[0].set_ylim(y.min(), y.max())
axs[0].set_xlabel('x [m]')
axs[0].set_ylabel('y [m]')

axs[0].plot(reducedX,reducedY, 'k--', marker='o')

# Use a boundary norm instead
axs[1].plot(t,v)
axs[1].set_xlim(t.min(), t.max())
axs[1].set_ylim(v.min(), v.max())
axs[1].set_xlabel('t [s]')
axs[1].set_ylabel('v [m/s]')

plt.show()