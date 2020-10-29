##########################################
### Skoods.org -> Self-Racing Car Team ###
##########################################

import os, time, pickle, airsim, math
from pynput import keyboard # Run $pip install pynput in the Anaconda prompt
from skoods import utils
import numpy as np
from rdp import rdp
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


class Waypoints:
    def __init__(self, car_name):
        self.waypoints_list = []
        self.last_race_wps = []
        self.current_time = time.time()
        self.last_time = self.current_time
        self.past_x_val = 0.0
        self.past_y_val = 0.0
        self.car_name = car_name

    def recordRaceWaypoint(self, car, sample_time = 0.005):
        # Get and append waypoint after reaching sample time
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        car_state = car.client.getCarState(car.name)
        x_val = car_state.kinematics_estimated.position.x_val
        y_val = car_state.kinematics_estimated.position.y_val
        # Record waypoints only 10cm of distance from past waypoint, otherwise can return same waypoint. You can change if you want to.
        if utils.distance_of_two_points(x_val, y_val, self.past_x_val, self.past_y_val) > 0.1:
            distance_bool = True
        else:
            distance_bool = False
        # also take in consideration sample time
        if ((delta_time >= sample_time) and distance_bool == True):
            self.last_race_wps.append(car_state)
            self.last_time = self.current_time
        self.past_x_val = x_val
        self.past_y_val = y_val

    def saveRecordedRaceToFile(self, filename):
        print (self.car_name + " || LAST RACE: Saving waypoints to pickle file.")
        with open(os.path.join(filename),"wb") as f:
            pickle.dump(self.last_race_wps, f)
        print(self.car_name + " || LAST RACE: Success! %d waypoints saved to disk." % (len(self.last_race_wps)))

    def getCurrentWaypoint(self, car, sample_time):
        # Get and append waypoint after reaching sample time
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        car_state = car.client.getCarState(car.name)
        x_val = car_state.kinematics_estimated.position.x_val
        y_val = car_state.kinematics_estimated.position.y_val
        # Record waypoints only 10cm of distance from past waypoint, otherwise can return same waypoint. You can change if you want to.
        if utils.distance_of_two_points(x_val, y_val, self.past_x_val, self.past_y_val) > 0.1:
            distance_bool = True
        else:
            distance_bool = False
        # also take in consideration sample time
        if ((delta_time >= sample_time) and distance_bool == True):
            self.waypoints_list.append(car_state)
            self.last_time = self.current_time
        self.past_x_val = x_val
        self.past_y_val = y_val

    def saveWaypointsToFile(self, filename):
        print (self.car_name + " || WAYPOINTS: Saving waypoints to pickle file.")
        with open(os.path.join(filename),"wb") as f:
            pickle.dump(self.waypoints_list, f)
        print(self.car_name + " || WAYPOINTS: Success! %d waypoints saved to disk." % (len(self.waypoints_list)))
 
    def recordWaypointsToFile(self, car, sample_time, filename):
        # Press END key to complete the recording
        break_program = False
        print(self.car_name + " || PLANNING: Press END to save waypoints to file after driving.")
        def on_press(key):
            global break_program
            # print (key)
            if key == keyboard.Key.end:
                break_program = True
                # After pressing end, run...
                self.saveWaypointsToFile(filename)
                return False
        with keyboard.Listener(on_press=on_press) as listener:
            while break_program == False:
                # Before pressing end, run...
                self.getCurrentWaypoint(car, sample_time)
            listener.join()

    def loadWaypointsFromFile(self, filename):
        with open(os.path.join(filename), "rb") as f:
            self.waypoints_list = pickle.load(f)
        print(self.car_name + " || WAYPOINTS: Success! %d waypoints loaded from disk." % (len(self.waypoints_list)))

    def waypointsToLists(self, waypoints_correction):
        # Waypoints are objects. Here we are transforming them to lists.
        waypoints_x, waypoints_y, waypoints_v = [], [], []
        # FUTURE: waypoints_x, waypoints_y, waypoints_yaw, waypoints_v = [], [], [], []
        for each_waypoint in self.waypoints_list:
            waypoints_x.append(each_waypoint.kinematics_estimated.position.x_val) # Must apply correction when racing with more then one car because of initial position change for the grid.
            waypoints_y.append(each_waypoint.kinematics_estimated.position.y_val)
            # FUTURE: _, _, yaw_val = airsim.utils.to_eularian_angles(each_waypoint.kinematics_estimated.orientation) # NED coord
            # FUTURE: waypoints_yaw.append(yaw_val)
            waypoints_v.append(each_waypoint.speed) # m/s
        return waypoints_x, waypoints_y, waypoints_v
        # FUTURE: return waypoints_x, waypoints_y, waypoints_yaw, waypoints_v


class Behavior:
    def __init__(self, car):
        self.car = car
        self.mode = 'START'
        print(self.car.name + " || BEHAVIOR: Mode = START")

    def setCarBehavior(self):
        # You can create more modes here
        x_val = self.car.state.kinematics_estimated.position.x_val
        y_val = self.car.state.kinematics_estimated.position.y_val
        dist_from_start_point = math.sqrt(x_val**2 + y_val**2)
        if dist_from_start_point > 1.0 and self.mode != 'CRUZE':
            self.mode = 'CRUZE'
            print(self.car.name + " || BEHAVIOR: Mode = CRUZE")


class PathPlanner:
    """"
    This class implements a very straightforward path planning method. It builds a reference position & velocity profile
    by approximating the recorded trajectory by multiple line segments and provides a mean to select the next waypoint
    along this profile according to the current vehicle position.
    """
    def __init__(self, epsilon=1, sample_time=0.01, number_samples=500, min_distance=3):
        """
        Default constructor.

        :param epsilon: maximum deviation in meters of recorded waypoints from the approximated line segments
        :param sample_time: sampling time in seconds to employed to record the waypoints
        :param number_samples: number of vehicle state samples to keep (presentation only)
        :param min_distance: minimum distance to determine if a waypoint along the reference profile was reached
        """
        # Parameters
        self.epsilon = epsilon
        self.sample_time = sample_time
        self.number_samples = number_samples
        self.min_distance = min_distance

        # Recorded waypoints are stored for presentation purposes
        self.recorded_waypoints_x = []
        self.recorded_waypoints_y = []
        self.recorded_waypoints_v = []
        self.recorded_waypoints_t = []

        # Vehicle states are stored for presentation purposes
        self.vehicle_position_x = np.zeros(number_samples)
        self.vehicle_position_y = np.zeros(number_samples)
        self.vehicle_speed = np.zeros(number_samples)
        self.vehicle_track_angle = np.zeros(number_samples)

        # Initialize the reference profile
        self.reference_profile_waypoints_x = []
        self.reference_profile_waypoints_y = []
        self.reference_profile_waypoints_v = []
        self.reference_profile_waypoints_d = []
        self.reference_profile_waypoints_a = []

        # Initialize auxiliary indexes
        self.initial_search_index = 0
        self.last_waypoint_index = 0
        
        self.fig = []
        self.axs = []
        self.lc = []


    def load_reference_profile(self, race_csv = "race.csv"):
        """
         Build the reference profile using the CSV recorded waypoints position and speed

        :param race_csv: csv file with ("id, x, y, v, newx, newy")
        :return: None
        """

        if os.path.isfile(race_csv):
            race_x = []
            race_y = []
            race_v = []

            with open(race_csv) as csv_file:
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
            # Set the reference profile
            self.reference_profile_waypoints_x = np.array(race_x, dtype=np.float32)
            self.reference_profile_waypoints_y = np.array(race_y, dtype=np.float32)
            self.reference_profile_waypoints_v = np.array(race_v, dtype=np.float32)

            # Set the reference profile
            self.reference_profile_waypoints_x = self.recorded_waypoints_x[mask]
            self.reference_profile_waypoints_y = self.recorded_waypoints_y[mask]
            self.reference_profile_waypoints_v = self.recorded_waypoints_v[mask]

            # Compute displacements between consecutive waypoints
            reference_profile_waypoints_dx = self.reference_profile_waypoints_x - \
                                            np.roll(self.reference_profile_waypoints_x, 1)
            reference_profile_waypoints_dy = self.reference_profile_waypoints_y - \
                                            np.roll(self.reference_profile_waypoints_y, 1)

            # Compute line segments lengths and orientations
            self.reference_profile_waypoints_d = np.hypot(reference_profile_waypoints_dx, reference_profile_waypoints_dy)
            self.reference_profile_waypoints_a = np.arctan2(reference_profile_waypoints_dy, reference_profile_waypoints_dx)
            self.reference_profile_waypoints_d[0] = 100.0
            self.reference_profile_waypoints_a[0] = -math.pi
            # Reset auxiliary indexes
            self.initial_search_index = 0
            self.last_waypoint_index = 0

    def update_reference_profile_from_recorded_waypoints(self, recorded_waypoints_x, recorded_waypoints_y,
                                                         recorded_waypoints_v):
        """
        Build the reference profile using the recorded waypoints position and speed.

        :param recorded_waypoints_x: recorded waypoints position X coordinates
        :param recorded_waypoints_y: recorded waypoints position Y coordinates
        :param recorded_waypoints_v: recorded waypoints velocity magnitude
        :return: None
        """
        # Get recorded waypoints' positions and speeds
        self.recorded_waypoints_x = np.array(recorded_waypoints_x)
        self.recorded_waypoints_y = np.array(recorded_waypoints_y)
        self.recorded_waypoints_v = np.array(recorded_waypoints_v)

        n = self.recorded_waypoints_v.size
        tmin = 0
        tmax = (n - 1) * self.sample_time
        self.recorded_waypoints_t = np.linspace(tmin, tmax, num=n)

        # Approximate the waypoints by line segments
        xy = np.vstack((self.recorded_waypoints_x, self.recorded_waypoints_y)).T
        mask = rdp(xy, self.epsilon, return_mask=True)
        mask[-1] = False

        # Set the reference profile
        self.reference_profile_waypoints_x = self.recorded_waypoints_x[mask]
        self.reference_profile_waypoints_y = self.recorded_waypoints_y[mask]
        self.reference_profile_waypoints_v = self.recorded_waypoints_v[mask]

        # Compute displacements between consecutive waypoints
        reference_profile_waypoints_dx = self.reference_profile_waypoints_x - \
                                         np.roll(self.reference_profile_waypoints_x, 1)
        reference_profile_waypoints_dy = self.reference_profile_waypoints_y - \
                                         np.roll(self.reference_profile_waypoints_y, 1)

        # Compute line segments lengths and orientations
        self.reference_profile_waypoints_d = np.hypot(reference_profile_waypoints_dx, reference_profile_waypoints_dy)
        self.reference_profile_waypoints_a = np.arctan2(reference_profile_waypoints_dy, reference_profile_waypoints_dx)
        self.reference_profile_waypoints_d[0] = 100.0
        self.reference_profile_waypoints_a[0] = -math.pi

        ############ NEW PLANNING FILTER
        enable_filter = False
        angle_diff_discard = 2.0
        if (enable_filter):
            n = self.reference_profile_waypoints_a.size
            mask2 = np.array([], dtype=bool)
            idx = 0
            print("Apply angle filter on :" + str(n))
            time.sleep(2)
            while(idx < n):
                next_idx = idx+1
                if next_idx >= len(self.reference_profile_waypoints_a):
                    next_idx = 0
                curr_segment_a = self.reference_profile_waypoints_a[idx]
                next2_segment_a = self.reference_profile_waypoints_a[next_idx]

                da = curr_segment_a - next2_segment_a
                # Distance and angle to the next waypoint
                while (da > math.pi):
                    da -= 2*math.pi
                while (da < -math.pi):
                    da += 2*math.pi

                # Check the angle between two segments and remove point if the angle is minimum
                angle_in_deg = np.rad2deg(da)
                #print( str(idx) + " to next segment diff: " + str(angle_in_deg))
                if (abs(angle_in_deg) <= angle_diff_discard):
                    mask2 = np.append(mask2, False)
                    #print("Pop waypoint index: " + str(idx))
                else:
                    mask2 = np.append(mask2, True)
                idx+=1

            # Set the reference profile
            self.reference_profile_waypoints_x = self.reference_profile_waypoints_x[mask2]
            self.reference_profile_waypoints_y = self.reference_profile_waypoints_y[mask2]
            self.reference_profile_waypoints_v = self.reference_profile_waypoints_v[mask2]
            self.reference_profile_waypoints_a = self.reference_profile_waypoints_a[mask2]
            self.reference_profile_waypoints_d = self.reference_profile_waypoints_d[mask2]
            #print (self.reference_profile_waypoints_v)
            length = len(self.reference_profile_waypoints_a)
            print("New set waypoints:" + str(length) + " - Filtered: " + str(n - length))

             # Compute AGAIN displacements between consecutive waypoints
            reference_profile_waypoints_dx = self.reference_profile_waypoints_x - \
                                            np.roll(self.reference_profile_waypoints_x, 1)
            reference_profile_waypoints_dy = self.reference_profile_waypoints_y - \
                                            np.roll(self.reference_profile_waypoints_y, 1)

            # Compute AGAIN line segments lengths and orientations
            self.reference_profile_waypoints_d = np.hypot(reference_profile_waypoints_dx, reference_profile_waypoints_dy)
            self.reference_profile_waypoints_a = np.arctan2(reference_profile_waypoints_dy, reference_profile_waypoints_dx)
            self.reference_profile_waypoints_d[0] = 100.0
            self.reference_profile_waypoints_a[0] = -math.pi
        # END NEW PLANNING FILTER

        # Reset auxiliary indexes
        self.initial_search_index = 0
        self.last_waypoint_index = 0

    def get_next_reference_profile_waypoint(self, current_vehicle_position_x, current_vehicle_position_y,
                                            current_vehicle_speed, current_vehicle_track_angle):
        """
        Select the next waypoint along the reference profile.

        :param current_vehicle_position_x: current vehicle position X coordinate
        :param current_vehicle_position_y: current vehicle position Y coordinate
        :param current_vehicle_speed: current vehicle velocity magnitude
        :param current_vehicle_track_angle: current vehicle velocity orientation
        :return: Next waypoint position and velocity; current segment length and orientation;
         a flag indicating if a new lap was completed
        """
        length = len(self.reference_profile_waypoints_v)
        final_search_index = self.initial_search_index + 20 # define as last element
        if final_search_index > length-1:
            final_search_index = -1

        # Roll buffers
        self.vehicle_position_x = np.roll(self.vehicle_position_x, -1)
        self.vehicle_position_y = np.roll(self.vehicle_position_y, -1)
        self.vehicle_speed = np.roll(self.vehicle_speed, -1)
        self.vehicle_track_angle = np.roll(self.vehicle_track_angle, -1)

        # Fill buffers
        self.vehicle_position_x[-1] = current_vehicle_position_x
        self.vehicle_position_y[-1] = current_vehicle_position_y
        self.vehicle_speed[-1] = current_vehicle_speed
        self.vehicle_track_angle[-1] = current_vehicle_track_angle

        # Compute squared distances to all waypoint from the initial search index to the last one
        dx = [current_vehicle_position_x - wpx for wpx in self.reference_profile_waypoints_x[self.initial_search_index:final_search_index]]
        dy = [current_vehicle_position_y - wpy for wpy in self.reference_profile_waypoints_y[self.initial_search_index:final_search_index]]
        d = [ix ** 2 + iy ** 2 for (ix, iy) in zip(dx, dy)]

        # Get the index to the closest one ahead
        nearest_waypoint_distance = min(d)
        nearest_waypoint_index = self.initial_search_index + d.index(nearest_waypoint_distance)

        # Check whether the selected waypoint was reached or not
        if nearest_waypoint_distance < self.min_distance ** 2:
            if nearest_waypoint_index >= length - 1:
                nearest_waypoint_index = 0
            else:
                nearest_waypoint_index += 1

        # Check end of waypoints index
        if nearest_waypoint_index >= length - 2:
            self.initial_search_index = 0
        else:
            self.initial_search_index = nearest_waypoint_index

        # Check whether the lap was completed or not
        completed_lap = self.last_waypoint_index > nearest_waypoint_index
        self.last_waypoint_index = nearest_waypoint_index

        #  print("nearest_waypoint_index: " + str(nearest_waypoint_index))
        #  print("last_waypoint_index: " + str(self.last_waypoint_index))
        #  print("initial_search_index: " + str(self.initial_search_index))

        # Get selected waypoint position and speed
        next_waypoint_x = self.reference_profile_waypoints_x[nearest_waypoint_index]
        next_waypoint_y = self.reference_profile_waypoints_y[nearest_waypoint_index]
        next_waypoint_v = self.reference_profile_waypoints_v[nearest_waypoint_index]

        curr_segment_d = self.reference_profile_waypoints_d[nearest_waypoint_index]
        curr_segment_a = self.reference_profile_waypoints_a[nearest_waypoint_index]

        return next_waypoint_x, next_waypoint_y, next_waypoint_v, curr_segment_d, curr_segment_a, completed_lap

    def show_reference_profile(self, show_full_circuit=False):
        """
        Plot the reference profile along the recorded points; indicates the next waypoint and the vehicle trajectory.
        :return: None
        """
        # Get recorded points as line segments
        points = np.array([self.recorded_waypoints_x, self.recorded_waypoints_y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        # Create two axis
        if self.fig == []:
            self.fig, self.axs = plt.subplots(1, 1)
            
            # Create a continuous norm to map from data points to colors
            norm = plt.Normalize(self.recorded_waypoints_v.min(), self.recorded_waypoints_v.max())
            self.lc = LineCollection(segments, cmap='jet', norm=norm)

            # Set the values used for color mapping
            self.lc.set_array(self.recorded_waypoints_v)
            self.lc.set_linewidth(6)
            # Add recorded waypoints
            line = self.axs.add_collection(self.lc)
            self.fig.colorbar(line, ax=self.axs)
        else:
            plt.figure(self.fig.number)
            plt.clf()
            plt.axes(self.axs)

        # Get selected waypoint position and speed
        last_waypoint_x = self.reference_profile_waypoints_x[self.last_waypoint_index]
        last_waypoint_y = self.reference_profile_waypoints_y[self.last_waypoint_index]
        last_waypoint_v = self.reference_profile_waypoints_v[self.last_waypoint_index]

        # Get last vehicle position and speed
        last_vehicle_position_x = self.vehicle_position_x[-1]
        last_vehicle_position_y = self.vehicle_position_y[-1]
        last_vehicle_speed = self.vehicle_speed[-1]
        last_vehicle_track_angle = self.vehicle_track_angle[-1]

        # Plot recorded position profile
        if show_full_circuit:
            self.axs.set_xlim(self.recorded_waypoints_x.min(), self.recorded_waypoints_x.max())
            self.axs.set_ylim(self.recorded_waypoints_y.min(), self.recorded_waypoints_y.max())
        else:
            dist = 100
            self.axs.set_xlim(last_vehicle_position_x-dist, last_vehicle_position_x+dist)
            self.axs.set_ylim(last_vehicle_position_y-dist, last_vehicle_position_y+dist)

        self.axs.set_xlabel('x [m]')
        self.axs.set_ylabel('y [m]')
        self.axs.plot(self.reference_profile_waypoints_x, self.reference_profile_waypoints_y, 'k--', marker='o', markersize=4)
        self.axs.plot([last_vehicle_position_x, last_waypoint_x], [last_vehicle_position_y, last_waypoint_y], 'g-', marker='o')
        self.axs.plot(self.vehicle_position_x, self.vehicle_position_y, 'b--')
        sp = 0.05
        

        # Adds point index to the graphic
        idx = 0
        for i,j in zip(self.reference_profile_waypoints_x, self.reference_profile_waypoints_y):
             angle_in_deg = np.rad2deg(self.reference_profile_waypoints_a[idx])
             self.axs.annotate(round(angle_in_deg,2), xy=(i+sp,j+sp))
             idx += 1

        # Add segment length at initial point
        wp_idx = 0
        next_wp_idx = 0
        for i,j in zip(self.reference_profile_waypoints_x, self.reference_profile_waypoints_y):
            next_wp_idx = wp_idx+1
            if next_wp_idx > (len(self.reference_profile_waypoints_x)-1): 
                next_wp_idx = 0

            d = utils.distance_of_two_points(self.reference_profile_waypoints_x[wp_idx], self.reference_profile_waypoints_y[wp_idx],
                 self.reference_profile_waypoints_x[next_wp_idx], self.reference_profile_waypoints_y[next_wp_idx])
            #self.axs.annotate(round(d, 1), xy=(i+sp,j+sp))
            wp_idx += 1

        # Plot recorded speed profile
        # axs[1].plot(self.recorded_waypoints_t, self.recorded_waypoints_v)
        # axs[1].set_xlim(self.recorded_waypoints_t.min(), self.recorded_waypoints_t.max())
        # axs[1].set_ylim(self.recorded_waypoints_v.min(), self.recorded_waypoints_v.max())
        # axs[1].set_xlabel('t [s]')
        # axs[1].set_ylabel('v [m/s]')

        if show_full_circuit:
            plt.show()
        else:
            plt.pause(0.001)

    def getReferenceProfile(self):
        return self.reference_profile_waypoints_x, self.reference_profile_waypoints_y, self.reference_profile_waypoints_v
