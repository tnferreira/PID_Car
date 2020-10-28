##########################################
### Skoods.org -> Self-Racing Car Team ###
##########################################

# import airsim
from skoods import race
from garage import pid_car

# Connect to Skoods simulation
sample_time = 0.1  # Define the sample time to perform all processing.
race = race.Race(sample_time)
race.client.reset()
speedy = True
 
### INITIALIZE CARS

'''
# OPTION A: Qualify and record waypoints
# Need to change the settings.json file. Check the JSON_examples folder
'''
art_car = pid_car.Car(race.client, race.sample_time, 'SetCarName1', race.mode_input, filename='run-fast4-old.pickle',
                      compute_sample_time=False, show_profile=False, show_pid=True)  # Give the car the name you want
cars = [art_car]

'''
# OPTION B: Race 3 cars
# Need to change the settings.json file. Check the JSON_examples folder
pid_car1 = pid_car.Car(race.client, race.sample_time, 'SetCarName1', race.mode_input, filename='run-fast4.pickle') # Give the car the name you want
pid_car2 = pid_car.Car(race.client, race.sample_time, 'SetCarName2', race.mode_input, waypoints_correction=[0, -7], filename='run-fast2.pickle') 
pid_car3 = pid_car.Car(race.client, race.sample_time, 'SetCarName3', race.mode_input, waypoints_correction=[0, -14], filename='run-fast4.pickle')
cars = [pid_car1, pid_car2, pid_car3]
'''

if race.mode_input == '1': # Record Waypoints
    cars[0].recordWaypointsToFile() # Will run only the first car to record waypoints. Change settings.json file to only one car.

elif race.mode_input == '2' or race.mode_input == '3':
    if race.mode_input == '2':  # Will run only the first car to Qualify. Change settings.json file to only one car.
        race.setNumberOfLaps(1)
        cars = [cars[0]]
    elif race.mode_input == '3':
        race.setNumberOfLaps(3)
    race.setCars(cars)
    race.setInitialTime()
    keep_racing = True

    while keep_racing:
        for each_car in cars:
            ### RUN YOUR CODE HERE
            if speedy:
                keep_racing_from_car = each_car.speedy_race()
            else:
                keep_racing_from_car = each_car.race(race.accum_time)  # keep_racing_from_car not being used, but I will leave here just in case

            ### END HERE
        race.playSimulation() # Will check for mode
        keep_racing_from_race = race.updateRaceParameters()
        keep_racing = (keep_racing_from_car and keep_racing_from_race) # you can add more interruptions if needed

art_car.saveRaceToFile('last-race.pickle')
# art_car.resetControls()
# race.client.enableApiControl(False, 'ART Car')
# race.client.reset()
