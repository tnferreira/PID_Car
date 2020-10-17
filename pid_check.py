import airsim
import time
from pid_plot import Plot
from garage import pid_car

# connect to the AirSim simulator 
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

#Start PID car
sample_time = 0.01 # Define the sample time to perform all processing.
pid_car1 = pid_car.Car(client, sample_time, 'SetCarName1', '2', filename='run-fast4.pickle') 

p = Plot(blit=True)
keep_racing_from_car = True
while(keep_racing_from_car):
    #keep_racing_from_car = pid_car1.race()
    keep_racing_from_car = pid_car1.drive(10.0, 0.0) #speed, steering
    p.update(pid_car1)
    
#restore to original state
client.reset()
client.enableApiControl(False)