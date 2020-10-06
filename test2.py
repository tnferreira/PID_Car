import time
from datetime import datetime
from matplotlib import pyplot
from matplotlib.animation import FuncAnimation
from random import randrange

x_data, y_data = [], []

figure = pyplot.figure()
line, = pyplot.plot_date(x_data, y_data, '-')

def update(frame):
    x_data.append(datetime.now())
    y_data.append(randrange(0, 100))
    line.set_data(x_data, y_data)
    figure.gca().relim()
    figure.gca().autoscale_view()
    return line,

animation = FuncAnimation(figure, update, interval=200)

#pyplot.show(block=False)
while(True):
    print("..")
    time.sleep(0.1)
    # draw the plot
    pyplot.draw() 
    pyplot.pause(0.01) #is necessary for the plot to update for some reason
    #pyplot.show()