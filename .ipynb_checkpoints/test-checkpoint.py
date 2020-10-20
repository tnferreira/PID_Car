from datetime import datetime
from matplotlib import pyplot
from matplotlib.animation import FuncAnimation
from random import randrange
import time

x_data, y_data = [], []
pyplot.ion()
figure = pyplot.figure()
line, = pyplot.plot_date(x_data, y_data, '-')

def update(frame):
    x_data.append(datetime.now())
    y_data.append(randrange(0, 100))
    line.set_data(x_data, y_data)
    figure.gca().relim()
    figure.gca().autoscale_view()
    time.sleep(.25) # keep refresh rate of 0.25 seconds
    return line,

animation = FuncAnimation(figure, update, interval=200, blit=True)

pyplot.show()

