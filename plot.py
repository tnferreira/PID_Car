from collections import deque
from matplotlib import pyplot as plt
import numpy as np
import time
import math


# Plots Config
# plt.style.use('fivethirtyeight')
# plt.style.use('fast')
# plt.style.use('Solarize_Light2')
# plt.style.use('bmh')


class Plot:
    def __init__(self, blit=False, sample_interval=0.01, time_span=10):
        number_samples = math.floor(time_span/sample_interval)
        self.blit = blit

        # Variables
        self.x = np.linspace(0, time_span, num=number_samples)
        self.y = [0.] * 13
        self.lines = []

        # Speed control loop
        self.speed_deque = np.zeros(number_samples)
        self.speed_set_point_deque = np.zeros(number_samples)
        self.throttle_deque = np.zeros(number_samples)
        self.brake_deque = np.zeros(number_samples)

        self.speed_proportional_term_deque = np.zeros(number_samples)
        self.speed_derivative_term_deque = np.zeros(number_samples)
        self.speed_integral_term_deque = np.zeros(number_samples)

        # Track angle control loop
        self.track_angle_deque = np.zeros(number_samples)
        self.track_angle_set_point_deque = np.zeros(number_samples)
        self.steering_deque = np.zeros(number_samples)

        self.steering_proportional_term_deque = np.zeros(number_samples)
        self.steering_derivative_term_deque = np.zeros(number_samples)
        self.steering_integral_term_deque = np.zeros(number_samples)

        # Initialization
        self.create_figures()
        self.setup_figures()

        plt.show(block=False)

        # self.t_start = time.time()

    def create_figures(self):
        lw = 1  # Line width

        # ----- Figure 1 ----- #
        self.fig1, self.fig1_axs = plt.subplots(6, 1)
        self.lines.append(self.fig1_axs[0].plot([], lw=lw, label='Speed')[0]) #0
        self.lines.append(self.fig1_axs[0].plot([], lw=2, label='Speed SP')[0])
        self.lines.append(self.fig1_axs[1].plot([], lw=lw, label='Throttle')[0])
        self.lines.append(self.fig1_axs[2].plot([], lw=lw, label='Brake')[0])

        self.lines.append(self.fig1_axs[3].plot([], lw=lw, label='Speed Proportional Term')[0])
        self.lines.append(self.fig1_axs[4].plot([], lw=lw, label='Speed Derivative Term')[0])
        self.lines.append(self.fig1_axs[5].plot([], lw=lw, label='Speed Integral Term')[0]) #6

        # ----- Figure 2 ----- #
        self.fig2, self.fig2_axs = plt.subplots(5, 1)
        self.lines.append(self.fig2_axs[0].plot([], lw=lw, label='Track Angle')[0]) #7
        self.lines.append(self.fig2_axs[0].plot([], lw=2, label='Track Angle SP')[0])
        self.lines.append(self.fig2_axs[1].plot([], lw=lw, label='Steering')[0])

        self.lines.append(self.fig2_axs[2].plot([], lw=lw, label='Steering Proportional Term')[0])
        self.lines.append(self.fig2_axs[3].plot([], lw=lw, label='Steering Derivative Term')[0])
        self.lines.append(self.fig2_axs[4].plot([], lw=lw, label='Steering Integral Term')[0]) #12

        #self.fig2_axs,
        self.figs_axs = [self.fig1_axs,  self.fig2_axs]

    def setup_figures(self):
        # Adjust axis limits
        self.adjust_axis_limits()

        # Adjust plot layout
        self.fig1.tight_layout()
        self.fig2.tight_layout()

        # Draw Canvas.
        # Note: the first draw comes before setting data
        self.fig1.canvas.draw()
        self.fig2.canvas.draw()

        # Cache the background
        if self.blit:
            self.fig1_axs_background = []
            for ax in self.fig1_axs:
                self.fig1_axs_background.append(self.fig1.canvas.copy_from_bbox(ax.bbox))

            self.fig2_axs_background = []
            for ax in self.fig2_axs:
                self.fig2_axs_background.append(self.fig2.canvas.copy_from_bbox(ax.bbox))

    def adjust_axis_limits(self):
        for fig_axs in self.figs_axs:
            for ax in fig_axs:
                ax.set_xlim(self.x.min(), self.x.max())

        self.fig1_axs[0].set_ylabel('Speed')
        self.fig1_axs[0].set_ylim([0., 30.])

        self.fig1_axs[1].set_ylabel('Throttle')
        self.fig1_axs[1].set_ylim([-1., 1])

        self.fig1_axs[2].set_ylabel('Brake')
        self.fig1_axs[2].set_ylim([-1., 1.])

        self.fig1_axs[3].set_ylabel('Speed P')
        self.fig1_axs[3].set_ylim([-1.5, 1.5])

        self.fig1_axs[4].set_ylabel('Speed I')
        self.fig1_axs[4].set_ylim([-1.5, 1.5])

        self.fig1_axs[5].set_ylabel('Speed D')
        self.fig1_axs[5].set_ylim([-1.5, 1.5])

        self.fig2_axs[0].set_ylabel('Track Angle')
        #self.fig2_axs[0].set_ylim([-180, 180])
        self.fig2_axs[0].set_ylim([-95, -85])

        self.fig2_axs[1].set_ylabel('Steering')
        self.fig2_axs[1].set_ylim([-0.5, 0.5])

        self.fig2_axs[2].set_ylabel('Steering P')
        self.fig2_axs[2].set_ylim([-1.5, 1.5])

        self.fig2_axs[3].set_ylabel('Steering I')
        self.fig2_axs[3].set_ylim([-1.5, 1.5])

        self.fig2_axs[4].set_ylabel('Steering D')
        self.fig2_axs[4].set_ylim([-1.5, 1.5])

    def update(self, car):
        car_state = car.state
        car_control = car.controls

        # Shift Buffers
        self.speed_deque = np.roll(self.speed_deque, -1)
        self.speed_set_point_deque = np.roll(self.speed_set_point_deque, -1)
        self.throttle_deque = np.roll(self.throttle_deque, -1)
        self.brake_deque = np.roll(self.brake_deque, -1)

        self.speed_proportional_term_deque = np.roll(self.speed_proportional_term_deque, -1)
        self.speed_derivative_term_deque = np.roll(self.speed_derivative_term_deque, -1)
        self.speed_integral_term_deque = np.roll(self.speed_integral_term_deque, -1)

        self.track_angle_deque = np.roll(self.track_angle_deque, -1)
        self.track_angle_set_point_deque = np.roll(self.track_angle_set_point_deque, -1)
        self.steering_deque = np.roll(self.steering_deque, -1)

        self.steering_proportional_term_deque = np.roll(self.steering_proportional_term_deque, -1)
        self.steering_derivative_term_deque = np.roll(self.steering_derivative_term_deque, -1)
        self.steering_integral_term_deque = np.roll(self.steering_integral_term_deque, -1)

        # Fill Buffers
        self.speed_deque[-1] = car_state.speed
        self.speed_set_point_deque[-1] = car.speed_controller.pid_controller.SetPoint
        self.throttle_deque[-1] = car_control.throttle
        self.brake_deque[-1] = car_control.brake

        self.speed_proportional_term_deque[-1] = car.speed_controller.pid_controller.PTerm
        self.speed_derivative_term_deque[-1] = car.speed_controller.pid_controller.Kd * \
                                               car.speed_controller.pid_controller.DTerm
        self.speed_integral_term_deque[-1] = car.speed_controller.pid_controller.Ki * \
                                             car.speed_controller.pid_controller.ITerm

        self.track_angle_deque[-1] = np.rad2deg(car.getCurrentTrackAngle())
        self.track_angle_set_point_deque[-1] = np.rad2deg(car.track_angle_controller.pid_controller.SetPoint)
        self.steering_deque[-1] = car_control.steering

        self.steering_proportional_term_deque[-1] = car.track_angle_controller.pid_controller.PTerm
        self.steering_derivative_term_deque[-1] = car.track_angle_controller.pid_controller.Kd * \
                                               car.track_angle_controller.pid_controller.DTerm
        self.steering_integral_term_deque[-1] = car.track_angle_controller.pid_controller.Ki * \
                                             car.track_angle_controller.pid_controller.ITerm

        # Links Data to Outputs
        self.y[0] = self.speed_deque
        self.y[1] = self.speed_set_point_deque
        self.y[2] = self.throttle_deque
        self.y[3] = self.brake_deque

        self.y[4] = self.speed_proportional_term_deque
        self.y[5] = self.speed_integral_term_deque
        self.y[6] = self.speed_derivative_term_deque

        self.y[7] = self.track_angle_deque
        self.y[8] = self.track_angle_set_point_deque
        self.y[9] = self.steering_deque

        self.y[10] = self.steering_proportional_term_deque
        self.y[11] = self.steering_integral_term_deque
        self.y[12] = self.steering_derivative_term_deque

        # Update Canvas
        for i in range(len(self.lines)):
            self.lines[i].set_data(self.x, self.y[i])

        # Live Limit adjustment
        # fig2_ax1.set_ylim([y[2].min(), y[2].max()])
        # fig2_ax2.set_ylim([y[5].min(), y[5].max()])
        # fig2_ax3.set_ylim([y[8].min(), y[8].max()])
        # fig2_ax4.set_ylim([y[11].min(), y[11].max()])
        # self.fig2_axs[0].set_ylim([self.y[6].min(), self.y[6].max()])
        # self.fig2_axs[1].set_ylim([self.y[7].min(), self.y[7].max()])
        # self.fig2_axs[2].set_ylim([self.y[8].min(), self.y[8].max()])
        # self.fig2_axs[3].set_ylim([self.y[9].min(), self.y[9].max()])
        # self.fig2_axs[4].set_ylim([self.y[10].min(), self.y[10].max()])
        # self.fig2_axs[5].set_ylim([self.y[11].min(), self.y[11].max()])

        if self.blit:
            # In this post http://bastibe.de/2013-05-30-speeding-up-matplotlib.html
            # it is mentioned that blit causes strong memory leakage.
            # however, I did not observe that.

            # Restore background
            for ax in self.fig1_axs_background:
                self.fig1.canvas.restore_region(ax)

            for ax in self.fig2_axs_background:
                self.fig2.canvas.restore_region(ax)

            # Redraw just the points
            self.fig1_axs[0].draw_artist(self.lines[0])
            self.fig1_axs[0].draw_artist(self.lines[1])
            self.fig1_axs[1].draw_artist(self.lines[2])
            self.fig1_axs[2].draw_artist(self.lines[3])

            self.fig1_axs[3].draw_artist(self.lines[4])
            self.fig1_axs[4].draw_artist(self.lines[5])
            self.fig1_axs[5].draw_artist(self.lines[6])

            self.fig2_axs[0].draw_artist(self.lines[7])
            self.fig2_axs[0].draw_artist(self.lines[8])
            self.fig2_axs[1].draw_artist(self.lines[9])

            self.fig2_axs[2].draw_artist(self.lines[10])
            self.fig2_axs[3].draw_artist(self.lines[11])
            self.fig2_axs[4].draw_artist(self.lines[12])

            # Fill in the axes rectangle
            for ax in self.fig1_axs:
                self.fig1.canvas.blit(ax.bbox)

            for ax in self.fig2_axs:
                self.fig2.canvas.blit(ax.bbox)

        else:
            # Redraw everything
            self.fig1.canvas.draw()
            self.fig2.canvas.draw()

        self.fig1.canvas.flush_events()
        self.fig2.canvas.flush_events()

        # print('Mean Frame Rate: {fps:.3f}FPS'.format(fps=((self.i + 1) / (time.time() - self.t_start))))
        # self.i += 1
