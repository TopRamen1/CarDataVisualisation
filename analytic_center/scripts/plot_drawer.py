#!/usr/bin/env python

import sys
import rospy
from core2.msg import Core2PubMsg
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime as dt
import numpy as np
from matplotlib.widgets import CheckButtons
import threading

class Odometry:
    def __init__(self):
        pass
    circuit_12V = 0 # other value in the future. now circuit it's just for example
    temperature = 0


def topic_callback(msg):
   Odometry.circuit_12V = msg.circuit_12V
   Odometry.temperature = msg.temperature
   

def animate(i, xs, ys):
    # Add x and y to lists
    time = dt.datetime.now()
    x_time.append(time)

    voltage = Odometry.circuit_12V
    y_voltage.append(voltage)
    temperature = Odometry.temperature
    y_temperature.append(temperature)

    
    # Draw circuit_12V
    circuit_12V_subplot.clear()
    circuit_12V_subplot.plot(x_time, y_voltage, label = voltage, linewidth=4, color = 'red')
    circuit_12V_subplot.legend(loc="upper right", fontsize = 18, prop= {'weight':'bold'})
    circuit_12V_subplot.set_title('circuit_12V', color = 'red', fontsize = 18,  loc='left')
    circuit_12V_subplot.set_xlabel('time')
    circuit_12V_subplot.set_ylabel('[V]')

    # Draw temperature
    temperature_subplot.clear()
    temperature_subplot.plot(x_time, y_temperature, label = temperature, linewidth=4,  color = 'blue')
    temperature_subplot.legend(loc="upper right", fontsize = 18, prop= {'weight':'bold'})
    temperature_subplot.set_title('temperature', color = 'blue', fontsize = 18,  loc='left')
    temperature_subplot.set_xlabel('time')
    temperature_subplot.set_ylabel('temperature')


    if len(x_time)> ARRAYS_SIZE:
        x_time.pop(0)
        y_voltage.pop(0)
        y_temperature.pop(0)
        


if __name__ == "__main__":  
    rospy.Subscriber('/external_devices_output', Core2PubMsg, topic_callback)
    rospy.init_node('analize', anonymous=True)

    x_time = []
    y_voltage = []
    y_temperature = []
    ANIMATION_INTERVAL = 500
    WINDOW_PERIOD_S = 30

    # Set window_period_s if node has been run with parm
    if (len(sys.argv)>1):
        WINDOW_PERIOD_S = int(sys.argv[1])

    ARRAYS_SIZE = WINDOW_PERIOD_S*1000/ANIMATION_INTERVAL 
        
    # Create and format figure for plot 
    fig = plt.figure('Analytical center', facecolor = 'gainsboro')
    fig.suptitle('Analytical center', fontsize = 20)
    fig.set_size_inches(18.5, 10.5, forward=True)
    plt.subplots_adjust(bottom=0.30)
    fig.subplots_adjust(hspace=0.4, wspace=0.4)
    
    # Create subplots
    checkbox_list =[]

    circuit_12V_subplot = fig.add_subplot(2, 1, 1)
    temperature_subplot = fig.add_subplot(2, 1, 2)
    circuit_12V_subplot.yaxis.tick_right()
    temperature_subplot.yaxis.tick_right()
    checkbox_list.append(circuit_12V_subplot)
    checkbox_list.append(temperature_subplot)
    
    # Make checkbuttons with all plotted lines with correct visibility
    rax = plt.axes([0, 0.8, 0.1, 0.2])
    labels = ['circuit_12V', 'temperature']
    visibility = [line.get_visible() for line in checkbox_list]
    checkbox = CheckButtons(rax, labels, visibility)

    def change_subplot_visibility(label):
        index = labels.index(label)
        checkbox_list[index].set_visible(not checkbox_list[index].get_visible())
        

    checkbox.on_clicked(change_subplot_visibility)
    
    # Start animation 
    ani = animation.FuncAnimation(fig, animate, fargs=(x_time, y_voltage), interval=ANIMATION_INTERVAL)
    plt.show()
