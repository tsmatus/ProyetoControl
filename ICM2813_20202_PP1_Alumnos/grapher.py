from matplotlib import pyplot as plt
import numpy as np

def pos_vel(time, pos, vel, pwm):
    
    plt.plot(time, pos, 'r')
    plt.plot(time, vel, 'b')
    plt.plot(time, pwm, 'g')
    plt.show()