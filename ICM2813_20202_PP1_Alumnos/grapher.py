from matplotlib import pyplot as plt
import numpy as np

def pos_vel(time, pos, vel, pwm):
    ref = np.ones(len(time)) * 90
    ref0 = np.zeros(len(time)) 
    plt.plot(time, ref, color="m", ls='dotted', label="Referencia 90º")
    plt.plot(time, ref0, color="c", ls='dotted', label="0º")
    plt.plot(time, pos, 'r' , label="Posición")
    plt.plot(time, vel, 'b', label = "Velocidad")
    plt.plot(time, pwm, 'g', label = "PWM")
    plt.legend()
    plt.show()