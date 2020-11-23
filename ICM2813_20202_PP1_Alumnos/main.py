from ICM2813.Server import Motor
import numpy as np
import time
from grapher import *
class MiControlador(Motor):
    def __init__(self):
        self.stop()        # detener simualción al principio
        super().__init__() # no modificar
        # Las siguientes variables son de ejemplo y puede agregar o eliminar según lo necesite.
        # Estas variables son útiles para almacenar valores entre cada iteración de su controlador
        self.t = []
        self.theta = []
        self.theta_continuos = []
        self.loop = 0
        

    def control(self, theta, t):
        '''
        Esta función es la única que se requiere para poder correr el programa. Si lo desea puede agregar más  
        funciones a la clase MiControlador. Esta función recibe como entrada (ya calculada) lo siguiente:
        * theta: Que corresponde al ángulo del motor en radianes (+/-pi)
        * t: tiempo entre cada vez que se le actualizan los comandos al robot (50 ms)
        A continuación se presenta un código para mover el motor en forma continua que debe modificar
        para implementar su controlador PID
        '''
        
        # Definir PWM a aplicar en el motor (voltaje)
        pwm_motor = 100
        
        
        # Almacenar variables que desee aquí
        # print("{:.3f},\t{:.2f}".format(t,np.rad2deg(theta)))
        if t!=0:
            self.t.append(t)
            self.theta.append(np.rad2deg(theta))
            
            if np.rad2deg(theta) < 0:
                theta = 360 + np.rad2deg(theta)
            else:
                theta = np.rad2deg(theta)

            theta += 360.0*self.loop

            if len(self.theta_continuos) > 1:
               
                if (self.theta_continuos[-1] - theta) > 350.0:
                    self.loop += 1
                    theta += 360.0
                elif (self.theta_continuos[-1] - theta) < -350.0:
                    self.loop -= 1
                    theta -= 360.0
            self.theta_continuos.append(theta)

        #Detender simulación después de un tiempo
        if t > 10:
            self.stop()



        return pwm_motor

m = MiControlador()
m.run()
sim_time = np.array(m.t)

continuous_list = m.theta_continuos

delta_continous_list = list(map(lambda x: x - continuous_list[0], continuous_list))
theta = np.array(delta_continous_list)
vel = theta / sim_time
pos_vel(sim_time, theta, vel, 100)







