from ICM2813.Server import Motor
import numpy as np
import time
from grapher import *
class MiControlador(Motor):
    def __init__(self, kp, kd):
        self.stop()        # detener simualción al principio
        super().__init__() # no modificar
        # Las siguientes variables son de ejemplo y puede agregar o eliminar según lo necesite.
        # Estas variables son útiles para almacenar valores entre cada iteración de su controlador
        self.t = []
        self.theta = []
        self.pwm = []
        self.theta_continuos = []
        self.loop = 0
        
        self.ref = 90
        self.kp = kp
        self.kd = kd

        self.valor_anterior = 0
        self.delta_tiempo = 0.05
        self.valores_anteriores = []

        self.periodo = 10

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
       
        
        
        # Almacenar variables que desee aquí
        # print("{:.3f},\t{:.2f}".format(t,np.rad2deg(theta)))

        ## Para Graficar, cambio de radianes a angulo continuo. 
        
            
        """ if np.rad2deg(theta) < 0:
            theta = 360 + np.rad2deg(theta)
        else:
            theta = np.rad2deg(theta)

        theta += 360.0*self.loop

        if len(self.theta_continuos) > 1:
            
            if (self.theta_continuos[-1] - theta) > 358.0:
                self.loop += 1
                theta += 360.0
            elif (self.theta_continuos[-1] - theta) < -358.0:
                self.loop -= 1
                theta -= 360.0 """

        theta = np.rad2deg(theta)
        print("theta:", theta)
        
        if t > 5:
            ref = self.ref
        else:
            ref = 0

        error = ref - theta

        derivado = (theta-self.valor_anterior)/self.delta_tiempo
        self.valores_anteriores.append(derivado)
        suma = 0
        valores_promedio = 6
        if len(self.valores_anteriores) >= valores_promedio:
            for indice in range(-1, -valores_promedio):
                suma += self.valores_anteriores[indice]
            derivado = suma/valores_promedio
        
        

        self.valor_anterior = theta
    
        pwm_motor = error * self.kp + derivado * self.kd 

        if t!=0: 
            self.t.append(t)
            self.theta.append(np.rad2deg(theta))
            self.theta_continuos.append(theta)
            self.pwm.append(pwm_motor)
            
        print("pwm:", pwm_motor)
        #Detender simulación después de un tiempo

        if t > self.periodo:
            self.stop()

        return pwm_motor



kp = 6
kd = 0.2

m = MiControlador(kp, kd)
m.run()
sim_time = np.array(m.t)

continuous_list = m.theta_continuos

delta_continous_list = list(map(lambda x: x - continuous_list[0], continuous_list))
delta_theta = np.array(delta_continous_list)
theta = np.array(continuous_list)
pwm = np.array(m.pwm)
vel = delta_theta / sim_time
pos_vel(sim_time, theta, vel, pwm)







