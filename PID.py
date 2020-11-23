from ICM2813.Server import Motor
import numpy as np
import time
from grapher import *
class MiControlador(Motor):
    def __init__(self, kp, kd, ki):
        self.stop()        # detener simualción al principio
        super().__init__() # no modificar
        # Las siguientes variables son de ejemplo y puede agregar o eliminar según lo necesite.
        # Estas variables son útiles para almacenar valores entre cada iteración de su controlador
        self.t = []
        self.theta = []
        self.pwm = []
        self.theta_continuos = []

        ## Definición de ganancias 
        self.ref = 90
        self.ref_inicial = 0
        self.kp = kp
        self.kd = kd
        self.ki = ki

        ## Variables para errores.
        self.error_anterior = 0
        self.delta_tiempo = 0.05
        self.errores_anteriores = []
        self.error_acumulado = 0
        self.max_integral = 400

        ## Periodo de simulación
        self.periodo = 10

        ##Variable para resetear el integral al cambio de escalón.
        self.switch = True

    def control(self, theta, t):

        ## Transfomar el angulo a grados. 
        theta = np.rad2deg(theta)
    
        ##Control de flujo para simular el cambio en escalón
        if t > 5:
            ref = self.ref
            if self.switch:
                self.error_acumulado = 0.0
                self.switch = False
        else:
            ref = self.ref_inicial

        ##Definición del error
        error = float(ref - theta)

        ## definición para derivativo Kd
        derivado = (error-self.error_anterior)/self.delta_tiempo
        ## almacenar los derivativos anteriores para filtrar con promedios 
        self.errores_anteriores.append(derivado)
        
        ## Filtra por promedio de los ultimos 2 valores y actualiza el derivativo.
        suma = 0
        valores_promedio = 3
        if len(self.errores_anteriores) >= valores_promedio:
            for indice in range(1, valores_promedio):
                suma += self.errores_anteriores[-indice]
               
            derivado = suma/valores_promedio
        ## Definición de control Integral
        ## Maximo valor que almacenará el Ki
        
        if self.error_acumulado > self.max_integral:
            integral = self.max_integral
        elif self.error_acumulado < - self.max_integral:
            integral = - self.max_integral
        else: 
            integral = self.error_acumulado

        ## Actualización del error acumulado y el anterior 
        self.error_acumulado += error * self.delta_tiempo
        self.error_anterior = theta
        
        ## Actualización de valor de salida del controlador, PWM con control PID
        pwm_motor = error * self.kp + derivado * self.kd + integral * self.ki

        ## Actualización de valores para gráficos.
        if t!=0:
            self.t.append(t)
            self.theta.append(np.rad2deg(theta))
            self.theta_continuos.append(theta)
            self.pwm.append(pwm_motor)

        #Detender simulación después del tiempo de simulación
        if t > self.periodo:
            self.stop()
        
        ##Salida del controlador
        return pwm_motor


## Definición de valores de ganancias utilizando método de Ensayo y error. 
kp = 2
kd = 0.03
ki = 0.1

## instancia de un controlador
m = MiControlador(kp, kd, ki)
##Correr la simulación
m.run()


## funciones para graficar. 
sim_time = np.array(m.t)
theta = np.array(m.theta_continuos)
pwm = np.array(m.pwm)
vel = theta / sim_time
pos_vel(sim_time, theta, vel, pwm)

