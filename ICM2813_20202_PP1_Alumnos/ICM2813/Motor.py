
import numpy as np

Ra = 5      #Resistencia de Armadura.
La = 0.009    #Inductancia de Armadura.
Jeq = 0.008     #Momento de inercia equivalente.
B = 0.02  #Constante de roce dinamico.
Kt = 1.9108      #Constante del motor.

'''
def DCMotor(t, x, u):
    # x[0] -> omega
    # x[1] -> i
    # u[0] -> voltaje
    # u[1] -> T_load

    return np.array([-(B/Jeq)*x[0] - (Kt/Jeq)*x[1] ,
                     (Kt/La)*x[0] - (Ra/La)*x[1] + (1/La)*u])
'''

def DCMotor(t, x, u):
    # x[0] -> i
    # x[1] -> omega
    # u[0] -> voltaje
    # u[1] -> T_load

    return np.array([ -Ra/La * x[0]  - Kt/La * x[1] + 1/La * u[0] ,
                     Kt/Jeq * x[0] - B/Jeq * x[1] - 1/Jeq*u[1]])           
                    
def Y(x):

    return np.array([Kt*x[0], x[1]])