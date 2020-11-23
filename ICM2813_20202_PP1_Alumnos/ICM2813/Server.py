import time
import functools
import numpy as np
import random
import sys

from ICM2813 import b0RemoteApi
from ICM2813.Motor import DCMotor
from ICM2813.step import step_model

from functools import reduce 
import threading
import datetime

class Motor:
    def __init__(self):
        # Cliente B0
        self.node = "icm2813"
        self.channel = "icm2813_motor"
        self.client = b0RemoteApi.RemoteApiClient(self.node , self.channel)
        self.client.doNextStep=True
        self.simTime = 0
        self.__running = True

        # Estado incial del motor.
        self._i = 0.
        self._theta = 0.

        # VARIABLES DEL ROBOT.
        self._motor = 'joint1'
        
        print("Inicializando Motor")
        if self.client._clientId != -1:
            self.client.runInSynchronousMode = True
            self.client.simxSynchronous(True)

            self._motorHandle = self.client.simxGetObjectHandle(self._motor , self.client.simxServiceCall())[1]
            self._dt = 0.05 
        else:
            raise Exception()


        self.stopSim = False

        
    def running(self):
        return self.__running
    def stop(self):
        self.__running = False

    def simulationStepStarted(self, msg):
        simTime=msg[1][b'simulationTime']
        # print('Simulation step started at simulation time: ',simTime)
        
    def simulationStepDone(self, msg):
        simTime=msg[1][b'simulationTime']
        # print('Simulation step done at simulation time: ',simTime)
        self.simTime = simTime
        self.client.doNextStep=True
    
    def control(self,theta,t):
        return 0


    def run(self):
        t = 0
        #self.client.simxStartSimulation(self.client.simxServiceCall())
        self.client.simxGetSimulationStepStarted(self.client.simxDefaultSubscriber(self.simulationStepStarted))
        self.client.simxGetSimulationStepDone(self.client.simxDefaultSubscriber(self.simulationStepDone))
        self.client.simxStartSimulation(self.client.simxServiceCall())
        
        
        while self.running():

            if self.client.runInSynchronousMode:
                while not self.client.doNextStep:
                    # Read data
                    self._theta = self.client.simxGetJointPosition(self._motorHandle, self.client.simxServiceCall())[1]
                    torque = self.client.simxGetJointForce(self._motorHandle, self.client.simxServiceCall())[1]
                    self._theta += random.gauss(0, 0.0087)
                    # Controller
                    voltage = self.control(self._theta,t)
                    # Motor Model
                    u = np.array([voltage, torque])
                    x = np.array([self._i, self._theta])
                    x_aux = step_model(DCMotor, u, t, self._dt , x)
                    self._i = x_aux[1]
                    self.client.simxSetJointTargetVelocity(self._motorHandle, x_aux[0] , self.client.simxServiceCall())
                    t += self._dt
                    # SpinOnce Simulation
                    self.client.simxSpinOnce()

                self.client.doNextStep=False
                self.client.simxSynchronousTrigger()
            
             
 
        _, state = self.client.simxGetSimulationState(self.client.simxServiceCall())
        if state != 0:
            self.client.simxStopSimulation(self.client.simxServiceCall())
        self.client.__exit__()

    
        print("# --- SIMULACION TERMINADA --- #")
        converted = datetime.timedelta(seconds=self.simTime)
        print("H : M : S")
        print("Su Motor se ha demorado ", converted, "en completar la simulacion.")
        print("Segundos : {}".format(self.simTime))
        

if __name__ == "__main__":
    m = Motor()
    m.run()
    del(m)