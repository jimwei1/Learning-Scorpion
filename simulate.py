import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as numpy

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#gravity
p.setGravity(0,0,-9.8)

#create floor
planeId = p.loadURDF("plane.urdf")


p.loadSDF("world.sdf")

robotID = p.loadURDF("body.urdf")

#Simulation setup
pyrosim.Prepare_To_Simulate(robotID)

#numpy vector to save sensor values, same iterations as for loop
backLegSensorValues = numpy.zeros(100)


for i in range(100):
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    #print(backLegTouch)

    time.sleep(1/600)
    #print(i)

print("BACKLEG SENSOR VALUES")
print(backLegSensorValues)
numpy.save('/Users/jim/Documents/GitHub/CS-396-Artificial-Life-Bots/data/test.npy', backLegSensorValues)


p.disconnect()