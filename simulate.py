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
frontLegSensorValues = numpy.zeros(100)


for i in range(100):
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

    pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = "Torso_BackLeg", controlMode = p.POSITION_CONTROL, targetPosition = 0.0, maxForce = 500)

    time.sleep(1/600)



    
numpy.save('/Users/jim/Documents/GitHub/CS-396-Artificial-Life-Bots/data/backLegSensorValues.npy', backLegSensorValues)
numpy.save('/Users/jim/Documents/GitHub/CS-396-Artificial-Life-Bots/data/frontLegSensorValues.npy', frontLegSensorValues)


p.disconnect()