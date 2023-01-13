import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as numpy
import random as random

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

timevalue = 1000
anchortimevalue = 1000

#numpy vector to save sensor values, same iterations as for loop
backLegSensorValues = numpy.zeros(timevalue)
frontLegSensorValues = numpy.zeros(timevalue)


for i in range(timevalue):
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")


    #pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = b'Torso_BackLeg', controlMode = p.POSITION_CONTROL, targetPosition = -1, maxForce = 500)
    #pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = b'Torso_FrontLeg', controlMode = p.POSITION_CONTROL, targetPosition = 1, maxForce = 500)

        #random
    #pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = b'Torso_BackLeg', controlMode = p.POSITION_CONTROL, targetPosition = random.uniform(-1.57, 1.57), maxForce = 50)
    #pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = b'Torso_FrontLeg', controlMode = p.POSITION_CONTROL, targetPosition = random.uniform(-1.57, 1.57), maxForce = 50)

        #sin
    targetAngles = numpy.sin((6.28 * anchortimevalue) / timevalue)

    #pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = b'Torso_BackLeg', controlMode = p.POSITION_CONTROL, targetPosition = numpy.sin(targetAngles), maxForce = 50)
    #pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = b'Torso_FrontLeg', controlMode = p.POSITION_CONTROL, targetPosition = numpy.sin(targetAngles), maxForce = 50)

    time.sleep(1/600)



    
numpy.save('/Users/jim/Documents/GitHub/CS-396-Artificial-Life-Bots/data/backLegSensorValues.npy', backLegSensorValues)
numpy.save('/Users/jim/Documents/GitHub/CS-396-Artificial-Life-Bots/data/frontLegSensorValues.npy', frontLegSensorValues)


p.disconnect()