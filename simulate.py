import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as numpy
import random as random
import constants as c
from simulation import SIMULATION

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

#Values



#numpy vector to save sensor values, same iterations as for loop
backLegSensorValues = numpy.zeros(c.timevalue)
frontLegSensorValues = numpy.zeros(c.timevalue)


for i in range(c.timevalue):

    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")


    #pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = b'Torso_BackLeg', controlMode = p.POSITION_CONTROL, targetPosition = -1, maxForce = 500)
    #pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = b'Torso_FrontLeg', controlMode = p.POSITION_CONTROL, targetPosition = 1, maxForce = 500)

        #random
    #pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = b'Torso_BackLeg', controlMode = p.POSITION_CONTROL, targetPosition = random.uniform(-1.57, 1.57), maxForce = 50)
    #pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = b'Torso_FrontLeg', controlMode = p.POSITION_CONTROL, targetPosition = random.uniform(-1.57, 1.57), maxForce = 50)

        #sin
    targetAngles_Front = numpy.zeros(c.timevalue)
    targetAngles_Back = numpy.zeros(c.timevalue)
    #targetAngles[i] = numpy.sin((numpy.pi * i/1000)/4)

    targetAngles_Front[i] = c.amplitude_Front * numpy.sin(c.frequency_Front * i + c.phaseOffset_Front)
    targetAngles_Front[i] = c.amplitude_Back * numpy.sin(c.frequency_Back * i + c.phaseOffset_Back)


    #print(targetAngles[i])
    print(i)
    #numpy.save('/Users/jim/Documents/GitHub/CS-396-Artificial-Life-Bots/data/sinTargetAngles.npy', targetAngles)

    pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = b'Torso_BackLeg', controlMode = p.POSITION_CONTROL, targetPosition = numpy.sin(targetAngles_Front[i]), maxForce = 50)
    pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = b'Torso_FrontLeg', controlMode = p.POSITION_CONTROL, targetPosition = numpy.sin(targetAngles_Back[i]), maxForce = 50)

    #time.sleep(1/240)



    
numpy.save('/Users/jim/Documents/GitHub/CS-396-Artificial-Life-Bots/data/backLegSensorValues.npy', backLegSensorValues)
numpy.save('/Users/jim/Documents/GitHub/CS-396-Artificial-Life-Bots/data/frontLegSensorValues.npy', frontLegSensorValues)


p.disconnect()