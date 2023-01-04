import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim

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

for i in range(1000):
    p.stepSimulation()
    backLegTouch = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    print(backLegTouch)

    time.sleep(1/600)
    #print(i)

p.disconnect()