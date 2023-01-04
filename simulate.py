import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#gravity
p.setGravity(0,0,-9.8)

#create floor
#planeId = p.loadURDF("plane.urdf")


p.loadSDF("world.sdf")

robotID = p.loadURDF("body.urdf")

for i in range(1000):
    p.stepSimulation()
    time.sleep(1/600)
    print(i)

p.disconnect()