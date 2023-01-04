import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("boxes.sdf")

#length, width, height
x = 0
y = 0
z = 0.5
pyrosim.Send_Cube(name="Box", pos=[x,y,z] , size=[1,1,1])

pyrosim.Send_Cube(name="Box2", pos=[x,y,z] , size=[1,1,1])

pyrosim.End()