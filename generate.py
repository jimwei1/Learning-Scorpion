import pyrosim.pyrosim as pyrosim



#length, width, height
x = 0
y = 0
z = 0

#pyrosim.Send_Cube(name="Box2", pos=[x +0.5 ,y,z+1] , size=[1,1,1])


"""
for i in range(5):
    for o in range (5):
        for p in range (5):
            pyrosim.Send_Cube(name="25Box", pos=[x + i*2,y + o*2,z + p*2] , size=[1,1,1])
"""

#E. Joints


def Create_World():
    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name="Box", pos=[x,y,z] , size=[1,1,1])
    pyrosim.End()

