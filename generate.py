import pyrosim.pyrosim as pyrosim



#length, width, height
x = 1
y = 1
z = 1

#pyrosim.Send_Cube(name="Box2", pos=[x +0.5 ,y,z+1] , size=[1,1,1])


"""
for i in range(5):
    for o in range (5):
        for p in range (5):
            pyrosim.Send_Cube(name="25Box", pos=[x + i*2,y + o*2,z + p*2] , size=[1,1,1])
"""

#E. Joints


def Generate_Body():
    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name="Box", pos=[x+2,y+2,z] , size=[1,1,1])
    pyrosim.End()

    pyrosim.Start_URDF("body.urdf")
    pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[1,1,1])
    pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1,0,1])
    pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5] , size=[1,1,1])
    pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2,0,1])
    pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5] , size=[1,1,1])
    

    #7 Leg Thing
    """
    pyrosim.Send_Cube(name="Link0", pos=[0,0,0.5] , size=[1,1,1])
    pyrosim.Send_Joint( name = "Link0_Link1" , parent= "Link0" , child = "Link1" , type = "revolute", position = [0,0,1])
    pyrosim.Send_Cube(name="Link1", pos=[0,0,0.5] , size=[1,1,1])
    pyrosim.Send_Joint( name = "Link1_Link2" , parent= "Link1" , child = "Link2" , type = "revolute", position = [0,0,1])
    pyrosim.Send_Cube(name="Link2", pos=[0,0,0.5] , size=[1,1,1])
    pyrosim.Send_Joint( name = "Link2_Link3" , parent= "Link2" , child = "Link3" , type = "revolute", position = [0,0.5,0.5])
    pyrosim.Send_Cube(name="Link3", pos=[0,0.5,0] , size=[1,1,1])
    """

    pyrosim.End()

def Generate_Brain():
    pyrosim.Start_NeuralNetwork("brain.nndf")
    pyrosim.Send_Sensor_Neuron(name = 0, linkName = "Torso")
    pyrosim.End()


Generate_Body()
Generate_Brain()