from simulation import SIMULATION
import sys as sys

directOrGUI = sys.argv[1]
solutionID = sys.argv[2]
simulation = SIMULATION(directOrGUI, solutionID)
simulation.Run()
print("SOLUTIONID:")
print(solutionID)
simulation.Get_Fitness()