from simulation import SIMULATION
import sys as sys
import os as os

directOrGui = sys.argv[1]
solutionID = sys.argv[2]

simulation = SIMULATION(directOrGui, solutionID)
simulation.Run()
simulation.Get_Fitness()