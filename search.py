from parallelHillClimber import PARALLEL_HILL_CLIMBER
import constants as c
import sys
import random

phc = PARALLEL_HILL_CLIMBER()
phc.Evolve()
input("Press Enter to Continue")
phc.Show_Best()
