from solution import SOLUTION
import constants as c
import copy as copy

class HILL_CLIMBER:

    def __init__(self) -> None: #constructor

        self.parent = SOLUTION()

    def Evolve(self):

        self.parent.Evaluate()

        for currentGeneration in range(0, c.numberofGenerations):
            self.Evolve_For_One_Generation()
    
    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.child.Evaluate()
        self.Select()
    
    def Spawn(self):
        self.child = copy.deepcopy(self.parent)

    def Mutate(self):
        self.child.Mutate()

    #might be an issue here, L Step 52
    def Evaluate(self):
        pass

    def Select(self):
        pass


