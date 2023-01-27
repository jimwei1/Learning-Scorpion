from solution import SOLUTION

class HILL_CLIMBER:

    def __init__(self): #constructor

        self.parent = SOLUTION()

    def Evolve(self):

        #L Step 21
        self.parent = SOLUTION.Evaluate()


