from solution import SOLUTION
import constants as c
import copy as copy

class PARALLEL_HILL_CLIMBER:

    def __init__(self) -> None: #constructor

        self.parent = SOLUTION()

    def Evolve(self):

        self.parent.Evaluate("DIRECT")

        for currentGeneration in range(0, c.numberofGenerations):
            self.Evolve_For_One_Generation()

            if currentGeneration == c.numberofGenerations - 1:
                self.Show_Best()
    
    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.child.Evaluate("DIRECT")
        self.Print()
        self.Select()
    
    def Spawn(self):
        self.child = copy.deepcopy(self.parent)

    def Mutate(self):
        self.child.Mutate()

    #def Evaluate(self):
        #pass

    def Select(self):
        if self.child.fitness >= self.parent.fitness:
            self.parent = self.child

    def Print(self):
        print("/n")
        print("Parent Fitness: ")
        print(self.parent.fitness)
        print("Child Fitness: ")
        print(self.child.fitness)

    def Show_Best(self):
        self.parent.Evaluate("GUI")
        quit()


