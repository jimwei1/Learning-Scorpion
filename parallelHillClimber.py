from solution import SOLUTION
import constants as c
import copy as copy
import os as os

class PARALLEL_HILL_CLIMBER:

    def __init__(self) -> None: #constructor

        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")

        self.nextAvailableID = 0

        self.parent = SOLUTION(self.nextAvailableID)
        self.parents = {}

        for parent in range(c.populationSize):
            
            self.parents[parent] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1
        
        
    def Evolve(self):

        for i in self.parents:
            self.parents[i].Start_Simulation("DIRECT")
            #self.parent.Evaluate("GUI")

        for currentGeneration in range(0, c.numberofGenerations):
            self.Evolve_For_One_Generation()

        if currentGeneration == c.numberofGenerations - 1:
            self.Show_Best()

        SOLUTION.Start_Simulation("GUI")
    
    def Evolve_For_One_Generation(self):

        self.Spawn()

        for i in self.children:
            self.Mutate()
        self.Evaluate(self.children)
        exit()
        self.Print()
        self.Select()
        
    
    def Spawn(self):
        #self.child = copy.deepcopy(self.parent)
        self.children = {}

        for i in self.parents:
            self.children[i] = copy.deepcopy(self.parents[i])
            self.child = self.children[i]

            #IF NO WORK, THIS (STEP 88) MIGHT HAVE TO CALL SOLUTION!
            self.children[i]._myID = self.nextAvailableID
            self.nextAvailableID += 1
        

    def Mutate(self):
        self.Spawn.child.Mutate()

    def Evaluate(self, solutions):
        #STEP 96 MIGHT BE WRONG. MIGHT NEED TO ALSO COPY THE OTHER FOR LOOP FROM EVOLVE()
   
        for i in solutions:
            solutions[i].Wait_For_Simulation_To_End()
            print("YYYYYYY")
            print(self.parents[i].fitness)


    def Select(self):
        intID = int(self.nextAvailableID) - 1
        print("INTID:")
        print(intID)
        print(self.parents[intID].fitness)
        print(self.child.fitness)
        if self.child.fitness >= self.parents[intID].fitness:
            self.parent = self.child

    def Print(self):
        printID = self.nextAvailableID
        print("/n")
        print("Parent Fitness: ")
        print(self.parents[printID].fitness)
        print("Child Fitness: ")
        print(self.child.fitness)
        pass

    def Show_Best(self):
        self.parent.Evaluate("GUI")
        quit()


