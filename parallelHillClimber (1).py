from solution import SOLUTION
import constants as c
import copy as copy
import os as os
import time as time

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

        self.Evaluate(self.parents)
            

        for currentGeneration in range(c.numberofGenerations):
            self.Evolve_For_One_Generation()

            if currentGeneration == c.numberofGenerations - 1:
                self.Show_Best()

        SOLUTION.Start_Simulation("GUI")
    
    def Evolve_For_One_Generation(self):

        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        self.Select()
    
    def Spawn(self):

        self.children = {}

        for i in self.parents:
            self.children[i] = copy.deepcopy(self.parents[i])
            self.child = self.children[i]

            self.children[i].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1 
        

    def Mutate(self):
        for i in self.children:
            self.children[i].Mutate()

    def Evaluate(self, solutions):
   
        for i in solutions:
            solutions[i].Start_Simulation("DIRECT")
        
        for i in solutions:
            solutions[i].Wait_For_Simulation_To_End()
            print("YYYYYYY")
            print(solutions[i].fitness)


    def Select(self):

        for i in self.parents:
            if self.children[i].fitness > self.parents[i].fitness:
                self.parents[i] = self.children[i]

    def Print(self):
        printID = self.nextAvailableID
        print("/n")
        print("Parent Fitness: ")
        print(self.parents[printID].fitness)
        print("Child Fitness: ")
        print(self.child.fitness)
        pass

    def Show_Best(self):

        for i in self.parents:
            if self.children[i].fitness > self.parents[i].fitness:
                self.parents[i] = self.children[i]
 
    def Print(self):
        for i in self.parents:
            print("")
            print("PARENT FITNESS: " + str(self.parents[i].fitness) + ", CHILD FITNESS:" + str(self.children[i].fitness))
            print("")

    def Show_Best(self):
        bestfitness = 10000000
        bestparent2 = None
        for i in self.parents:
            if self.parents[i].fitness < bestfitness:
                bestparent2 = self.parents[i]
                bestfitness = self.parents[i].fitness
            
        bestparent2.Start_Simulation("GUI")




