from solution import SOLUTION
import constants as c
import copy
import os

class PARALLEL_HILL_CLIMBER:

    def __init__(self):
        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")
        os.system("rm body*.urdf")
        self.tempFile = c.populationSize + "_" + c.numberofGenerations + "_" + c.seed
        os.system(f"rm temp\{self.tempFile}.txt")

        self.nextAvailableID = 0
        self.parents = {}
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1 

    def Evolve(self):
        self.Evaluate(self.parents)
        self.Save_Best()
        self.Show_Best()
        for currentGeneration in range(c.numberofGenerations):
            self.Evolve_For_One_Generation(currentGeneration)
            self.Save_Best()

    def Evolve_For_One_Generation(self, gen):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print(gen)
        self.Select()

    def Evaluate(self, solutions):
        for i in solutions.values():
            i.Start_Simulation("DIRECT")
        for i in solutions.values():
            i.Wait_For_Simulation_To_End()

    def Spawn(self):
        self.children = {}
        for key in self.parents.keys():
            self.children[key] = copy.deepcopy(self.parents[key])
            self.children[key].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

    def Mutate(self):
        for child in self.children.values():
            child.Mutate()

    def Select(self):
        for key in self.parents.keys(): 
            if self.children[key].fitness > self.parents[key].fitness:
                self.parents[key] = self.children[key]

    def Show_Best(self):
        best = self.parents[0]
        if len(self.parents) != 1:
            for parent in self.parents.values():
                if parent.fitness > best.fitness:
                    best = parent

        best.Start_Simulation("GUI")

    def Save_Best(self):
        bestFitness = self.parents[0].fitness
        if len(self.parents) != 1:
            for parent in self.parents.values():
                if parent.fitness > bestFitness:
                    bestFitness = parent.fitness
        
        f = open(f"temp/{self.tempFile}.txt", "a")
        f.write(f"{bestFitness}\n")
        f.close()

    def Print(self):
        pass