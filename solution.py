import numpy as numpy

class SOLUTION:

    def __init__(self): #constructor
        
        self.weights = numpy.random.rand(3, 2)
        self.weights = self.weights * 2 - 1