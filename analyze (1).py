import numpy as numpy
import matplotlib.pyplot as pyplot

#backLegSensorValues = numpy.load('/Users/jim/Documents/GitHub/CS-396-Artificial-Life-Bots/data/backLegSensorValues.npy')
#frontLegSensorValues = numpy.load('/Users/jim/Documents/GitHub/CS-396-Artificial-Life-Bots/data/frontLegSensorValues.npy')

#pyplot.plot(backLegSensorValues, label = 'backLeg', linewidth=4)
#pyplot.plot(frontLegSensorValues, label = 'frontLeg')

sinTargetAngles = numpy.load('/Users/jim/Documents/GitHub/CS-396-Artificial-Life-Bots/data/sinTargetAngles.npy')

pyplot.plot(sinTargetAngles)
pyplot.axis('tight')
pyplot.show()

#pyplot.legend()
#pyplot.show()

