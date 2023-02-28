# CS 396 Artificial Life Assignment 8

## Overview

This is a submission for Assignment 8 for CS 396: Artificial Life at Northwestern University, Winter 2023 by Jim Wei.

First, this assignment builds on Assignment 7 by adding both body and brain mutation functionality, as well as fitness graph tracking, to the robot in Assignment 7. However, the robot in assignment 7 was extremely difficult to create a functional brain for due to its unique recursive leg design. As a result, Assignment 8's robot is a Scorpion with the random sensor links having legs that touch th eground, also with both sensor and motor links attached.

![Assignment8](Assignment8.png)

The first blue cube is the Torso. Then, 13 to 15 links are attached behind the Torso cube, with 4 randomly selected links serving as the sensor and leg links.

![Fitness][Fitness.png]

Here are the fitness results per generation, with the specific seed and number of generations/populations shown in the Legend of the graph for 5 different seeds.


## Random Behavior:

**Legs** 4 randomly selected legs will contain sensor and motor neurons.

**Links:** Random number between 13 and 15 body links.

**Size of Links:** Cube of x, y, z dimensions randomized from a range of 0 to 1 for each dimension.

**Placement of Sensors/Motors:** 4 Sensors placed randomly in the Scorpion.

**Body:** The final cube, the Abdomen cube, is randomly changed to be a new size in each new iteration. This cube provides balancing and movement capabilities for the legs that touch the ground.


## Video Example:
https://youtu.be/L3ZA3SuhO1A

## How to Run:

Download repository, modify the 3 variables in **constants.py**, then run **search.py** and it will generate a certain number of Scorpion Generations/Populations for viewing :). Run **analyze.py** AFTER search.py and it will generate the fitness graph. Must have python3 and pybullet installed.

## Notes:

For the sensors, motors, and synapses, I set an overall self.counter that keeps going up (that serves as the name for the sensors and motors. Then, for each individual leg, I count the number of Sensors and Joints (Motors), as well as arrays that save the name (self.count) of the sensors and motors. Finally, I set the synapses with two for loops that iterate through range(numOfSensors) and range(numofJoints), but the names don't match, so I take the names from the sensorsArray and the jointsArray.


To revert to evolving behavior, uncomment changes in search, parallelHillClimber, and simulate.



## References:
Base code from: https://www.reddit.com/r/ludobots



