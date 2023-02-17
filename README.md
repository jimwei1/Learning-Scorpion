# CS 396 Artificial Life Assignment 7

## Overview

This is a submission for Assignemnt 7 for CS 396: Artificial Life at Northwestern University, Winter 2023 by Jim Wei.


prob gonna use multiple diff dictionaries to generate all the diff branches, then just generate the same way as snake. will ask in class tmrw prob

so basically we're making a function that makes a random number of legs for the snake. There should be a root + joint + root2 for each direction, and then just add an iteratable for every function within the function. I'm confused.


## Random Behavior:

**Links:** Random number between 8 to 12

**Size of Links:** Cube of x, y, z dimensions randomized from a range of 0 to 1 for each dimension.

**Placement of Sensors/Motors:** 4 Sensors placed randomly within the chain. Each sensor is placed at a different link so there will always be 4 sensors and motors.


## Video Example:

https://youtu.be/AWV2cExR79g 

## How to Run:

Download repository, run search.py and it will generate one snake for viewing :). Must have python3 and pybullet installed.

## Notes:

To revert to evolving behavior, uncomment changes in search, parallelHillClimber, and simulate.



## References:
Base code from: https://www.reddit.com/r/ludobots




