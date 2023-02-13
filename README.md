# CS 396 Artificial Life Assignment 6

## Overview

This is a submission for Assignemnt 6 for CS 396: Artificial Life at Northwestern University, Winter 2023 by Jim Wei.

In Assignment 6, I created a program that generates a kinematic chain (a chain of links connected by motorized joints).
This chain has a random number (between 8 to 12) of randomly shaped links with random sensor placement along the chain
of 4 sensors and motors connected to these sensors. Links without sensors are still connected to each other, but won't
be motorized. Links with sensors are colored green, while links without sensors are cyan.

## Video Example:

https://youtu.be/AWV2cExR79g 

## How to Run:

Download repository, run search.py and it will generate one snake for viewing :). Must have python3 and pybullet installed.

## Notes:

Snake spawns to the left of the camera, might have to pan camera to the left to see him. Cube generated on origin can be tossed away by the mouse. It is not part of the snake.

## References:
Base code from: https://www.reddit.com/r/ludobots


