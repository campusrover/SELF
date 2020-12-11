Project Title: Smart Efficient Line Follower (SELF)
Authors: Rachel Lese, Tiff Lyman


OVERVIEW:

This project uses reinforcement learning to direct a robot to follow a line.
It uses the same imaging as the original line follower, except instead of PID
it has a set of possible angular and linear velocities with a weight attributed
to each one. The velocity with the highest weight is what the robot uses.
These weights change over time as the robot takes in more data to influence the 
weights.

USAGE:

To run the program, launch learnLine.launch in Gazebo and then run 
Final_Project.py in the code editor
